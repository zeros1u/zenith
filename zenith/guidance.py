"""Target tracking, reachability ovals, and interceptor guidance."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .math3d import Vec3, basis_from_forward, clamp, lerp_vec
from .models import DroneSpec
from .physics import DroneState, maximum_travel_distance


PREDICTION_HORIZONS = (1.0, 2.0, 3.0, 5.0)


@dataclass(slots=True)
class TargetTrack:
    position: Vec3 | None = None
    last_measurement: Vec3 | None = None
    velocity: Vec3 = field(default_factory=Vec3)
    acceleration: Vec3 = field(default_factory=Vec3)
    last_time: float | None = None
    sample_count: int = 0

    def update(self, measured_position: Vec3, timestamp: float) -> None:
        self.last_measurement = measured_position
        if self.position is None or self.last_time is None:
            self.position = measured_position
            self.last_time = timestamp
            self.sample_count = 1
            return

        dt = timestamp - self.last_time
        if dt <= 1e-6:
            return
        # Alpha-beta tracking rejects pixel-quantisation range jitter while still
        # following intentional target maneuvers.
        predicted = self.position + self.velocity * dt
        residual = measured_position - predicted
        previous_velocity = self.velocity
        alpha = clamp(0.13 + dt * 0.8, 0.13, 0.24)
        beta = clamp(0.006 + dt * 0.05, 0.006, 0.018)
        self.position = predicted + residual * alpha
        self.velocity = self.velocity + residual * (beta / dt)
        raw_acceleration = (self.velocity - previous_velocity) / dt
        self.acceleration = lerp_vec(
            self.acceleration, raw_acceleration, clamp(dt * 1.8, 0.02, 0.12)
        )
        self.last_time = timestamp
        self.sample_count += 1

    def coast(self, timestamp: float) -> None:
        """Propagate a temporarily unobserved track without inventing measurements."""
        if self.position is None or self.last_time is None:
            return
        dt = timestamp - self.last_time
        if dt <= 0.0:
            return
        self.position = self.position + self.velocity * dt
        self.last_time = timestamp


@dataclass(slots=True)
class PredictionOval:
    horizon_s: float
    center: Vec3
    ballistic_center: Vec3
    extremes: tuple[Vec3, Vec3, Vec3, Vec3]  # +X, -X, +Y, -Y
    reachable: tuple[bool, bool, bool, bool]

    @property
    def fully_reachable(self) -> bool:
        return all(self.reachable)


@dataclass(slots=True)
class GuidanceSolution:
    mode: str
    aim_point: Vec3
    commanded_acceleration: Vec3
    selected_horizon_s: float | None
    ovals: tuple[PredictionOval, ...]
    closing_speed: float
    time_to_contact_s: float
    reachable_count: int


def _maneuver_endpoint(
    position: Vec3,
    velocity: Vec3,
    direction: Vec3,
    spec: DroneSpec,
    horizon_s: float,
) -> Vec3:
    """Integrate a maximum lateral maneuver to create a reachable-set extreme."""
    point = Vec3(position.x, position.y, position.z)
    speed = Vec3(velocity.x, velocity.y, velocity.z)
    steps = 12
    dt = horizon_s / steps
    accel = direction.normalized() * spec.lateral_accel
    for _ in range(steps):
        speed = (speed + accel * dt).clamp_length(spec.max_speed)
        point = point + speed * dt
    return point


def point_is_reachable(interceptor: DroneState, point: Vec3, horizon_s: float) -> bool:
    offset = point - interceptor.position
    distance = offset.length()
    if distance <= interceptor.spec.collision_radius:
        return True
    direction = offset.normalized()
    along_speed = interceptor.velocity.dot(direction)
    available = maximum_travel_distance(along_speed, interceptor.spec, horizon_s)
    # Turning and drag reserve makes the claim conservative enough for the demo.
    return distance <= available * 0.96 + interceptor.spec.collision_radius


def build_prediction_ovals(
    target_position: Vec3,
    target_velocity: Vec3,
    target_spec: DroneSpec,
    interceptor: DroneState,
) -> tuple[PredictionOval, ...]:
    line_of_sight = (target_position - interceptor.position).normalized(Vec3(0, 0, 1))
    plane_x, plane_y, _ = basis_from_forward(line_of_sight)
    result: list[PredictionOval] = []
    for horizon in PREDICTION_HORIZONS:
        ballistic = target_position + target_velocity * horizon
        extremes = (
            _maneuver_endpoint(target_position, target_velocity, plane_x, target_spec, horizon),
            _maneuver_endpoint(target_position, target_velocity, -plane_x, target_spec, horizon),
            _maneuver_endpoint(target_position, target_velocity, plane_y, target_spec, horizon),
            _maneuver_endpoint(target_position, target_velocity, -plane_y, target_spec, horizon),
        )
        center = (extremes[0] + extremes[1] + extremes[2] + extremes[3]) / 4.0
        reachable = tuple(point_is_reachable(interceptor, point, horizon) for point in extremes)
        result.append(
            PredictionOval(
                horizon,
                center,
                ballistic,
                extremes,
                reachable,  # type: ignore[arg-type]
            )
        )
    return tuple(result)


def solve_guidance(
    interceptor: DroneState,
    track: TargetTrack,
    target_spec: DroneSpec,
) -> GuidanceSolution | None:
    if track.position is None:
        return None

    relative = track.position - interceptor.position
    distance = relative.length()
    line = relative.normalized(Vec3(0, 0, 1))
    relative_velocity = track.velocity - interceptor.velocity
    closing_speed = -relative_velocity.dot(line)
    time_to_contact = distance / closing_speed if closing_speed > 0.05 else math.inf
    ovals = build_prediction_ovals(track.position, track.velocity, target_spec, interceptor)

    selected: PredictionOval | None = None
    for oval in reversed(ovals):
        if oval.fully_reachable:
            selected = oval
            break

    if distance < 55.0 or time_to_contact < 1.4:
        # Closed-form constant-velocity lead. It remains a camera-only solution:
        # relative position and target velocity both come from the image track.
        speed = max(5.0, interceptor.spec.max_speed)
        a = track.velocity.length_squared() - speed * speed
        b = 2.0 * relative.dot(track.velocity)
        c = relative.length_squared()
        lead_time = distance / speed
        if abs(a) < 1e-8:
            if abs(b) > 1e-8:
                candidate = -c / b
                if candidate > 0.0:
                    lead_time = candidate
        else:
            discriminant = b * b - 4.0 * a * c
            if discriminant >= 0.0:
                roots = [
                    root
                    for root in (
                        (-b - math.sqrt(discriminant)) / (2.0 * a),
                        (-b + math.sqrt(discriminant)) / (2.0 * a),
                    )
                    if root > 0.0
                ]
                if roots:
                    lead_time = min(roots)
        lead_time = clamp(lead_time, 0.04, 1.5)
        terminal_position = (
            lerp_vec(track.position, track.last_measurement, 0.72)
            if track.last_measurement is not None
            else track.position
        )
        aim = terminal_position + track.velocity * lead_time
        mode = "TERMINAL PURSUIT"
        horizon = lead_time
    elif selected is not None:
        aim = selected.center
        mode = "OVAL CENTER"
        horizon = selected.horizon_s
    else:
        # The unchanged-trajectory point sits inside the smallest oval.
        aim = ovals[0].ballistic_center
        mode = "BALLISTIC FALLBACK"
        horizon = PREDICTION_HORIZONS[0]

    aim = Vec3(aim.x, max(2.0, aim.y), aim.z)
    to_aim = aim - interceptor.position
    if mode == "TERMINAL PURSUIT":
        desired_closing = clamp(distance * 0.78 + 4.0, 11.0, 46.0)
        desired_velocity = (
            track.velocity + to_aim.normalized(line) * desired_closing
        ).clamp_length(interceptor.spec.max_speed)
    else:
        desired_velocity = (
            to_aim.normalized(line) * interceptor.spec.max_speed
        )
    command = (desired_velocity - interceptor.velocity) * 2.8

    # A small proportional-navigation term damps sideways line-of-sight motion.
    lateral_relative = relative_velocity - line * relative_velocity.dot(line)
    command = command + lateral_relative * 1.25
    command = command.clamp_length(interceptor.spec.max_accel)

    selected_reachable = sum(selected.reachable) if selected else sum(ovals[0].reachable)
    return GuidanceSolution(
        mode,
        aim,
        command,
        horizon,
        ovals,
        closing_speed,
        time_to_contact,
        selected_reachable,
    )
