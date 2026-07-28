"""Target tracking, reachability ovals, and interceptor guidance."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .math3d import Vec3, angle_between, basis_from_forward, clamp, lerp_vec
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

@dataclass(slots=True)
class PredictionOval:
    horizon_s: float
    center: Vec3
    ballistic_center: Vec3
    extremes: tuple[Vec3, Vec3, Vec3, Vec3]  # +X, -X, +Y, -Y
    reachable: tuple[bool, bool, bool, bool]
    plane_x: Vec3
    plane_y: Vec3
    plane_normal: Vec3
    radius_x: float
    radius_y: float

    @property
    def fully_reachable(self) -> bool:
        return all(self.reachable)

    def contains_projected(self, point: Vec3, tolerance: float = 1e-8) -> bool:
        """Whether a future position's camera-plane projection is contained."""
        offset = point - self.center
        x = offset.dot(self.plane_x) / max(self.radius_x, 1e-9)
        y = offset.dot(self.plane_y) / max(self.radius_y, 1e-9)
        return x * x + y * y <= 1.0 + tolerance


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


def _acceleration_support(spec: DroneSpec, forward: Vec3, direction: Vec3) -> float:
    """Maximum acceleration component allowed in one world direction."""
    axis = direction.normalized()
    if spec.flight_model in ("multirotor", "vectored_vtol"):
        horizontal_component = math.sqrt(max(0.0, 1.0 - axis.y**2))
        return math.sqrt(
            (spec.lateral_accel * horizontal_component) ** 2
            + (spec.max_accel * axis.y) ** 2
        )

    axial_projection = clamp(forward.dot(axis), -1.0, 1.0)
    forward_support = spec.max_accel * max(0.0, axial_projection)
    reverse_limit = 0.0 if spec.flight_model == "rocket" else spec.brake_accel
    reverse_support = reverse_limit * max(0.0, -axial_projection)
    lateral_projection = math.sqrt(max(0.0, 1.0 - axial_projection**2))
    return forward_support + reverse_support + spec.lateral_accel * lateral_projection


def point_is_reachable(interceptor: DroneState, point: Vec3, horizon_s: float) -> bool:
    offset = point - interceptor.position
    distance = offset.length()
    if distance <= interceptor.spec.collision_radius:
        return True
    direction = offset.normalized()
    along_speed = interceptor.velocity.dot(direction)
    effective_horizon = horizon_s
    if (
        interceptor.spec.flight_model in ("fixed_wing", "rocket")
        and interceptor.velocity.length() > 1.0
    ):
        heading = interceptor.velocity.normalized()
        turn_rate = min(
            math.radians(interceptor.spec.max_turn_rate_deg),
            interceptor.spec.lateral_accel / interceptor.velocity.length(),
        )
        turn_time = angle_between(heading, direction) / max(turn_rate, 1e-6)
        effective_horizon = max(0.0, horizon_s - turn_time * 0.55)
    available = maximum_travel_distance(
        along_speed, interceptor.spec, effective_horizon
    )
    # Reserve covers drag and the difference between ideal path length and the
    # rate-limited vehicle trajectory.
    return distance <= available * 0.96 + interceptor.spec.collision_radius


def build_prediction_ovals(
    target_position: Vec3,
    target_velocity: Vec3,
    target_spec: DroneSpec,
    interceptor: DroneState,
    observation_axis: Vec3 | None = None,
) -> tuple[PredictionOval, ...]:
    line_of_sight = (
        observation_axis
        if observation_axis is not None
        else target_position - interceptor.position
    ).normalized(Vec3(0, 0, 1))
    plane_x, plane_y, plane_normal = basis_from_forward(line_of_sight)
    target_forward = target_velocity.normalized(line_of_sight)
    result: list[PredictionOval] = []
    for horizon in PREDICTION_HORIZONS:
        ballistic = target_position + target_velocity * horizon
        scale = 0.5 * horizon**2
        positive_x = _acceleration_support(
            target_spec, target_forward, plane_x
        ) * scale
        negative_x = _acceleration_support(
            target_spec, target_forward, -plane_x
        ) * scale
        positive_y = _acceleration_support(
            target_spec, target_forward, plane_y
        ) * scale
        negative_y = _acceleration_support(
            target_spec, target_forward, -plane_y
        ) * scale

        # Component support gives a guaranteed rectangle in the camera plane.
        # A directional vehicle uses sqrt(2) to circumscribe that rectangle.
        # The camera-plane axes align with the vectored craft's horizontal and
        # projected vertical authority, so its acceleration ellipsoid needs no
        # extra inflation.
        center = (
            ballistic
            + plane_x * ((positive_x - negative_x) * 0.5)
            + plane_y * ((positive_y - negative_y) * 0.5)
        )
        containment_scale = (
            1.0
            if target_spec.flight_model in ("multirotor", "vectored_vtol")
            else math.sqrt(2.0)
        )
        radius_x = max(
            0.05,
            (positive_x + negative_x) * 0.5 * containment_scale,
        )
        radius_y = max(
            0.05,
            (positive_y + negative_y) * 0.5 * containment_scale,
        )
        extremes = (
            center + plane_x * radius_x,
            center - plane_x * radius_x,
            center + plane_y * radius_y,
            center - plane_y * radius_y,
        )
        reachable = tuple(point_is_reachable(interceptor, point, horizon) for point in extremes)
        result.append(
            PredictionOval(
                horizon,
                center,
                ballistic,
                extremes,
                reachable,  # type: ignore[arg-type]
                plane_x,
                plane_y,
                plane_normal,
                radius_x,
                radius_y,
            )
        )
    return tuple(result)


def solve_guidance(
    interceptor: DroneState,
    track: TargetTrack,
    target_spec: DroneSpec,
    observation_axis: Vec3 | None = None,
) -> GuidanceSolution | None:
    if track.position is None:
        return None

    relative = track.position - interceptor.position
    distance = relative.length()
    line = relative.normalized(Vec3(0, 0, 1))
    relative_velocity = track.velocity - interceptor.velocity
    closing_speed = -relative_velocity.dot(line)
    time_to_contact = distance / closing_speed if closing_speed > 0.05 else math.inf
    ovals = build_prediction_ovals(
        track.position,
        track.velocity,
        target_spec,
        interceptor,
        observation_axis,
    )

    selected: PredictionOval | None = None
    for oval in reversed(ovals):
        if oval.fully_reachable:
            selected = oval
            break

    if distance < 90.0 or time_to_contact < 1.8:
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
        tracked_acceleration = track.acceleration.clamp_length(
            target_spec.max_accel
        )
        aim = (
            terminal_position
            + track.velocity * lead_time
            + tracked_acceleration * (0.5 * lead_time**2)
        )
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
        aim_direction = to_aim.normalized(line)
        incoming_target = track.velocity.dot(line) < -5.0
        if incoming_target:
            # Do not command a fixed-wing interceptor to reverse and velocity-
            # match an incoming rocket. Keep nose-on closing speed and steer the
            # physical flight path through the predicted contact point.
            desired_velocity = (
                aim_direction * interceptor.spec.max_speed
            )
        else:
            desired_closing = clamp(distance * 0.95 + 6.0, 15.0, 52.0)
            desired_velocity = (
                track.velocity + aim_direction * desired_closing
            ).clamp_length(interceptor.spec.max_speed)
    else:
        desired_velocity = (
            to_aim.normalized(line) * interceptor.spec.max_speed
        )
    command = (desired_velocity - interceptor.velocity) * 2.8

    # A small proportional-navigation term damps sideways line-of-sight motion.
    lateral_relative = relative_velocity - line * relative_velocity.dot(line)
    command = command + lateral_relative * 1.25
    command_limit = interceptor.spec.max_accel
    if interceptor.spec.flight_model in ("fixed_wing", "rocket"):
        # Thrust/braking and aerodynamic turning are independent bounded force
        # components. The physics layer clamps each one to the published limit.
        command_limit = math.hypot(
            max(interceptor.spec.max_accel, interceptor.spec.brake_accel),
            interceptor.spec.lateral_accel,
        )
    command = command.clamp_length(command_limit)

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
