"""Target tracking, reachability ovals, and interceptor guidance."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .math3d import (
    Vec3,
    WORLD_UP,
    basis_from_forward,
    clamp,
    lerp_vec,
)
from .models import DroneSpec
from .physics import DroneState, maximum_travel_distance


PREDICTION_HORIZONS = (1.0, 2.0, 3.0, 5.0)
PREDICTION_EDGE_COUNT = 96
PREDICTION_EDGE_DIRECTIONS = tuple(
    (
        math.cos(math.tau * index / PREDICTION_EDGE_COUNT),
        math.sin(math.tau * index / PREDICTION_EDGE_COUNT),
    )
    for index in range(PREDICTION_EDGE_COUNT)
)


@dataclass(slots=True)
class TargetTrack:
    position: Vec3 | None = None
    last_measurement: Vec3 | None = None
    velocity: Vec3 = field(default_factory=Vec3)
    acceleration: Vec3 = field(default_factory=Vec3)
    last_time: float | None = None
    sample_count: int = 0
    position_sigma_m: float = 0.25
    velocity_sigma_mps: float = 0.50
    confidence: float = 0.0

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
        residual_length = residual.length()
        previous_velocity = self.velocity
        alpha = clamp(0.13 + dt * 0.8, 0.13, 0.24)
        beta = clamp(0.006 + dt * 0.05, 0.006, 0.018)
        self.position = predicted + residual * alpha
        self.velocity = self.velocity + residual * (beta / dt)
        raw_acceleration = (self.velocity - previous_velocity) / dt
        self.acceleration = lerp_vec(
            self.acceleration, raw_acceleration, clamp(dt * 1.8, 0.02, 0.12)
        )
        uncertainty_alpha = 1.0 - math.exp(-2.0 * dt)
        self.position_sigma_m += (
            max(0.08, residual_length) - self.position_sigma_m
        ) * uncertainty_alpha
        raw_velocity_sigma = clamp(
            residual_length / max(dt, 0.10),
            0.15,
            8.0,
        )
        self.velocity_sigma_mps += (
            max(0.15, raw_velocity_sigma) - self.velocity_sigma_mps
        ) * min(0.08, uncertainty_alpha)
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
    edge_points: tuple[Vec3, ...] = ()
    edge_reachable: tuple[bool, ...] = ()
    likely_point: Vec3 | None = None
    uncertainty_margin_m: float = 0.0
    invalid_reason: str | None = None
    approximate_radius_px: float = 0.0
    observer_position: Vec3 | None = None

    @property
    def fully_reachable(self) -> bool:
        return (
            not self.invalid_reason
            and bool(self.edge_reachable)
            and all(self.edge_reachable)
        )

    @property
    def cardinal_reachable_count(self) -> int:
        return sum(self.reachable)

    @property
    def edge_reachable_count(self) -> int:
        return sum(self.edge_reachable)

    @property
    def edge_total(self) -> int:
        return len(self.edge_reachable)

    def contains_projected(self, point: Vec3, tolerance: float = 1e-8) -> bool:
        """Whether a future position's camera-plane projection is contained."""
        if self.observer_position is not None:
            relative = point - self.observer_position
            depth = relative.dot(self.plane_normal)
            plane_depth = (
                self.center - self.observer_position
            ).dot(self.plane_normal)
            if depth > 1e-6:
                point = self.observer_position + relative * (
                    plane_depth / depth
                )
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
    reachable_total: int
    terminal_trigger: str | None = None


@dataclass(slots=True)
class _ReachabilityContext:
    position: Vec3
    velocity: Vec3
    spec: DroneSpec
    heading: Vec3
    turn_rate_rad_s: float
    collision_radius: float
    powered_time_s: float | None

    @classmethod
    def from_state(cls, interceptor: DroneState) -> "_ReachabilityContext":
        speed = interceptor.velocity.length()
        heading = interceptor.velocity.normalized(
            interceptor.forward_direction()
        )
        directional = interceptor.spec.flight_model in ("fixed_wing", "rocket")
        turn_rate = (
            min(
                math.radians(interceptor.spec.max_turn_rate_deg),
                interceptor.spec.lateral_accel / speed,
            )
            if directional and speed > 1.0
            else 0.0
        )
        return cls(
            interceptor.position,
            interceptor.velocity,
            interceptor.spec,
            heading,
            turn_rate,
            interceptor.spec.collision_radius,
            (
                interceptor.main_burn_remaining_s
                if interceptor.spec.flight_model == "rocket"
                else None
            ),
        )

    def contains(self, point: Vec3, horizon_s: float) -> bool:
        dx = point.x - self.position.x
        dy = point.y - self.position.y
        dz = point.z - self.position.z
        distance_squared = dx * dx + dy * dy + dz * dz
        if distance_squared <= self.collision_radius * self.collision_radius:
            return True
        distance = math.sqrt(distance_squared)
        inverse_distance = 1.0 / distance
        direction_x = dx * inverse_distance
        direction_y = dy * inverse_distance
        direction_z = dz * inverse_distance
        along_speed = (
            self.velocity.x * direction_x
            + self.velocity.y * direction_y
            + self.velocity.z * direction_z
        )
        effective_horizon = horizon_s
        if self.turn_rate_rad_s > 0.0:
            heading_dot = clamp(
                self.heading.x * direction_x
                + self.heading.y * direction_y
                + self.heading.z * direction_z,
                -1.0,
                1.0,
            )
            turn_time = math.acos(heading_dot) / max(
                self.turn_rate_rad_s,
                1e-6,
            )
            effective_horizon = max(0.0, horizon_s - turn_time * 0.55)
        available = maximum_travel_distance(
            along_speed,
            self.spec,
            effective_horizon,
            self.powered_time_s,
        )
        return distance <= available * 0.96 + self.collision_radius


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
    """Whether the physical interceptor can reach one world point in time."""
    return _ReachabilityContext.from_state(interceptor).contains(
        point,
        horizon_s,
    )


def _project_to_target_plane(
    point: Vec3,
    observer: Vec3,
    plane_normal: Vec3,
    plane_depth: float,
) -> Vec3 | None:
    relative = point - observer
    depth = relative.dot(plane_normal)
    if depth <= 1e-6:
        return None
    return observer + relative * (plane_depth / depth)


def build_prediction_ovals(
    target_position: Vec3,
    target_velocity: Vec3,
    target_spec: DroneSpec,
    interceptor: DroneState,
    observation_axis: Vec3 | None = None,
    camera_focal_px: float | None = None,
    position_sigma_m: float = 0.0,
    velocity_sigma_mps: float = 0.0,
) -> tuple[PredictionOval, ...]:
    line_of_sight = (
        observation_axis
        if observation_axis is not None
        else target_position - interceptor.position
    ).normalized(Vec3(0, 0, 1))
    plane_x, plane_y, plane_normal = basis_from_forward(line_of_sight)
    target_forward = target_velocity.normalized(line_of_sight)
    reachability = _ReachabilityContext.from_state(interceptor)
    plane_depth = max(
        0.01,
        (target_position - interceptor.position).dot(plane_normal),
    )
    result: list[PredictionOval] = []
    for horizon in PREDICTION_HORIZONS:
        # Each 60 Hz calculation freezes the camera pose "now", projects the
        # future target set into that recorded image, then back-projects it onto
        # the current target-depth plane for the 3D presentation.
        future_position = target_position + target_velocity * horizon
        future_depth = (future_position - interceptor.position).dot(plane_normal)
        depth_acceleration = _acceleration_support(
            target_spec,
            target_forward,
            -plane_normal,
        )
        uncertainty = max(
            0.0,
            position_sigma_m + velocity_sigma_mps * horizon,
        )
        minimum_depth = (
            future_depth
            - 0.5 * depth_acceleration * horizon**2
            - uncertainty
        )
        invalid_reason = (
            "CAMERA CROSSING"
            if minimum_depth <= 2.0 or future_depth <= 2.0
            else None
        )
        projected_ballistic = _project_to_target_plane(
            future_position,
            interceptor.position,
            plane_normal,
            plane_depth,
        )
        ballistic = projected_ballistic or target_position
        perspective_scale = plane_depth / max(2.0, minimum_depth)
        scale = 0.5 * horizon**2
        positive_x = _acceleration_support(
            target_spec, target_forward, plane_x
        ) * scale * perspective_scale
        negative_x = _acceleration_support(
            target_spec, target_forward, -plane_x
        ) * scale * perspective_scale
        positive_y = _acceleration_support(
            target_spec, target_forward, plane_y
        ) * scale * perspective_scale
        negative_y = _acceleration_support(
            target_spec, target_forward, -plane_y
        ) * scale * perspective_scale
        projected_uncertainty = uncertainty * perspective_scale

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
            (positive_x + negative_x) * 0.5 * containment_scale
            + projected_uncertainty,
        )
        radius_y = max(
            0.05,
            (positive_y + negative_y) * 0.5 * containment_scale
            + projected_uncertainty,
        )
        extremes = (
            center + plane_x * radius_x,
            center - plane_x * radius_x,
            center + plane_y * radius_y,
            center - plane_y * radius_y,
        )
        reachable = tuple(
            reachability.contains(point, horizon)
            and invalid_reason is None
            for point in extremes
        )
        radius_px = (
            camera_focal_px * max(radius_x, radius_y) / max(plane_depth, 1e-6)
            if camera_focal_px is not None
            else 36.0
        )
        # Half-pixel angular spacing checks the complete rendered border rather
        # than assuming the four cardinal points cover diagonal escape routes.
        # The renderer uses the same 96-segment border. Reachability evaluates
        # every displayed segment direction rather than only four cardinals.
        edge_points_list: list[Vec3] = []
        edge_reachable_list: list[bool] = []
        for cosine, sine in PREDICTION_EDGE_DIRECTIONS:
            x_scale = cosine * radius_x
            y_scale = sine * radius_y
            point = Vec3(
                center.x + plane_x.x * x_scale + plane_y.x * y_scale,
                center.y + plane_x.y * x_scale + plane_y.y * y_scale,
                center.z + plane_x.z * x_scale + plane_y.z * y_scale,
            )
            edge_points_list.append(point)
            edge_reachable_list.append(
                invalid_reason is None
                and reachability.contains(point, horizon)
            )
        edge_points = tuple(edge_points_list)
        edge_reachable = tuple(edge_reachable_list)
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
                edge_points,
                edge_reachable,
                ballistic,
                projected_uncertainty,
                invalid_reason,
                radius_px,
                interceptor.position,
            )
        )
    return tuple(result)


def solve_guidance(
    interceptor: DroneState,
    track: TargetTrack,
    target_spec: DroneSpec,
    observation_axis: Vec3 | None = None,
    camera_focal_px: float | None = None,
    terminal_mode: str = "ONE_SECOND_ENVELOPE",
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
        camera_focal_px,
        track.position_sigma_m,
        track.velocity_sigma_mps,
    )

    tracked_acceleration = track.acceleration.clamp_length(
        target_spec.max_accel
    )
    track_confidence = clamp(
        track.confidence * min(1.0, track.sample_count / 30.0),
        0.0,
        1.0,
    )
    for oval in ovals:
        predicted = (
            track.position
            + track.velocity * oval.horizon_s
            + tracked_acceleration * (0.5 * oval.horizon_s**2)
        )
        projected = _project_to_target_plane(
            predicted,
            interceptor.position,
            oval.plane_normal,
            (track.position - interceptor.position).dot(oval.plane_normal),
        )
        candidate = projected or oval.center
        offset = candidate - oval.center
        normalized_x = offset.dot(oval.plane_x) / max(oval.radius_x, 1e-9)
        normalized_y = offset.dot(oval.plane_y) / max(oval.radius_y, 1e-9)
        normalized_length = math.hypot(normalized_x, normalized_y)
        if normalized_length > 0.65:
            offset = offset * (0.65 / normalized_length)
        oval.likely_point = oval.center + offset * track_confidence

    selected: PredictionOval | None = None
    for oval in reversed(ovals):
        if (
            oval.fully_reachable
            and oval.likely_point is not None
            and point_is_reachable(interceptor, oval.likely_point, oval.horizon_s)
        ):
            selected = oval
            break

    smallest = ovals[0]
    smallest_aim = smallest.likely_point or smallest.ballistic_center
    enters_smallest = point_is_reachable(
        interceptor,
        smallest_aim,
        smallest.horizon_s,
    )
    terminal_trigger: str | None = None
    if terminal_mode == "TTC_1S":
        terminal = time_to_contact <= 1.0
        if terminal:
            terminal_trigger = "TTC <= 1.0s"
    else:
        terminal = enters_smallest
        if terminal:
            terminal_trigger = "ENTER 1s OVAL"
    if time_to_contact <= 0.75:
        terminal = True
        terminal_trigger = "TTC SAFETY <= 0.75s"
    if interceptor.spec.flight_model == "rocket":
        terminal = True
        terminal_trigger = "ROCKET PROPORTIONAL NAVIGATION"

    if terminal:
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
        lead_time = clamp(
            lead_time,
            0.04,
            5.0 if interceptor.spec.flight_model == "rocket" else 1.5,
        )
        terminal_position = (
            lerp_vec(track.position, track.last_measurement, 0.72)
            if track.last_measurement is not None
            else track.position
        )
        aim = (
            terminal_position
            + track.velocity * lead_time
            + tracked_acceleration * (0.5 * lead_time**2)
        )
        mode = "TERMINAL PURSUIT"
        horizon = lead_time
    elif selected is not None:
        aim = selected.likely_point or selected.center
        mode = "WEIGHTED OVAL"
        horizon = selected.horizon_s
    else:
        # Without a complete green containment result, measured acceleration
        # is too uncertain to justify biasing toward an edge. Keep pursuing the
        # transparent unchanged-motion prediction while reporting the failure.
        aim = smallest.ballistic_center
        mode = "NO GUARANTEED OVAL"
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
    command = command + lateral_relative * 4.0
    if interceptor.spec.flight_model == "rocket":
        # The main motor is fixed at full thrust. Guidance therefore requests
        # only a bounded RCS turn toward the collision ray instead of asking a
        # solid booster to slow down or velocity-match.
        rocket_forward = interceptor.velocity.normalized(
            interceptor.forward_direction()
        )
        collision_ray = (aim - interceptor.position).normalized(line)
        steering_error = (
            collision_ray
            - rocket_forward * collision_ray.dot(rocket_forward)
        )
        steering = (
            steering_error.normalized()
            * interceptor.spec.lateral_accel
            if steering_error.length() > 1e-6
            else Vec3()
        )
        command = (
            rocket_forward * interceptor.spec.max_accel
            + steering
        )
    if interceptor.spec.flight_model in ("fixed_wing", "rocket"):
        # Directional craft must generate aerodynamic/steering force to cancel
        # the part of gravity that their current wing lift does not support.
        command = command + WORLD_UP * max(
            0.0,
            9.81 - interceptor.lift_acceleration.y,
        )
    command_limit = interceptor.spec.max_accel
    if interceptor.spec.flight_model in ("fixed_wing", "rocket"):
        # Thrust/braking and aerodynamic turning are independent bounded force
        # components. The physics layer clamps each one to the published limit.
        command_limit = math.hypot(
            max(interceptor.spec.max_accel, interceptor.spec.brake_accel),
            interceptor.spec.lateral_accel,
        )
    command = command.clamp_length(command_limit)

    reported = selected or smallest
    return GuidanceSolution(
        mode,
        aim,
        command,
        horizon,
        ovals,
        closing_speed,
        time_to_contact,
        reported.edge_reachable_count,
        reported.edge_total,
        terminal_trigger,
    )
