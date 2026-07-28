"""Complete sensor-to-guidance simulation state machine."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .camera import (
    CameraModel,
    Detection,
    RangeEstimate,
    detect_box,
    estimate_range,
    position_from_detection,
)
from .controls import (
    ControlMode,
    ManualCommand,
    ManualControlInput,
    ManualFlightController,
)
from .guidance import GuidanceSolution, TargetTrack, solve_guidance
from .math3d import (
    Vec3,
    clamp,
    rotate_towards,
    world_direction_from_camera,
)
from .models import DroneSpec, get_spec
from .physics import DroneState


SCENARIOS = (
    ("steady", "STEADY FLIGHT"),
    ("weave", "LATERAL WEAVE"),
    ("evasive", "EVASIVE MANEUVERS"),
    ("braking", "AIRBRAKE TEST"),
    ("rotating", "ROTATING TARGET"),
    ("rocket_attack", "ROCKET ATTACK"),
)


@dataclass(slots=True)
class SimulationConfig:
    interceptor_code: str = "TLR"
    target_code: str = "FX1"
    interceptor_position: Vec3 = field(default_factory=lambda: Vec3(0.0, 28.0, 0.0))
    target_position: Vec3 = field(default_factory=lambda: Vec3(10.0, 36.0, 240.0))
    scenario: str = "evasive"
    camera: CameraModel = field(default_factory=CameraModel)


@dataclass(slots=True)
class TelemetrySample:
    time_s: float
    true_range_m: float
    estimated_range_m: float | None
    apparent_px: float
    range_error_m: float | None
    closing_speed: float


class InterceptionSimulation:
    DRONE_SIGNAL_DELAY_S = 0.65
    ROCKET_SIGNAL_DELAY_S = 0.42
    # Includes protruding rotors/fins that are not captured perfectly by the
    # lightweight collision sphere, while the segment test still proves an
    # actual close pass rather than a multi-metre "hit."
    CONTACT_TOLERANCE_M = 0.35
    GIMBAL_SLEW_RATE_DEG_S = 120.0
    SEARCH_SLEW_RATE_DEG_S = 120.0

    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config or SimulationConfig()
        self.time_s = 0.0
        self.paused = False
        self.finished = False
        self.hit = False
        self.hit_time_s: float | None = None
        self.status = "SEARCHING"
        self.signal_progress = 0.0
        self.identity_confirmed = False
        self.visual_locked = False
        self.lost_time_s = 0.0
        self.reacquisition_count = 0
        self.sensor_occluded = False
        self.control_mode = ControlMode.AUTO
        self.manual_input = ManualControlInput()
        self.manual_controller = ManualFlightController()
        self.manual_command = ManualCommand()
        self.guidance_advisory_command = Vec3()
        self.last_event = "Simulation initialized"
        self.events: list[tuple[float, str]] = [(0.0, self.last_event)]
        self.telemetry: list[TelemetrySample] = []
        self._telemetry_timer = 0.0

        interceptor_spec = get_spec(self.config.interceptor_code)
        target_spec = get_spec(self.config.target_code)
        initial_line = (self.config.target_position - self.config.interceptor_position).normalized(
            Vec3(0, 0, 1)
        )
        self.interceptor = DroneState(
            interceptor_spec,
            self.config.interceptor_position,
            initial_line * min(24.0, interceptor_spec.max_speed * 0.38),
        )
        incoming_rocket = (
            target_spec.vehicle_type == "rocket"
            or self.config.scenario == "rocket_attack"
        )
        target_velocity = (
            Vec3(0.0, 0.0, -min(65.0, target_spec.max_speed * 0.72))
            if incoming_rocket
            else Vec3(8.0, 0.0, min(23.0, target_spec.max_speed * 0.42))
        )
        self.target = DroneState(
            target_spec,
            self.config.target_position,
            target_velocity,
            orientation=Vec3(0.0, math.pi if incoming_rocket else 0.25, 0.0),
        )
        self.camera_forward = initial_line
        self.search_anchor_forward = initial_line
        self.search_direction = initial_line
        self.search_hold_altitude_m = self.config.interceptor_position.y
        self.last_bearing_x_deg = 0.0
        self.last_bearing_y_deg = 0.0
        self.bearing_rate_x_deg_s = 0.0
        self.bearing_rate_y_deg_s = 0.0
        self._bearing_sample_valid = False
        self.detection = Detection(False, (0, 0, 0, 0), (0, 0), 0, 0, 0, 0, 0, 0, ())
        self.range_estimate: RangeEstimate | None = None
        self.track = TargetTrack()
        self.guidance: GuidanceSolution | None = None
        self.last_estimated_position: Vec3 | None = None
        self.explosion_age_s = math.inf

    @property
    def true_range_m(self) -> float:
        return self.interceptor.position.distance_to(self.target.position)

    @property
    def target_spec_available(self) -> DroneSpec | None:
        return self.target.spec if self.identity_confirmed else None

    @property
    def success_message(self) -> str:
        if self.target.spec.vehicle_type == "rocket":
            return "ROCKET INTERCEPTED SUCCESSFULLY"
        return "DRONE HIT SUCCESSFULLY"

    @property
    def controlled_vehicle(self) -> DroneState | None:
        if self.control_mode is ControlMode.INTERCEPTOR:
            return self.interceptor
        if self.control_mode is ControlMode.TARGET:
            return self.target
        return None

    def set_manual_input(self, controls: ManualControlInput) -> None:
        self.manual_input = controls

    def clear_manual_input(self) -> None:
        self.manual_input = ManualControlInput()

    def toggle_controlled_engine(self) -> bool | None:
        controlled = self.controlled_vehicle
        if controlled is None:
            self._event("Select a player vehicle before toggling its engine")
            return None
        if controlled.crashed:
            self._event(f"{controlled.spec.code} engine unavailable after impact")
            return False
        controlled.engine_enabled = not controlled.engine_enabled
        state = "started" if controlled.engine_enabled else "cut"
        self._event(f"{controlled.spec.code} engine {state} by player")
        return controlled.engine_enabled

    def cycle_control_mode(self) -> ControlMode:
        """Cycle AUTO -> own vehicle -> target vehicle -> AUTO."""
        current_vehicle = self.controlled_vehicle
        if current_vehicle is not None:
            current_vehicle.airbrake = False
        if self.control_mode is ControlMode.AUTO:
            self.control_mode = ControlMode.INTERCEPTOR
        elif self.control_mode is ControlMode.INTERCEPTOR:
            self.control_mode = ControlMode.TARGET
        else:
            self.control_mode = ControlMode.AUTO
        self.clear_manual_input()
        self.manual_command = ManualCommand()
        controlled = self.controlled_vehicle
        if controlled is not None:
            self.manual_controller = ManualFlightController()
            self.manual_controller.initialize(controlled)
        self._event(f"Control mode: {self.control_mode.value}")
        return self.control_mode

    def _event(self, message: str) -> None:
        if message == self.last_event:
            return
        self.last_event = message
        self.events.append((self.time_s, message))
        self.events = self.events[-12:]

    def toggle_sensor_occlusion(self) -> None:
        """Presentation control for proving lock-loss behavior deterministically."""
        self.sensor_occluded = not self.sensor_occluded
        state = "blocked" if self.sensor_occluded else "restored"
        self._event(f"Camera image {state} by test control")

    def _update_bearing_motion(self, dt: float, reset: bool = False) -> None:
        """Filter image-plane motion without reading target ground truth."""
        bearing_x = self.detection.bearing_x_deg
        bearing_y = self.detection.bearing_y_deg
        if self._bearing_sample_valid and not reset and dt > 1e-6:
            raw_x = clamp(
                (bearing_x - self.last_bearing_x_deg) / dt,
                -120.0,
                120.0,
            )
            raw_y = clamp(
                (bearing_y - self.last_bearing_y_deg) / dt,
                -90.0,
                90.0,
            )
            alpha = 1.0 - math.exp(-5.0 * dt)
            self.bearing_rate_x_deg_s += (
                raw_x - self.bearing_rate_x_deg_s
            ) * alpha
            self.bearing_rate_y_deg_s += (
                raw_y - self.bearing_rate_y_deg_s
            ) * alpha
        elif reset:
            self.bearing_rate_x_deg_s = 0.0
            self.bearing_rate_y_deg_s = 0.0
        self.last_bearing_x_deg = bearing_x
        self.last_bearing_y_deg = bearing_y
        self._bearing_sample_valid = True

    def _predicted_search_direction(self) -> Vec3:
        """Extrapolate the last image motion, then perform a horizon-safe scan."""
        anchor = self.search_anchor_forward.normalized(self.camera_forward)
        anchor_yaw = math.atan2(anchor.x, anchor.z)
        anchor_elevation = math.asin(clamp(anchor.y, -1.0, 1.0))
        extrapolation_s = min(1.5, self.lost_time_s)
        predicted_yaw = anchor_yaw + math.radians(
            self.last_bearing_x_deg
            + self.bearing_rate_x_deg_s * extrapolation_s
        )
        predicted_elevation = anchor_elevation + math.radians(
            self.last_bearing_y_deg
            + self.bearing_rate_y_deg_s * extrapolation_s
        )

        search_elapsed_s = max(0.0, self.lost_time_s - 1.5)
        horizontal_radius = (
            math.radians(min(160.0, 4.0 + search_elapsed_s * 18.0))
            if search_elapsed_s > 0.0
            else 0.0
        )
        vertical_radius = (
            math.radians(min(18.0, 2.0 + search_elapsed_s * 3.0))
            if search_elapsed_s > 0.0
            else 0.0
        )
        predicted_yaw += horizontal_radius * math.sin(search_elapsed_s * 1.35)
        predicted_elevation += vertical_radius * math.sin(
            search_elapsed_s * 0.91
        )
        predicted_elevation = clamp(
            predicted_elevation,
            math.radians(-35.0),
            math.radians(35.0),
        )
        horizontal = math.cos(predicted_elevation)
        return Vec3(
            math.sin(predicted_yaw) * horizontal,
            math.sin(predicted_elevation),
            math.cos(predicted_yaw) * horizontal,
        ).normalized(anchor)

    def _lost_search_command(self) -> Vec3:
        """Turn toward the predicted horizontal bearing while holding altitude."""
        if not self._bearing_sample_valid:
            return Vec3()
        spec = self.interceptor.spec
        horizontal_direction = Vec3(
            self.search_direction.x,
            0.0,
            self.search_direction.z,
        ).normalized(self.interceptor.forward_direction())
        current_forward = Vec3(
            self.interceptor.velocity.x,
            0.0,
            self.interceptor.velocity.z,
        ).normalized(self.interceptor.forward_direction())
        turn_direction = (
            horizontal_direction
            - current_forward * horizontal_direction.dot(current_forward)
        )
        turn_ramp = clamp(self.lost_time_s / 0.8, 0.0, 1.0)
        horizontal_turn = (
            turn_direction.normalized()
            * spec.lateral_accel
            * turn_direction.length()
            * turn_ramp
        )
        vertical_request = clamp(
            (self.search_hold_altitude_m - self.interceptor.position.y) * 1.4
            - self.interceptor.velocity.y * 1.1,
            -spec.lateral_accel,
            spec.lateral_accel,
        )
        return (
            horizontal_turn + Vec3(0.0, vertical_request, 0.0)
        ).clamp_length(max(spec.max_accel, spec.lateral_accel))

    @staticmethod
    def _empty_detection(depth: float = 0.0) -> Detection:
        return Detection(
            False,
            (0, 0, 0, 0),
            (0, 0),
            0,
            0,
            0,
            0,
            0,
            depth,
            (),
        )

    def _target_command(self) -> Vec3:
        spec = self.target.spec
        velocity = self.target.velocity
        scenario = self.config.scenario
        incoming_rocket = (
            spec.vehicle_type == "rocket" or scenario == "rocket_attack"
        )
        cruise_z = (
            -min(65.0, spec.max_speed * 0.72)
            if incoming_rocket
            else min(23.0, spec.max_speed * 0.42)
        )
        target_altitude = self.config.target_position.y
        stabilise = Vec3(
            0.0,
            (target_altitude - self.target.position.y) * 0.35,
            cruise_z - velocity.z,
        )
        stabilise = stabilise * 0.7
        if spec.flight_model in ("fixed_wing", "rocket"):
            stabilise = stabilise + Vec3(
                0.0,
                max(0.0, 9.81 - self.target.lift_acceleration.y),
                0.0,
            )
        self.target.airbrake = False

        if scenario == "weave":
            return (
                stabilise
                + Vec3(
                    math.sin(self.time_s * 1.55) * spec.lateral_accel * 0.62,
                    math.sin(self.time_s * 0.83) * spec.lateral_accel * 0.16,
                    0.0,
                )
            ).clamp_length(spec.max_accel)
        if scenario == "rocket_attack":
            maneuver_scale = 0.0 if spec.code == "SR1" else 0.22
            terminal_weave = Vec3(
                math.sin(self.time_s * 1.2) * spec.lateral_accel * maneuver_scale,
                math.sin(self.time_s * 0.7) * spec.lateral_accel * maneuver_scale * 0.45,
                0.0,
            )
            return (stabilise + terminal_weave).clamp_length(spec.max_accel)
        if scenario == "evasive":
            phase = int(self.time_s / 1.35) % 4
            directions = (
                Vec3(1.0, 0.32, 0.0),
                Vec3(-0.65, -0.42, 0.25),
                Vec3(-1.0, 0.28, 0.0),
                Vec3(0.72, -0.18, -0.2),
            )
            evade = directions[phase].normalized() * spec.lateral_accel * 0.82
            return (stabilise + evade).clamp_length(spec.max_accel)
        if scenario == "braking":
            cycle = self.time_s % 7.0
            if 3.0 < cycle < 5.2:
                self.target.airbrake = True
                return Vec3(
                    0.0,
                    (target_altitude - self.target.position.y) * 0.3,
                    0.0,
                )
            return stabilise.clamp_length(spec.max_accel)
        return stabilise.clamp_length(spec.max_accel)

    def _update_sensor(self, dt: float) -> None:
        if self.visual_locked and self.identity_confirmed and self.track.position is not None:
            predicted_direction = (
                self.track.position - self.interceptor.position
            ).normalized(self.camera_forward)
            self.camera_forward = rotate_towards(
                self.camera_forward,
                predicted_direction,
                math.radians(self.GIMBAL_SLEW_RATE_DEG_S) * dt,
            )
        elif self.lost_time_s > 0.0:
            # Search uses only the last observed image bearing and bearing rate.
            # It deliberately does not point at simulation truth.
            self.search_direction = self._predicted_search_direction()
            self.camera_forward = rotate_towards(
                self.camera_forward,
                self.search_direction,
                math.radians(self.SEARCH_SLEW_RATE_DEG_S) * dt,
            )

        self.detection = detect_box(
            self.target,
            self.interceptor.position,
            self.camera_forward,
            self.config.camera,
        )
        if self.sensor_occluded:
            self.detection = self._empty_detection(self.detection.camera_depth)
        incoming_rocket = self.target.spec.vehicle_type == "rocket"
        was_locked = self.visual_locked
        if self.identity_confirmed:
            # Known-model detection can remain valid at a smaller apparent size
            # than the initial generic-object / signal-query threshold.
            minimum_lock_pixels = 2.0 if was_locked else 2.5
        else:
            minimum_lock_pixels = 3.0 if incoming_rocket else 4.0
        signal_delay = (
            self.ROCKET_SIGNAL_DELAY_S
            if incoming_rocket
            else self.DRONE_SIGNAL_DELAY_S
        )
        visual_detection = self.detection.visible and max(
            self.detection.width_px, self.detection.height_px
        ) >= minimum_lock_pixels

        previous_lost_time = self.lost_time_s
        if visual_detection:
            self._update_bearing_motion(
                dt,
                reset=not was_locked and previous_lost_time > 0.25,
            )
            self.visual_locked = True
            self.lost_time_s = 0.0
            if not was_locked:
                if self.identity_confirmed:
                    self.reacquisition_count += 1
                    self._event("Visual lock reacquired by image detector")
                else:
                    self._event("Visual detector acquired an unknown aerial vehicle")
        else:
            if was_locked:
                self.search_anchor_forward = self.camera_forward
                self.search_direction = self.camera_forward
                self.search_hold_altitude_m = self.interceptor.position.y
                self._event("Visual lock lost; guidance disabled")
            self.visual_locked = False
            self.lost_time_s += dt
            self.range_estimate = None

        if self.visual_locked and not self.identity_confirmed:
            self.signal_progress += dt
            if self.status == "SEARCHING":
                self.status = "VISUAL LOCK / SIGNAL QUERY"
            if self.signal_progress >= signal_delay:
                self.identity_confirmed = True
                self.status = "IDENTITY CONFIRMED"
                self._event(f"Signal match: {self.target.spec.name}")
        elif not self.visual_locked and not self.identity_confirmed:
            self.signal_progress = max(0.0, self.signal_progress - dt * 0.5)
            self.status = "SEARCHING"

        if self.identity_confirmed and self.visual_locked:
            if not was_locked and previous_lost_time > 0.25:
                # A stale metric track must not silently become a current lock.
                # Reacquisition begins a fresh camera-derived velocity estimate.
                self.track = TargetTrack()
            # A known-model pose solver supplies orientation. Image-size-dependent
            # deterministic error keeps the simulation repeatable.
            pose_error = math.radians(
                (1.0 - self.detection.confidence) * 4.0 * math.sin(self.time_s * 2.3)
            )
            pose_estimate = Vec3(
                self.target.orientation.x + pose_error * 0.35,
                self.target.orientation.y + pose_error,
                self.target.orientation.z - pose_error * 0.2,
            )
            self.range_estimate = estimate_range(
                self.detection,
                self.target.spec,
                pose_estimate,
                self.camera_forward,
                self.config.camera,
            )
            if self.range_estimate is not None:
                estimated_position = position_from_detection(
                    self.interceptor.position,
                    self.camera_forward,
                    self.detection,
                    self.range_estimate.distance_m,
                )
                self.last_estimated_position = estimated_position
                self.track.update(estimated_position, self.time_s)

                camera_ray = Vec3(
                    math.tan(math.radians(self.detection.bearing_x_deg)),
                    math.tan(math.radians(self.detection.bearing_y_deg)),
                    1.0,
                )
                measured_bearing = world_direction_from_camera(
                    camera_ray, self.camera_forward
                )
                self.camera_forward = rotate_towards(
                    self.camera_forward,
                    measured_bearing,
                    math.radians(self.GIMBAL_SLEW_RATE_DEG_S) * dt,
                )
        elif self.identity_confirmed:
            self.status = "TARGET LOST / SEARCHING"

    def _record_telemetry(self, dt: float) -> None:
        self._telemetry_timer += dt
        if self._telemetry_timer < 0.1:
            return
        self._telemetry_timer = 0.0
        estimate = self.range_estimate.distance_m if self.range_estimate else None
        closing = self.guidance.closing_speed if self.guidance else 0.0
        self.telemetry.append(
            TelemetrySample(
                self.time_s,
                self.true_range_m,
                estimate,
                max(self.detection.width_px, self.detection.height_px),
                estimate - self.true_range_m if estimate is not None else None,
                closing,
            )
        )
        self.telemetry = self.telemetry[-600:]

    def step(self, dt: float = 1.0 / 60.0) -> None:
        if self.paused or self.finished:
            return
        dt = min(dt, 1.0 / 30.0)
        self.time_s += dt
        self.explosion_age_s += dt
        if self.hit:
            # After contact, guidance and target autopilots are disabled. Each
            # crashed body receives exactly one gravity integration per tick.
            self.interceptor.integrate(Vec3(), dt)
            self.target.integrate(Vec3(), dt)
            self.status = self.success_message
            if self.explosion_age_s > 5.0:
                self.finished = True
            return

        previous_relative = self.target.position - self.interceptor.position

        self._update_sensor(dt)
        if (
            self.identity_confirmed
            and self.visual_locked
            and self.track.position is not None
        ):
            self.guidance = solve_guidance(
                self.interceptor,
                self.track,
                self.target.spec,
                self.camera_forward,
            )
            advisory_command = (
                self.guidance.commanded_acceleration if self.guidance else Vec3()
            )
            if self.status != "INTERCEPT GUIDANCE":
                self.status = "INTERCEPT GUIDANCE"
                self._event("Oval reachability guidance active")
        elif self.visual_locked:
            # Bearing-only pursuit while the signal lookup runs.
            self.guidance = None
            advisory_command = (
                self.camera_forward * self.interceptor.spec.max_speed
                - self.interceptor.velocity
            ).clamp_length(self.interceptor.spec.max_accel)
        else:
            self.guidance = None
            advisory_command = self._lost_search_command()

        self.guidance_advisory_command = advisory_command
        if self.control_mode is ControlMode.INTERCEPTOR:
            self.manual_command = self.manual_controller.command(
                self.interceptor,
                self.manual_input,
                dt,
            )
            interceptor_command = self.manual_command.acceleration
            if self.guidance is not None:
                self.status = "PLAYER CONTROL / GUIDANCE ADVISORY"
            elif not self.visual_locked:
                self.status = "PLAYER CONTROL / SENSOR SEARCH"
        else:
            self.interceptor.airbrake = False
            interceptor_command = advisory_command
            if self.control_mode is ControlMode.AUTO:
                self.manual_command = ManualCommand()

        if self.control_mode is ControlMode.TARGET:
            self.manual_command = self.manual_controller.command(
                self.target,
                self.manual_input,
                dt,
            )
            target_command = self.manual_command.acceleration
        else:
            target_command = self._target_command()

        interceptor_yaw = (
            self.manual_command.desired_yaw_rad
            if self.control_mode is ControlMode.INTERCEPTOR
            else None
        )
        target_yaw = (
            self.manual_command.desired_yaw_rad
            if self.control_mode is ControlMode.TARGET
            else None
        )
        self.interceptor.integrate(interceptor_command, dt, interceptor_yaw)
        self.target.integrate(target_command, dt, target_yaw)
        if (
            self.config.scenario == "rotating"
            and self.control_mode is not ControlMode.TARGET
        ):
            if self.target.spec.flight_model in ("multirotor", "vectored_vtol"):
                self.target.orientation = Vec3(
                    self.target.orientation.x,
                    self.target.orientation.y + 1.45 * dt,
                    math.sin(self.time_s * 2.4) * 1.1,
                )
            else:
                # Directional craft may roll about their flight axis, but their
                # nose remains tied to their physically integrated velocity.
                self.target.orientation = Vec3(
                    self.target.orientation.x,
                    self.target.orientation.y,
                    math.sin(self.time_s * 2.4) * 0.65,
                )

        # Continuous relative segment test prevents high-speed tunnelling.
        current_relative = self.target.position - self.interceptor.position
        segment = current_relative - previous_relative
        denominator = segment.length_squared()
        closest_t = (
            clamp(-previous_relative.dot(segment) / denominator, 0.0, 1.0)
            if denominator > 1e-9
            else 0.0
        )
        closest = previous_relative + segment * closest_t
        collision_distance = (
            self.interceptor.spec.collision_radius + self.target.spec.collision_radius
            + self.CONTACT_TOLERANCE_M
        )
        if closest.length() <= collision_distance and not self.hit:
            self.hit = True
            self.hit_time_s = self.time_s
            self.status = self.success_message
            self._event(self.success_message)
            impact = (self.interceptor.position + self.target.position) * 0.5
            self.interceptor.position = impact
            self.target.position = impact
            average_velocity = (self.interceptor.velocity + self.target.velocity) * 0.5
            self.interceptor.velocity = average_velocity + Vec3(-2.0, 1.5, 0.0)
            self.target.velocity = average_velocity + Vec3(2.0, 2.5, 0.0)
            self.interceptor.crashed = True
            self.target.crashed = True
            self.interceptor.engine_enabled = False
            self.target.engine_enabled = False
            self.explosion_age_s = 0.0
            self.clear_manual_input()

        self._record_telemetry(dt)
