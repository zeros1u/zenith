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
from .guidance import (
    GuidanceSolution,
    PredictionOval,
    SharedOvalOverlap,
    TargetTrack,
    command_toward_shared_aim,
    point_is_reachable,
    shared_oval_overlap,
    solve_guidance,
)
from .math3d import (
    Vec3,
    clamp,
    world_direction_from_camera,
)
from .models import DRONE_SPECS, DroneSpec, get_spec
from .physics import DroneState


SCENARIOS = (
    ("steady", "STEADY FLIGHT"),
    ("weave", "LATERAL WEAVE"),
    ("evasive", "EVASIVE MANEUVERS"),
    ("braking", "AIRBRAKE TEST"),
    ("rotating", "ROTATING TARGET"),
    ("tricky", "TRICKY AI"),
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
    enemy_count: int = 1
    detector_backend: str = "synthetic_projection"


@dataclass(slots=True)
class TelemetrySample:
    time_s: float
    true_range_m: float
    estimated_range_m: float | None
    apparent_px: float
    range_error_m: float | None
    closing_speed: float
    estimated_target_position: Vec3 | None
    guidance_mode: str
    selected_horizon_s: float | None
    edge_reachable: int
    edge_total: int
    interceptor_burn_remaining_s: float
    interceptor_rcs_remaining_s: float
    target_burn_remaining_s: float
    target_rcs_remaining_s: float
    active_contact_id: str
    visible_contacts: int
    total_contacts: int
    priority_score: float
    target_type: str
    target_model: str
    multi_guidance_mode: str
    shared_pair: str | None
    shared_horizon_s: float | None
    target_ai_state: str
    target_ai_threat_range_m: float | None
    target_ai_closing_speed_mps: float | None


@dataclass(slots=True)
class PredictionCheck:
    """A recorded sensor-frame audit; never an input to guidance."""

    capture_time_s: float
    camera_position: Vec3
    camera_forward: Vec3
    oval: PredictionOval
    evaluated: bool = False
    result_inside: bool | None = None
    projected_truth: Vec3 | None = None


@dataclass(slots=True)
class TargetContact:
    """Independent image-derived state for one visible intruder."""

    vehicle: DroneState
    track_id: str
    spawn_altitude_m: float
    detection: Detection = field(
        default_factory=lambda: Detection(
            False, (0, 0, 0, 0), (0, 0), 0, 0, 0, 0, 0, 0, ()
        )
    )
    range_estimate: RangeEstimate | None = None
    track: TargetTrack = field(default_factory=TargetTrack)
    guidance: GuidanceSolution | None = None
    signal_progress: float = 0.0
    identity_confirmed: bool = False
    visual_locked: bool = False
    lost_time_s: float = 0.0
    reacquisition_count: int = 0
    last_estimated_position: Vec3 | None = None
    last_bearing_x_deg: float = 0.0
    last_bearing_y_deg: float = 0.0
    bearing_rate_x_deg_s: float = 0.0
    bearing_rate_y_deg_s: float = 0.0
    bearing_sample_valid: bool = False
    priority_score: float = -math.inf


class InterceptionSimulation:
    DRONE_SIGNAL_DELAY_S = 0.65
    ROCKET_SIGNAL_DELAY_S = 0.42
    # Includes protruding rotors/fins that are not captured perfectly by the
    # lightweight collision sphere, while the segment test still proves an
    # actual close pass rather than a multi-metre "hit."
    CONTACT_TOLERANCE_M = 0.42
    def __init__(self, config: SimulationConfig | None = None) -> None:
        self.config = config or SimulationConfig()
        if self.config.detector_backend != "synthetic_projection":
            raise ValueError(
                "This build has no loaded image-model weights; "
                "detector_backend must be 'synthetic_projection'."
            )
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
        self.terminal_mode = "ONE_SECOND_ENVELOPE"
        self.prediction_check: PredictionCheck | None = None
        self.last_event = "Simulation initialized"
        self.events: list[tuple[float, str]] = [(0.0, self.last_event)]
        self.telemetry: list[TelemetrySample] = []
        self._telemetry_timer = 0.0

        interceptor_spec = get_spec(self.config.interceptor_code)
        target_spec = get_spec(self.config.target_code)
        initial_line = (self.config.target_position - self.config.interceptor_position).normalized(
            Vec3(0, 0, 1)
        )
        if interceptor_spec.flight_model == "rocket":
            # The one-shot interceptors use a pre-launch ballistic loft so the
            # finite booster and small RCS budget do not pretend to hover. The
            # loft stays inside the fixed nose-camera field of view.
            launch_loft = 0.25 if interceptor_spec.code == "SR1" else 0.20
            launch_direction = (
                initial_line + Vec3(0.0, launch_loft, 0.0)
            ).normalized(initial_line)
        elif interceptor_spec.flight_model in ("multirotor", "vectored_vtol"):
            # Drones enter in level flight. The target may be above or below
            # the interceptor, but that image elevation must become a later
            # guidance maneuver rather than an unexplained vertical spawn
            # velocity.
            launch_direction = Vec3(
                initial_line.x,
                0.0,
                initial_line.z,
            ).normalized(Vec3(0.0, 0.0, 1.0))
        else:
            # A fixed wing's velocity and nose cannot be decoupled. A shallow
            # initial flight path toward the visible setup coordinate avoids
            # inventing a later vertical side-force during an incoming attack.
            launch_direction = initial_line
        interceptor_orientation = Vec3(
            -math.asin(clamp(launch_direction.y, -1.0, 1.0)),
            math.atan2(launch_direction.x, launch_direction.z),
            0.0,
        )
        incoming_target = (
            target_spec.vehicle_type == "rocket"
            or self.config.scenario == "rocket_attack"
        )
        initial_interceptor_speed = (
            min(24.0, interceptor_spec.max_speed * 0.38)
            if (
                interceptor_spec.flight_model == "rocket"
                or (
                    incoming_target
                    and interceptor_spec.flight_model == "fixed_wing"
                )
            )
            else interceptor_spec.max_speed * 0.5
        )
        self.interceptor = DroneState(
            interceptor_spec,
            self.config.interceptor_position,
            launch_direction * initial_interceptor_speed,
            orientation=interceptor_orientation,
        )
        target_velocity = (
            Vec3(0.0, 0.0, -min(65.0, target_spec.max_speed * 0.72))
            if incoming_target
            else Vec3(8.0, 0.0, min(23.0, target_spec.max_speed * 0.42))
        )
        self.target = DroneState(
            target_spec,
            self.config.target_position,
            target_velocity,
            orientation=Vec3(0.0, math.pi if incoming_target else 0.25, 0.0),
        )
        self.primary_target = self.target
        self.targets: list[DroneState] = [self.target]
        requested_enemy_count = int(clamp(float(self.config.enemy_count), 1.0, 3.0))
        if incoming_target:
            # A rocket attack remains a single-intruder terminal test. The
            # optional multi-contact scene is intentionally a drone scenario.
            requested_enemy_count = 1
        self.config.enemy_count = requested_enemy_count
        available_extra_specs = [
            spec for spec in DRONE_SPECS if spec.code != target_spec.code
        ]
        extra_offsets = (
            Vec3(-42.0, 5.0, -38.0),
            Vec3(52.0, -4.0, 24.0),
        )
        for extra_index in range(requested_enemy_count - 1):
            extra_spec = available_extra_specs[
                (extra_index + DRONE_SPECS.index(target_spec) + 1)
                % len(available_extra_specs)
            ] if target_spec in DRONE_SPECS else available_extra_specs[extra_index]
            offset = extra_offsets[extra_index]
            extra_position = self.config.target_position + offset
            direction_sign = -1.0 if extra_index % 2 else 1.0
            extra_velocity = Vec3(
                direction_sign * (6.0 + extra_index * 2.0),
                0.0,
                min(21.0, extra_spec.max_speed * (0.36 + extra_index * 0.03)),
            )
            self.targets.append(
                DroneState(
                    extra_spec,
                    extra_position,
                    extra_velocity,
                    orientation=Vec3(
                        0.0,
                        0.18 * direction_sign,
                        0.0,
                    ),
                )
            )
        self.camera_forward = self.interceptor.sensor_direction()
        self.search_anchor_forward = self.camera_forward
        self.search_direction = self.camera_forward
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
        self.contacts = [
            TargetContact(
                vehicle,
                f"T{index + 1}",
                vehicle.position.y,
            )
            for index, vehicle in enumerate(self.targets)
        ]
        self.active_contact_index = 0
        self._priority_candidate_index = 0
        self._priority_candidate_time_s = 0.0
        self.shared_overlap: SharedOvalOverlap | None = None
        self.shared_overlap_pair: tuple[int, int] | None = None
        self.multi_committed_contact_index: int | None = None
        self.multi_commit_horizon_s: float | None = None
        self.multi_guidance_mode = "SINGLE TARGET"
        self.explosion_age_s = math.inf
        # The tricky target autopilot is deterministic so demonstrations and
        # verification runs are reproducible. It may use simulation truth
        # because the scenario explicitly assumes that the target knows about
        # the interceptor; truth is never exposed to our sensor or guidance.
        self.evader_decision = "INACTIVE"
        self.evader_decision_until_s = 0.0
        self.evader_decision_index = 0
        self.evader_brake_demonstrated = False
        self.evader_threat_range_m = math.inf
        self.evader_closing_speed_mps = 0.0

    @property
    def true_range_m(self) -> float:
        return self.interceptor.position.distance_to(self.target.position)

    @property
    def active_contact(self) -> TargetContact:
        return self.contacts[self.active_contact_index]

    @property
    def visible_contact_count(self) -> int:
        return sum(contact.visual_locked for contact in self.contacts)

    @property
    def identified_contact_count(self) -> int:
        return sum(contact.identity_confirmed for contact in self.contacts)

    @property
    def active_contact_label(self) -> str:
        return f"{self.active_contact.track_id}/{len(self.contacts)}"

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

    def _store_active_contact(self) -> None:
        """Persist the legacy active-target fields into its independent track."""
        contact = self.active_contact
        contact.detection = self.detection
        contact.range_estimate = self.range_estimate
        contact.track = self.track
        contact.guidance = self.guidance
        contact.signal_progress = self.signal_progress
        contact.identity_confirmed = self.identity_confirmed
        contact.visual_locked = self.visual_locked
        contact.lost_time_s = self.lost_time_s
        contact.reacquisition_count = self.reacquisition_count
        contact.last_estimated_position = self.last_estimated_position
        contact.last_bearing_x_deg = self.last_bearing_x_deg
        contact.last_bearing_y_deg = self.last_bearing_y_deg
        contact.bearing_rate_x_deg_s = self.bearing_rate_x_deg_s
        contact.bearing_rate_y_deg_s = self.bearing_rate_y_deg_s
        contact.bearing_sample_valid = self._bearing_sample_valid

    def _load_active_contact(self) -> None:
        """Expose the selected contact through the original single-target API."""
        contact = self.active_contact
        self.target = contact.vehicle
        self.detection = contact.detection
        self.range_estimate = contact.range_estimate
        self.track = contact.track
        self.guidance = contact.guidance
        self.signal_progress = contact.signal_progress
        self.identity_confirmed = contact.identity_confirmed
        self.visual_locked = contact.visual_locked
        self.lost_time_s = contact.lost_time_s
        self.reacquisition_count = contact.reacquisition_count
        self.last_estimated_position = contact.last_estimated_position
        self.last_bearing_x_deg = contact.last_bearing_x_deg
        self.last_bearing_y_deg = contact.last_bearing_y_deg
        self.bearing_rate_x_deg_s = contact.bearing_rate_x_deg_s
        self.bearing_rate_y_deg_s = contact.bearing_rate_y_deg_s
        self._bearing_sample_valid = contact.bearing_sample_valid

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
        if controlled.spec.flight_model == "rocket":
            self._event(
                f"{controlled.spec.code} uses a fixed nonrestartable booster"
            )
            return controlled.engine_enabled
        controlled.engine_enabled = not controlled.engine_enabled
        state = "started" if controlled.engine_enabled else "cut"
        self._event(f"{controlled.spec.code} engine {state} by player")
        return controlled.engine_enabled

    def capture_prediction_check(self) -> bool:
        """Record the current +2 s sensor prediction for a truth-only audit."""
        if self.guidance is None or not self.visual_locked:
            self._event("2 s prediction check needs a valid visual oval")
            return False
        oval = next(
            (
                candidate
                for candidate in self.guidance.ovals
                if abs(candidate.horizon_s - 2.0) < 1e-6
                and candidate.invalid_reason is None
            ),
            None,
        )
        if oval is None:
            self._event("2 s prediction is unbounded or unavailable")
            return False
        self.prediction_check = PredictionCheck(
            self.time_s,
            Vec3(
                self.interceptor.position.x,
                self.interceptor.position.y,
                self.interceptor.position.z,
            ),
            Vec3(
                self.camera_forward.x,
                self.camera_forward.y,
                self.camera_forward.z,
            ),
            oval,
        )
        self._event("Recorded +2 s sensor-frame prediction")
        return True

    def clear_prediction_check(self) -> None:
        self.prediction_check = None
        self._event("Cleared recorded prediction check")

    def _update_prediction_check(self) -> None:
        check = self.prediction_check
        if (
            check is None
            or check.evaluated
            or self.time_s < check.capture_time_s + check.oval.horizon_s
        ):
            return
        relative = self.target.position - check.camera_position
        depth = relative.dot(check.oval.plane_normal)
        plane_depth = (
            check.oval.center - check.camera_position
        ).dot(check.oval.plane_normal)
        if depth <= 1e-6:
            projected = None
        else:
            projected = check.camera_position + relative * (plane_depth / depth)
        check.projected_truth = projected
        check.result_inside = (
            projected is not None and check.oval.contains_projected(projected)
        )
        check.evaluated = True
        self._event(
            "2 s prediction check: "
            + ("INSIDE" if check.result_inside else "OUTSIDE")
        )

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

    def _sync_sensor_boresight(self) -> None:
        """Rigidly mount the sensor axis to its fixed published body cant."""
        self.camera_forward = self.interceptor.sensor_direction()

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

    def _cruise_command(
        self,
        desired_direction: Vec3,
        hold_altitude_m: float,
        speed_fraction: float = 0.5,
    ) -> Vec3:
        """Regulate a level search cruise without target range information."""
        spec = self.interceptor.spec
        direction = Vec3(
            desired_direction.x,
            0.0,
            desired_direction.z,
        ).normalized(self.interceptor.forward_direction())
        horizontal_velocity = Vec3(
            self.interceptor.velocity.x,
            0.0,
            self.interceptor.velocity.z,
        )
        desired_speed = spec.max_speed * clamp(speed_fraction, 0.25, 0.7)
        horizontal_request = (
            direction * desired_speed - horizontal_velocity
        ) * 1.15
        horizontal_request = horizontal_request.clamp_length(
            max(spec.max_accel, spec.lateral_accel)
        )
        vertical_request = clamp(
            (hold_altitude_m - self.interceptor.position.y) * 1.4
            - self.interceptor.velocity.y * 1.1,
            -spec.lateral_accel,
            spec.lateral_accel,
        )
        if spec.flight_model in ("fixed_wing", "rocket"):
            vertical_request += max(
                0.0,
                9.81 - self.interceptor.lift_acceleration.y,
            )
        command = horizontal_request + Vec3(0.0, vertical_request, 0.0)
        command_limit = max(spec.max_accel, spec.lateral_accel)
        if spec.flight_model in ("fixed_wing", "rocket"):
            command_limit = math.hypot(
                max(spec.max_accel, spec.brake_accel),
                spec.lateral_accel,
            )
        return command.clamp_length(command_limit)

    def _bearing_only_cruise_command(self) -> Vec3:
        """Use only image bearing while identity/range are still unavailable."""
        camera_vector = Vec3(
            math.tan(math.radians(self.detection.bearing_x_deg)),
            math.tan(math.radians(self.detection.bearing_y_deg)),
            1.0,
        )
        observed_direction = world_direction_from_camera(
            camera_vector,
            self.camera_forward,
        )
        return self._cruise_command(
            observed_direction,
            self.search_hold_altitude_m,
            0.5,
        )

    def _lost_search_command(self) -> Vec3:
        """Search from image history while holding altitude and half-speed cruise."""
        current_forward = Vec3(
            self.interceptor.velocity.x,
            0.0,
            self.interceptor.velocity.z,
        ).normalized(self.interceptor.forward_direction())
        if not self._bearing_sample_valid:
            return self._cruise_command(
                current_forward,
                self.search_hold_altitude_m,
                0.5,
            )
        horizontal_direction = Vec3(
            self.search_direction.x,
            0.0,
            self.search_direction.z,
        ).normalized(current_forward)
        turn_ramp = clamp(self.lost_time_s / 0.8, 0.0, 1.0)
        blended_direction = (
            current_forward * (1.0 - turn_ramp)
            + horizontal_direction * turn_ramp
        ).normalized(current_forward)
        return self._cruise_command(
            blended_direction,
            self.search_hold_altitude_m,
            0.5,
        )

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

    def _target_command(
        self,
        target: DroneState | None = None,
        contact_index: int | None = None,
    ) -> Vec3:
        target = target or self.target
        if contact_index is None:
            contact_index = self.active_contact_index
        spec = target.spec
        velocity = target.velocity
        scenario = self.config.scenario
        phase_time = self.time_s + contact_index * 0.43
        incoming_rocket = (
            spec.vehicle_type == "rocket" or scenario == "rocket_attack"
        )
        cruise_z = (
            -min(65.0, spec.max_speed * 0.72)
            if incoming_rocket
            else min(23.0, spec.max_speed * 0.42)
        )
        target_altitude = self.contacts[contact_index].spawn_altitude_m
        stabilise = Vec3(
            0.0,
            (target_altitude - target.position.y) * 0.55
            - velocity.y * 1.15,
            cruise_z - velocity.z,
        )
        stabilise = stabilise * 0.7
        if spec.flight_model in ("fixed_wing", "rocket"):
            stabilise = stabilise + Vec3(
                0.0,
                max(0.0, 9.81 - target.lift_acceleration.y),
                0.0,
            )
        target.airbrake = False

        if scenario == "tricky" and target is self.primary_target:
            return self._smart_evader_command(target, stabilise)
        if scenario == "weave":
            return (
                stabilise
                + Vec3(
                    math.sin(phase_time * 1.55) * spec.lateral_accel * 0.10,
                    math.sin(phase_time * 0.83) * spec.lateral_accel * 0.04,
                    0.0,
                )
            ).clamp_length(spec.max_accel)
        if scenario == "rocket_attack":
            maneuver_scale = 0.0 if spec.code == "SR1" else 0.04
            terminal_weave = Vec3(
                math.sin(phase_time * 1.2) * spec.lateral_accel * maneuver_scale,
                math.sin(phase_time * 0.7) * spec.lateral_accel * maneuver_scale * 0.45,
                0.0,
            )
            return (stabilise + terminal_weave).clamp_length(spec.max_accel)
        if scenario in ("evasive", "tricky"):
            phase = int(phase_time / 1.35) % 4
            directions = (
                Vec3(1.0, 0.32, 0.0),
                Vec3(-0.65, -0.42, 0.25),
                Vec3(-1.0, 0.28, 0.0),
                Vec3(0.72, -0.18, -0.2),
            )
            # The baseline evasive script remains deliberately moderate; the
            # separate TRICKY AI mode is the full threat-aware stress test.
            evade = directions[phase].normalized() * spec.lateral_accel * 0.15
            return (stabilise + evade).clamp_length(spec.max_accel)
        if scenario == "braking":
            cycle = phase_time % 7.0
            if 3.0 < cycle < 5.2:
                target.airbrake = True
                return Vec3(
                    0.0,
                    (target_altitude - target.position.y) * 0.3,
                    0.0,
                )
            return stabilise.clamp_length(spec.max_accel)
        return stabilise.clamp_length(spec.max_accel)

    def _choose_evader_decision(
        self,
        distance_m: float,
        closing_speed_mps: float,
    ) -> None:
        """Choose a deterministic, threat-aware maneuver for the tricky mode."""
        index = self.evader_decision_index
        force_brake_demo = (
            distance_m < 75.0 and not self.evader_brake_demonstrated
        )
        if distance_m > 165.0 or closing_speed_mps < 2.0:
            choices = ("DECEPTIVE CRUISE", "OFFSET JINK", "SPEED SHIFT")
            duration = 1.15
        elif distance_m > 95.0:
            choices = ("OFFSET JINK", "HARD BREAK", "CLIMB", "SPRINT")
            duration = 0.88
        elif distance_m > 48.0:
            choices = (
                "BRAKE TRAP",
                "SNAP LEFT",
                "VERTICAL BREAK",
                "SNAP RIGHT",
                "BOOST ESCAPE",
            )
            duration = 0.68
        else:
            choices = (
                "BRAKE TRAP",
                "SNAP RIGHT",
                "BOOST ESCAPE",
                "SNAP LEFT",
                "VERTICAL BREAK",
            )
            duration = 0.52
        # The range bucket perturbs the sequence without introducing random
        # state, so identical inputs always produce identical demonstrations.
        range_phase = int(distance_m / 17.0)
        self.evader_decision = (
            "BRAKE TRAP"
            if force_brake_demo
            else choices[(index + range_phase) % len(choices)]
        )
        if force_brake_demo:
            self.evader_brake_demonstrated = True
        self.evader_decision_index += 1
        self.evader_decision_until_s = self.time_s + duration
        self._event(f"Target AI: {self.evader_decision}")

    def _smart_evader_command(
        self,
        target: DroneState,
        stabilise: Vec3,
    ) -> Vec3:
        """Reactive target autopilot constrained by the selected craft limits."""
        spec = target.spec
        relative = self.interceptor.position - target.position
        distance = relative.length()
        threat_direction = relative.normalized(Vec3(0.0, 0.0, -1.0))
        relative_velocity = self.interceptor.velocity - target.velocity
        closing = max(0.0, -relative_velocity.dot(threat_direction))
        self.evader_threat_range_m = distance
        self.evader_closing_speed_mps = closing

        if self.time_s >= self.evader_decision_until_s:
            self._choose_evader_decision(distance, closing)

        # A horizontal perpendicular is a predictable way to maximize angular
        # displacement from the interceptor's current line of approach.
        horizontal_threat = Vec3(threat_direction.x, 0.0, threat_direction.z)
        horizontal_threat = horizontal_threat.normalized(Vec3(0.0, 0.0, -1.0))
        side = Vec3(horizontal_threat.z, 0.0, -horizontal_threat.x)
        side_sign = -1.0 if self.evader_decision_index % 2 else 1.0
        lateral = side * side_sign
        away = -horizontal_threat
        authority = spec.lateral_accel
        decision = self.evader_decision

        if decision == "DECEPTIVE CRUISE":
            maneuver = lateral * authority * 0.30
        elif decision == "OFFSET JINK":
            maneuver = lateral * authority * 0.72 + away * authority * 0.18
        elif decision == "SPEED SHIFT":
            # Alternating axial changes complicate a constant-velocity track.
            speed_sign = -1.0 if self.evader_decision_index % 2 else 1.0
            maneuver = (
                target.velocity.normalized(away)
                * spec.max_accel
                * 0.55
                * speed_sign
                + lateral * authority * 0.25
            )
        elif decision == "HARD BREAK":
            maneuver = lateral * authority * 0.92 + away * authority * 0.30
        elif decision == "CLIMB":
            maneuver = lateral * authority * 0.36 + Vec3(0.0, spec.max_accel, 0.0)
        elif decision == "SPRINT":
            maneuver = away * spec.max_accel + lateral * authority * 0.18
        elif decision == "BRAKE TRAP":
            target.airbrake = True
            maneuver = lateral * authority * 0.20
        elif decision == "VERTICAL BREAK":
            vertical_sign = -1.0 if self.evader_decision_index % 2 else 1.0
            if target.position.y < 12.0:
                vertical_sign = 1.0
            maneuver = (
                Vec3(0.0, vertical_sign * spec.max_accel, 0.0)
                + lateral * authority * 0.35
            )
        elif decision == "BOOST ESCAPE":
            maneuver = away * spec.max_accel + Vec3(0.0, spec.max_accel * 0.22, 0.0)
        else:  # SNAP LEFT / SNAP RIGHT
            snap_sign = -1.0 if decision == "SNAP LEFT" else 1.0
            maneuver = side * snap_sign * authority + away * authority * 0.20

        # SMART EVADER has the richest actuator set, but even it keeps a
        # repeatable demonstration reserve. Other catalogue craft use the same
        # controller scaled to their published lateral authority.
        if spec.code == "SEV":
            maneuver = maneuver * 0.66
        else:
            maneuver = maneuver * min(
                0.66,
                18.0 / max(spec.lateral_accel, 0.001),
            )

        # Close to the ground, vertical recovery overrides any downward break.
        ground_recovery = (
            Vec3(0.0, (10.0 - target.position.y) * 3.0, 0.0)
            if target.position.y < 10.0
            else Vec3()
        )
        return (stabilise * 0.55 + maneuver + ground_recovery).clamp_length(
            spec.max_accel
        )

    @staticmethod
    def _update_contact_bearing_motion(
        contact: TargetContact,
        dt: float,
        reset: bool,
    ) -> None:
        """Filter one contact's image motion without using world truth."""
        bearing_x = contact.detection.bearing_x_deg
        bearing_y = contact.detection.bearing_y_deg
        if contact.bearing_sample_valid and not reset and dt > 1e-6:
            raw_x = clamp(
                (bearing_x - contact.last_bearing_x_deg) / dt,
                -120.0,
                120.0,
            )
            raw_y = clamp(
                (bearing_y - contact.last_bearing_y_deg) / dt,
                -90.0,
                90.0,
            )
            alpha = 1.0 - math.exp(-5.0 * dt)
            contact.bearing_rate_x_deg_s += (
                raw_x - contact.bearing_rate_x_deg_s
            ) * alpha
            contact.bearing_rate_y_deg_s += (
                raw_y - contact.bearing_rate_y_deg_s
            ) * alpha
        elif reset:
            contact.bearing_rate_x_deg_s = 0.0
            contact.bearing_rate_y_deg_s = 0.0
        contact.last_bearing_x_deg = bearing_x
        contact.last_bearing_y_deg = bearing_y
        contact.bearing_sample_valid = True

    def _sense_contact(self, contact: TargetContact, dt: float) -> None:
        """Run the same detector, signal, range and track pipeline per target."""
        contact.detection = detect_box(
            contact.vehicle,
            self.interceptor.position,
            self.camera_forward,
            self.config.camera,
            self.time_s,
        )
        if self.sensor_occluded:
            contact.detection = self._empty_detection(
                contact.detection.camera_depth
            )

        was_locked = contact.visual_locked
        previous_lost_time = contact.lost_time_s
        minimum_lock_pixels = (
            2.0 if contact.identity_confirmed and was_locked
            else 2.5 if contact.identity_confirmed
            else 3.0
        )
        visual_detection = contact.detection.visible and max(
            contact.detection.width_px,
            contact.detection.height_px,
        ) >= minimum_lock_pixels

        if visual_detection:
            self._update_contact_bearing_motion(
                contact,
                dt,
                reset=not was_locked and previous_lost_time > 0.25,
            )
            contact.visual_locked = True
            contact.lost_time_s = 0.0
            if not was_locked:
                if contact.identity_confirmed:
                    contact.reacquisition_count += 1
                    self._event(
                        f"{contact.track_id} visual lock reacquired by image detector"
                    )
                else:
                    self._event(
                        f"{contact.track_id} detector acquired unknown aerial vehicle"
                    )
        else:
            if was_locked and contact is self.active_contact:
                self.search_anchor_forward = self.camera_forward
                self.search_direction = self.camera_forward
                self.search_hold_altitude_m = self.interceptor.position.y
                self._event(
                    f"{contact.track_id} visual lock lost; guidance disabled"
                )
            contact.visual_locked = False
            contact.lost_time_s += dt
            contact.range_estimate = None

        signal_delay = (
            self.ROCKET_SIGNAL_DELAY_S
            if contact.vehicle.spec.vehicle_type == "rocket"
            else self.DRONE_SIGNAL_DELAY_S
        )
        if contact.visual_locked and not contact.identity_confirmed:
            contact.signal_progress += dt
            if contact.signal_progress >= signal_delay:
                contact.identity_confirmed = True
                self._event(
                    f"{contact.track_id} signal match: {contact.vehicle.spec.name}"
                )
        elif not contact.visual_locked and not contact.identity_confirmed:
            contact.signal_progress = max(
                0.0,
                contact.signal_progress - dt * 0.5,
            )

        if contact.identity_confirmed and contact.visual_locked:
            if not was_locked and previous_lost_time > 0.25:
                # Reacquisition begins a new camera-derived velocity estimate.
                contact.track = TargetTrack()
            pose_estimate = contact.detection.pose_estimate
            contact.range_estimate = (
                estimate_range(
                    contact.detection,
                    contact.vehicle.spec,
                    pose_estimate,
                    self.camera_forward,
                    self.config.camera,
                )
                if pose_estimate is not None
                else None
            )
            if contact.range_estimate is not None:
                estimated_position = position_from_detection(
                    self.interceptor.position,
                    self.camera_forward,
                    contact.detection,
                    contact.range_estimate.distance_m,
                )
                contact.last_estimated_position = estimated_position
                contact.track.confidence = contact.detection.confidence
                contact.track.update(
                    estimated_position,
                    self.time_s,
                    rapid_velocity_tracking=(
                        contact.vehicle.spec.vehicle_type == "rocket"
                    ),
                )
                contact.track.position_sigma_m = max(
                    contact.track.position_sigma_m,
                    min(12.0, contact.range_estimate.sigma_m),
                )

    def _contact_priority(self, contact: TargetContact) -> float:
        """Threat score derived only from detector and metric-track outputs."""
        if not contact.visual_locked:
            return -math.inf
        apparent = max(
            contact.detection.width_px,
            contact.detection.height_px,
        )
        score = contact.detection.confidence * 24.0 + min(30.0, apparent)
        if contact.track.position is None or contact.track.sample_count < 2:
            return score
        relative = contact.track.position - self.interceptor.position
        estimated_range = max(0.1, relative.length())
        line = relative.normalized(self.camera_forward)
        closing = max(
            0.0,
            (self.interceptor.velocity - contact.track.velocity).dot(line),
        )
        time_to_contact = (
            estimated_range / closing
            if closing > 0.5
            else math.inf
        )
        score += min(80.0, closing * 1.4)
        if math.isfinite(time_to_contact):
            score += 180.0 / (1.0 + time_to_contact)
        score += 35.0 / (1.0 + estimated_range / 80.0)
        return score

    def _choose_priority_contact(self, dt: float) -> None:
        for contact in self.contacts:
            contact.priority_score = self._contact_priority(contact)
        if self.multi_committed_contact_index is not None:
            committed = self.contacts[self.multi_committed_contact_index]
            if committed.visual_locked:
                self.active_contact_index = self.multi_committed_contact_index
                return
            self._event(
                f"{committed.track_id} committed lock lost; overlap search resumed"
            )
            self.multi_committed_contact_index = None
            self.multi_commit_horizon_s = None
            self.multi_guidance_mode = "SINGLE TARGET"
        if len(self.contacts) == 1 or self.control_mode is ControlMode.TARGET:
            return
        best_index = max(
            range(len(self.contacts)),
            key=lambda index: self.contacts[index].priority_score,
        )
        current = self.active_contact
        best = self.contacts[best_index]
        if best_index == self.active_contact_index:
            self._priority_candidate_index = best_index
            self._priority_candidate_time_s = 0.0
            return
        # A visible current target needs a meaningful score advantage and a
        # short dwell. This prevents frame-to-frame priority thrashing.
        advantage = best.priority_score - current.priority_score
        immediate = not current.visual_locked and best.visual_locked
        if not immediate and advantage < 8.0:
            self._priority_candidate_time_s = 0.0
            return
        if self._priority_candidate_index != best_index:
            self._priority_candidate_index = best_index
            self._priority_candidate_time_s = 0.0
        self._priority_candidate_time_s += dt
        if immediate or self._priority_candidate_time_s >= 0.35:
            previous = current.track_id
            self.active_contact_index = best_index
            self._priority_candidate_time_s = 0.0
            self.search_anchor_forward = self.camera_forward
            self.search_direction = self.camera_forward
            self._event(
                f"Priority switched {previous} -> {best.track_id} from image tracks"
            )

    def _contact_entry_horizon(
        self,
        contact: TargetContact,
        below_horizon_s: float = 2.0,
    ) -> float | None:
        """Smallest nested oval entered below the current shared horizon."""
        guidance = contact.guidance
        if guidance is None or not guidance.ovals:
            return None
        for oval in guidance.ovals:
            if (
                oval.horizon_s >= below_horizon_s - 1e-6
                or oval.invalid_reason is not None
            ):
                continue
            entry_point = oval.likely_point or oval.center
            if point_is_reachable(
                self.interceptor,
                entry_point,
                oval.horizon_s,
            ):
                return oval.horizon_s
        return None

    def _update_multi_contact_guidance(self) -> Vec3 | None:
        """Aim at one same-horizon two-target overlap until entry commits."""
        previous_pair = self.shared_overlap_pair
        previous_mode = self.multi_guidance_mode
        self.shared_overlap = None
        self.shared_overlap_pair = None
        if len(self.contacts) < 2:
            self.multi_guidance_mode = "SINGLE TARGET"
            return None
        if self.control_mode is ControlMode.TARGET:
            self.multi_guidance_mode = "PLAYER TARGET CONTROL"
            return None

        if self.multi_committed_contact_index is not None:
            committed_index = self.multi_committed_contact_index
            committed = self.contacts[committed_index]
            if committed.visual_locked and committed.guidance is not None:
                if self.active_contact_index != committed_index:
                    self._store_active_contact()
                    self.active_contact_index = committed_index
                    self._load_active_contact()
                self.multi_guidance_mode = (
                    f"COMMITTED {committed.track_id} / "
                    f"{self.multi_commit_horizon_s or 1.0:.0f}s OVAL ENTRY"
                )
                return None
            self.multi_committed_contact_index = None
            self.multi_commit_horizon_s = None

        eligible = [
            index
            for index, contact in enumerate(self.contacts)
            if contact.visual_locked
            and contact.identity_confirmed
            and contact.guidance is not None
        ]
        selected: tuple[
            float,
            int,
            int,
            SharedOvalOverlap,
        ] | None = None
        # Equal horizon is the rule: 5s is compared only with 5s, then 3s,
        # 2s, and 1s. The largest reachable shared horizon wins.
        for oval_index in range(3, -1, -1):
            candidates: list[
                tuple[float, int, int, SharedOvalOverlap]
            ] = []
            for first_position, first_index in enumerate(eligible):
                first_contact = self.contacts[first_index]
                assert first_contact.guidance is not None
                for second_index in eligible[first_position + 1:]:
                    second_contact = self.contacts[second_index]
                    assert second_contact.guidance is not None
                    overlap = shared_oval_overlap(
                        first_contact.guidance.ovals[oval_index],
                        second_contact.guidance.ovals[oval_index],
                        self.interceptor.position,
                    )
                    if overlap is None or not point_is_reachable(
                        self.interceptor,
                        overlap.aim_point,
                        overlap.horizon_s,
                    ):
                        continue
                    pair_score = (
                        first_contact.priority_score
                        + second_contact.priority_score
                        + min(40.0, overlap.normalized_area * 2000.0)
                    )
                    candidates.append(
                        (
                            pair_score,
                            first_index,
                            second_index,
                            overlap,
                        )
                    )
            if candidates:
                selected = max(candidates, key=lambda item: item[0])
                break

        if selected is None:
            fallback_entries = [
                (
                    index,
                    self._contact_entry_horizon(contact),
                )
                for index, contact in enumerate(self.contacts)
                if contact.visual_locked and contact.identity_confirmed
            ]
            fallback_entries = [
                item for item in fallback_entries if item[1] is not None
            ]
            if fallback_entries:
                commit_index, entry_horizon = max(
                    fallback_entries,
                    key=lambda item: self.contacts[item[0]].priority_score,
                )
                committed = self.contacts[commit_index]
                self._store_active_contact()
                self.active_contact_index = commit_index
                self.multi_committed_contact_index = commit_index
                self.multi_commit_horizon_s = entry_horizon
                self._load_active_contact()
                self.multi_guidance_mode = (
                    f"COMMITTED {committed.track_id} / "
                    f"{entry_horizon:.0f}s OVAL ENTRY"
                )
                self._event(
                    f"{committed.track_id} committed after entering "
                    f"its {entry_horizon:.0f}s oval"
                )
                return None
            self.multi_guidance_mode = (
                f"PRIORITY TARGET {self.active_contact.track_id}"
            )
            return None

        _, first_index, second_index, overlap = selected
        pair = (first_index, second_index)
        pair_entries = [
            (
                index,
                self._contact_entry_horizon(
                    self.contacts[index],
                    overlap.horizon_s,
                ),
            )
            for index in pair
        ]
        pair_entries = [
            item for item in pair_entries if item[1] is not None
        ]
        if pair_entries:
            commit_index, entry_horizon = max(
                pair_entries,
                key=lambda item: self.contacts[item[0]].priority_score,
            )
            committed = self.contacts[commit_index]
            self._store_active_contact()
            self.active_contact_index = commit_index
            self.multi_committed_contact_index = commit_index
            self.multi_commit_horizon_s = entry_horizon
            self._load_active_contact()
            self.multi_guidance_mode = (
                f"COMMITTED {committed.track_id} / "
                f"{entry_horizon:.0f}s OVAL ENTRY"
            )
            self._event(
                f"{committed.track_id} committed after entering "
                f"its {entry_horizon:.0f}s oval"
            )
            return None
        self.shared_overlap = overlap
        self.shared_overlap_pair = pair
        first_contact = self.contacts[first_index]
        second_contact = self.contacts[second_index]
        pair_priority_index = max(
            pair,
            key=lambda index: self.contacts[index].priority_score,
        )
        if self.active_contact_index != pair_priority_index:
            self._store_active_contact()
            self.active_contact_index = pair_priority_index
            self._load_active_contact()
        self.multi_guidance_mode = (
            f"SHARED {first_contact.track_id}+{second_contact.track_id} "
            f"/ {overlap.horizon_s:.0f}s OVERLAP"
        )
        if (
            previous_pair != pair
            or not previous_mode.startswith("SHARED")
        ):
            self._event(
                f"Shared aim: {first_contact.track_id}+"
                f"{second_contact.track_id} {overlap.horizon_s:.0f}s overlap"
            )
        reference_velocity = (
            first_contact.track.velocity + second_contact.track.velocity
        ) * 0.5
        return command_toward_shared_aim(
            self.interceptor,
            overlap.aim_point,
            reference_velocity,
        )

    def _update_sensor(self, dt: float) -> None:
        # The camera is not a target-following gimbal. Its FOV can move only
        # when the interceptor airframe turns.
        self._store_active_contact()
        self._sync_sensor_boresight()
        if self.lost_time_s > 0.0:
            self.search_direction = self._predicted_search_direction()

        for contact in self.contacts:
            self._sense_contact(contact, dt)
        self._choose_priority_contact(dt)
        self._load_active_contact()

        if self.identity_confirmed and not self.visual_locked:
            self.status = "TARGET LOST / SEARCHING"
        elif self.identity_confirmed:
            self.status = "IDENTITY CONFIRMED"
        elif self.visual_locked:
            self.status = "VISUAL LOCK / SIGNAL QUERY"
        else:
            self.status = "SEARCHING"

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
                self.track.position,
                self.guidance.mode if self.guidance else "NONE",
                self.guidance.selected_horizon_s if self.guidance else None,
                self.guidance.reachable_count if self.guidance else 0,
                self.guidance.reachable_total if self.guidance else 0,
                max(0.0, self.interceptor.main_burn_remaining_s),
                max(0.0, self.interceptor.rcs_remaining_s),
                max(0.0, self.target.main_burn_remaining_s),
                max(0.0, self.target.rcs_remaining_s),
                self.active_contact.track_id,
                self.visible_contact_count,
                len(self.contacts),
                self.active_contact.priority_score,
                self.target.spec.vehicle_type,
                self.target.spec.name,
                self.multi_guidance_mode,
                (
                    "+".join(
                        self.contacts[index].track_id
                        for index in self.shared_overlap_pair
                    )
                    if self.shared_overlap_pair is not None
                    else None
                ),
                (
                    self.shared_overlap.horizon_s
                    if self.shared_overlap is not None
                    else None
                ),
                self.evader_decision,
                (
                    self.evader_threat_range_m
                    if self.config.scenario == "tricky"
                    else None
                ),
                (
                    self.evader_closing_speed_mps
                    if self.config.scenario == "tricky"
                    else None
                ),
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
            for target in self.targets:
                target.integrate(Vec3(), dt)
            self._sync_sensor_boresight()
            self._update_prediction_check()
            self.status = self.success_message
            if self.explosion_age_s > 5.0:
                self.finished = True
            return

        previous_relatives = [
            target.position - self.interceptor.position
            for target in self.targets
        ]

        self._update_sensor(dt)
        for contact in self.contacts:
            if (
                contact.identity_confirmed
                and contact.visual_locked
                and contact.track.position is not None
            ):
                contact.guidance = solve_guidance(
                    self.interceptor,
                    contact.track,
                    contact.vehicle.spec,
                    self.camera_forward,
                    self.config.camera.focal_px,
                    self.terminal_mode,
                )
            else:
                contact.guidance = None
        self._load_active_contact()
        shared_command = self._update_multi_contact_guidance()
        if shared_command is not None:
            advisory_command = shared_command
            self.status = f"MULTI-CONTACT / {self.multi_guidance_mode}"
        elif self.guidance is not None:
            advisory_command = self.guidance.commanded_acceleration
            if self.status != "INTERCEPT GUIDANCE":
                self.status = "INTERCEPT GUIDANCE"
                self._event(
                    f"{self.active_contact.track_id} oval reachability guidance active"
                )
        elif self.visual_locked:
            # Bearing-only pursuit while the signal lookup runs.
            self.guidance = None
            advisory_command = self._bearing_only_cruise_command()
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
            if shared_command is not None:
                self.status = "PLAYER CONTROL / SHARED-OVERLAP ADVISORY"
            elif self.guidance is not None:
                self.status = "PLAYER CONTROL / GUIDANCE ADVISORY"
            elif not self.visual_locked:
                self.status = "PLAYER CONTROL / SENSOR SEARCH"
        else:
            self.interceptor.airbrake = False
            interceptor_command = advisory_command
            if self.control_mode is ControlMode.AUTO:
                self.manual_command = ManualCommand()

        interceptor_yaw = (
            self.manual_command.desired_yaw_rad
            if self.control_mode is ControlMode.INTERCEPTOR
            else None
        )
        target_commands: list[Vec3] = []
        target_yaws: list[float | None] = []
        for index, target in enumerate(self.targets):
            if (
                self.control_mode is ControlMode.TARGET
                and target is self.target
            ):
                self.manual_command = self.manual_controller.command(
                    target,
                    self.manual_input,
                    dt,
                )
                target_commands.append(self.manual_command.acceleration)
                target_yaws.append(self.manual_command.desired_yaw_rad)
            else:
                target_commands.append(self._target_command(target, index))
                target_yaws.append(None)
        self.interceptor.integrate(interceptor_command, dt, interceptor_yaw)
        for target, command, target_yaw in zip(
            self.targets,
            target_commands,
            target_yaws,
        ):
            target.integrate(command, dt, target_yaw)
        self._sync_sensor_boresight()
        self._update_prediction_check()
        if (
            self.config.scenario == "rotating"
            and self.control_mode is not ControlMode.TARGET
        ):
            for index, target in enumerate(self.targets):
                phase = self.time_s * 2.4 + index * 0.7
                if target.spec.flight_model in (
                    "multirotor",
                    "vectored_vtol",
                ):
                    target.orientation = Vec3(
                        target.orientation.x,
                        target.orientation.y + 1.45 * dt,
                        math.sin(phase) * 1.1,
                    )
                else:
                    # Directional craft may roll about their flight axis, but
                    # the nose remains tied to integrated velocity.
                    target.orientation = Vec3(
                        target.orientation.x,
                        target.orientation.y,
                        math.sin(phase) * 0.65,
                    )

        # Continuous relative segment tests prevent tunnelling through any
        # separately tracked intruder.
        for index, (target, previous_relative) in enumerate(
            zip(self.targets, previous_relatives)
        ):
            current_relative = target.position - self.interceptor.position
            segment = current_relative - previous_relative
            denominator = segment.length_squared()
            closest_t = (
                clamp(
                    -previous_relative.dot(segment) / denominator,
                    0.0,
                    1.0,
                )
                if denominator > 1e-9
                else 0.0
            )
            closest = previous_relative + segment * closest_t
            collision_distance = (
                self.interceptor.spec.collision_radius
                + target.spec.collision_radius
                + self.CONTACT_TOLERANCE_M
            )
            if closest.length() > collision_distance or self.hit:
                continue
            self._store_active_contact()
            self.active_contact_index = index
            self._load_active_contact()
            self.hit = True
            self.hit_time_s = self.time_s
            self.status = self.success_message
            self._event(self.success_message)
            impact = (self.interceptor.position + target.position) * 0.5
            self.interceptor.position = impact
            target.position = impact
            average_velocity = (
                self.interceptor.velocity + target.velocity
            ) * 0.5
            self.interceptor.velocity = average_velocity + Vec3(-2.0, 1.5, 0.0)
            target.velocity = average_velocity + Vec3(2.0, 2.5, 0.0)
            self.interceptor.crashed = True
            target.crashed = True
            self.interceptor.engine_enabled = False
            target.engine_enabled = False
            self.explosion_age_s = 0.0
            self.clear_manual_input()
            break

        self._store_active_contact()
        self._record_telemetry(dt)
