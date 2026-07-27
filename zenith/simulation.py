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
from .guidance import GuidanceSolution, TargetTrack, solve_guidance
from .math3d import Vec3, clamp, lerp_vec
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
    CONTACT_TOLERANCE_M = 0.04

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

    def _event(self, message: str) -> None:
        if message == self.last_event:
            return
        self.last_event = message
        self.events.append((self.time_s, message))
        self.events = self.events[-12:]

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
        self.target.airbrake = False

        if scenario == "weave":
            return (
                stabilise
                + Vec3(
                    math.sin(self.time_s * 1.55) * spec.lateral_accel * 0.72,
                    math.sin(self.time_s * 0.83) * spec.lateral_accel * 0.24,
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
        if self.identity_confirmed and self.track.position is not None:
            predicted_direction = (
                self.track.position - self.interceptor.position
            ).normalized(self.camera_forward)
            tracker_alpha = 1.0 - math.exp(-4.5 * dt)
            self.camera_forward = lerp_vec(
                self.camera_forward, predicted_direction, tracker_alpha
            ).normalized(self.camera_forward)

        self.detection = detect_box(
            self.target,
            self.interceptor.position,
            self.camera_forward,
            self.config.camera,
        )
        incoming_rocket = self.target.spec.vehicle_type == "rocket"
        minimum_lock_pixels = 3.0 if incoming_rocket else 4.0
        signal_delay = (
            self.ROCKET_SIGNAL_DELAY_S
            if incoming_rocket
            else self.DRONE_SIGNAL_DELAY_S
        )
        visual_lock = self.detection.visible and max(
            self.detection.width_px, self.detection.height_px
        ) >= minimum_lock_pixels

        if visual_lock and not self.identity_confirmed:
            self.signal_progress += dt
            if self.status == "SEARCHING":
                self.status = "VISUAL LOCK / SIGNAL QUERY"
                self._event("Visual detector acquired an unknown aerial vehicle")
            if self.signal_progress >= signal_delay:
                self.identity_confirmed = True
                self.status = "IDENTITY CONFIRMED"
                self._event(f"Signal match: {self.target.spec.name}")
        elif not visual_lock and not self.identity_confirmed:
            self.signal_progress = max(0.0, self.signal_progress - dt * 0.5)
            self.status = "SEARCHING"

        if self.identity_confirmed and self.detection.visible:
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

                desired_forward = (
                    estimated_position - self.interceptor.position
                ).normalized(self.camera_forward)
                tracker_alpha = 1.0 - math.exp(-7.0 * dt)
                self.camera_forward = lerp_vec(
                    self.camera_forward, desired_forward, tracker_alpha
                ).normalized(self.camera_forward)
        elif self.identity_confirmed:
            self.track.coast(self.time_s)

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
        if self.identity_confirmed and self.track.position is not None:
            self.guidance = solve_guidance(self.interceptor, self.track, self.target.spec)
            interceptor_command = (
                self.guidance.commanded_acceleration if self.guidance else Vec3()
            )
            if self.status != "INTERCEPT GUIDANCE":
                self.status = "INTERCEPT GUIDANCE"
                self._event("Oval reachability guidance active")
        elif self.detection.visible:
            # Bearing-only pursuit while the signal lookup runs.
            interceptor_command = (
                self.camera_forward * self.interceptor.spec.max_speed
                - self.interceptor.velocity
            ).clamp_length(self.interceptor.spec.max_accel)
        else:
            interceptor_command = Vec3()

        target_command = self._target_command()
        self.interceptor.integrate(interceptor_command, dt)
        self.target.integrate(target_command, dt)
        if self.config.scenario == "rotating":
            self.target.orientation = Vec3(
                math.sin(self.time_s * 1.7) * 0.42,
                self.target.orientation.y + 1.45 * dt,
                math.sin(self.time_s * 2.4) * 1.1,
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
            self.explosion_age_s = 0.0

        self._record_telemetry(dt)
