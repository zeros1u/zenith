"""Player-control requests translated into physically valid flight commands."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math

from .math3d import Vec3, WORLD_UP, clamp
from .physics import DroneState


class ControlMode(Enum):
    """Who owns the two vehicles in the current demonstration."""

    AUTO = "AUTO"
    INTERCEPTOR = "PLAYER / OUR VEHICLE"
    TARGET = "PLAYER / TARGET"


@dataclass(slots=True)
class ManualControlInput:
    """Normalised presentation-key input sampled by the desktop application."""

    forward: float = 0.0
    turn: float = 0.0
    vertical: float = 0.0
    full_thrust: bool = False
    brake: bool = False

    @property
    def active(self) -> bool:
        return (
            abs(self.forward) > 1e-6
            or abs(self.turn) > 1e-6
            or abs(self.vertical) > 1e-6
            or self.full_thrust
            or self.brake
        )


@dataclass(slots=True)
class ManualCommand:
    acceleration: Vec3 = field(default_factory=Vec3)
    desired_yaw_rad: float | None = None
    requested_engine: float = 0.0
    brake_available: bool = True
    floor_protection: bool = False


class ManualFlightController:
    """Assisted control: keys express intent, the normal physics remains final."""

    FLOOR_ALTITUDE_M = 2.0
    NORMAL_AUTHORITY = 0.62

    def __init__(self) -> None:
        self.heading_yaw_rad = 0.0
        self.hold_altitude_m = 0.0
        self.initialized = False

    def initialize(self, state: DroneState) -> None:
        forward = state.forward_direction()
        self.heading_yaw_rad = math.atan2(forward.x, forward.z)
        self.hold_altitude_m = max(self.FLOOR_ALTITUDE_M, state.position.y)
        self.initialized = True

    def command(
        self,
        state: DroneState,
        controls: ManualControlInput,
        dt: float,
    ) -> ManualCommand:
        if not self.initialized:
            self.initialize(state)

        spec = state.spec
        full_scale = 1.0 if controls.full_thrust else self.NORMAL_AUTHORITY
        forward_request = clamp(controls.forward, -1.0, 1.0)
        if controls.full_thrust and abs(forward_request) < 1e-6:
            forward_request = 1.0

        floor_protection = (
            state.position.y <= self.FLOOR_ALTITUDE_M + 0.35
            and controls.vertical < 0.0
        )
        vertical_request = 0.0 if floor_protection else clamp(controls.vertical, -1.0, 1.0)
        climb_rate = min(14.0, max(5.0, spec.max_speed * 0.24))
        self.hold_altitude_m = max(
            self.FLOOR_ALTITUDE_M,
            self.hold_altitude_m
            + vertical_request * climb_rate * full_scale * dt,
        )
        vertical_acceleration = clamp(
            (self.hold_altitude_m - state.position.y) * 1.8
            - state.velocity.y * 1.2,
            -spec.lateral_accel,
            spec.max_accel,
        )
        brake_available = spec.flight_model != "rocket"
        state.airbrake = controls.brake and brake_available

        if spec.flight_model in ("multirotor", "vectored_vtol"):
            turn_rate = math.radians(spec.max_turn_rate_deg)
            self.heading_yaw_rad += clamp(controls.turn, -1.0, 1.0) * turn_rate * dt
            self.heading_yaw_rad = (
                self.heading_yaw_rad + math.pi
            ) % math.tau - math.pi
            heading = Vec3(
                math.sin(self.heading_yaw_rad),
                0.0,
                math.cos(self.heading_yaw_rad),
            )
            horizontal = (
                heading
                * forward_request
                * spec.lateral_accel
                * full_scale
            )
            vertical = WORLD_UP * vertical_acceleration
            requested = horizontal + vertical
            desired_yaw = self.heading_yaw_rad
        else:
            forward = state.velocity.normalized(state.forward_direction())
            right = WORLD_UP.cross(forward).normalized(Vec3(1.0, 0.0, 0.0))
            if spec.flight_model == "rocket" and forward_request < 0.0:
                forward_request = 0.0
            axial_limit = (
                spec.max_accel
                if forward_request >= 0.0
                else spec.brake_accel
            )
            directional_vertical = vertical_acceleration
            if state.engine_enabled or spec.lift_efficiency > 0.0:
                directional_vertical += max(
                    0.0,
                    9.81 - state.lift_acceleration.y,
                )
            requested = (
                forward * forward_request * axial_limit * full_scale
                + right
                * clamp(controls.turn, -1.0, 1.0)
                * spec.lateral_accel
                * full_scale
                + WORLD_UP
                * directional_vertical
            )
            desired_yaw = None

        if state.position.y < self.FLOOR_ALTITUDE_M:
            recovery = clamp(
                (self.FLOOR_ALTITUDE_M - state.position.y) * 5.0
                - state.velocity.y * 1.5,
                0.0,
                spec.max_accel,
            )
            requested = requested + WORLD_UP * recovery
            floor_protection = True

        requested = requested.clamp_length(
            max(spec.max_accel, spec.lateral_accel, spec.brake_accel)
        )
        requested_engine = clamp(
            max(0.0, requested.dot(state.forward_direction()))
            / max(0.001, spec.max_accel),
            0.0,
            1.0,
        )
        return ManualCommand(
            requested,
            desired_yaw,
            requested_engine,
            brake_available,
            floor_protection,
        )
