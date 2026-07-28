"""Fixed-timestep drone physics used by the proof of concept."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .math3d import (
    Vec3,
    WORLD_FORWARD,
    WORLD_UP,
    clamp,
    lerp,
    lerp_vec,
    rotate_euler,
    rotate_towards,
)
from .models import DroneSpec


@dataclass(slots=True)
class DroneState:
    spec: DroneSpec
    position: Vec3
    velocity: Vec3
    orientation: Vec3 = field(default_factory=Vec3)
    acceleration: Vec3 = field(default_factory=Vec3)
    crashed: bool = False
    airbrake: bool = False
    thrust_vector: Vec3 = field(default_factory=Vec3)
    engine_output: float = 0.0
    engine_enabled: bool = True
    lift_acceleration: Vec3 = field(default_factory=Vec3)
    drag_acceleration: Vec3 = field(default_factory=Vec3)

    def integrate(
        self,
        commanded_acceleration: Vec3,
        dt: float,
        desired_yaw_rad: float | None = None,
    ) -> None:
        if self.crashed:
            self.engine_enabled = False
            gravity = Vec3(0.0, -9.81, 0.0)
            drag = self.velocity * (-0.004 * self.velocity.length())
            self.lift_acceleration = Vec3()
            self.drag_acceleration = drag
            self.engine_output = 0.0
            self.acceleration = gravity + drag
            self.velocity = self.velocity + self.acceleration * dt
            self.position = self.position + self.velocity * dt
            self.orientation = Vec3(
                self.orientation.x + 1.8 * dt,
                self.orientation.y + 1.2 * dt,
                self.orientation.z + 2.4 * dt,
            )
            if self.position.y < 0.0:
                self.position = Vec3(self.position.x, 0.0, self.position.z)
                self.velocity = Vec3(self.velocity.x * 0.55, 0.0, self.velocity.z * 0.55)
            return

        if self.spec.flight_model in ("fixed_wing", "rocket"):
            self._integrate_directional(commanded_acceleration, dt)
        else:
            self._integrate_vectored(
                commanded_acceleration,
                dt,
                desired_yaw_rad,
            )

    def forward_direction(self) -> Vec3:
        """Vehicle nose direction used by engines and envelope calculations."""
        pose = Vec3(self.orientation.x, self.orientation.y, 0.0)
        return rotate_euler(WORLD_FORWARD, pose).normalized(WORLD_FORWARD)

    def _drag_acceleration(self) -> Vec3:
        speed = self.velocity.length()
        passive_drag = self.velocity * (-self.spec.drag_coefficient * speed)
        if self.airbrake and speed > 0.01:
            passive_drag = passive_drag - self.velocity.normalized() * self.spec.brake_accel
        return passive_drag

    def _aerodynamic_lift(self, forward: Vec3, speed: float) -> Vec3:
        """Simplified wing lift: airflow and a non-vertical path are required."""
        if self.spec.stall_speed <= 0.0 or self.spec.lift_efficiency <= 0.0:
            return Vec3()
        lift_direction = (
            WORLD_UP - forward * WORLD_UP.dot(forward)
        ).normalized()
        if lift_direction.length_squared() < 1e-8:
            return Vec3()
        airflow_factor = clamp(
            (speed / self.spec.stall_speed) ** 2,
            0.0,
            1.0,
        )
        return (
            lift_direction
            * 9.81
            * self.spec.lift_efficiency
            * airflow_factor
        )

    def _apply_ground_contact(self) -> None:
        if self.position.y >= 0.0:
            return
        impact_speed = max(0.0, -self.velocity.y)
        self.position = Vec3(self.position.x, 0.0, self.position.z)
        if impact_speed > 4.0:
            self.crashed = True
            self.engine_enabled = False
            self.engine_output = 0.0
            self.velocity = Vec3(
                self.velocity.x * 0.55,
                0.0,
                self.velocity.z * 0.55,
            )
        else:
            self.velocity = Vec3(self.velocity.x, 0.0, self.velocity.z)

    def _integrate_vectored(
        self,
        commanded_acceleration: Vec3,
        dt: float,
        desired_yaw_rad: float | None = None,
    ) -> None:
        """Multirotor/VTOL thrust: requested motion requires a visible body tilt."""
        horizontal = Vec3(
            commanded_acceleration.x,
            0.0,
            commanded_acceleration.z,
        )
        vertical = Vec3(0.0, commanded_acceleration.y, 0.0)
        normalized_demand = math.sqrt(
            (horizontal.length() / max(0.001, self.spec.lateral_accel)) ** 2
            + (vertical.length() / max(0.001, self.spec.max_accel)) ** 2
        )
        demand_scale = 1.0 / max(1.0, normalized_demand)
        command = (horizontal + vertical) * demand_scale
        gravity = WORLD_UP * -9.81
        gravity_compensation = -gravity
        desired_thrust = (
            command + gravity_compensation
            if self.engine_enabled
            else Vec3()
        )

        # A rotorcraft cannot point unlimited thrust sideways while maintaining
        # altitude. Clamp the thrust cone, then slew it instead of teleporting it.
        max_tilt = math.radians(58.0 if self.spec.flight_model == "multirotor" else 46.0)
        desired_direction = rotate_towards(
            WORLD_UP, desired_thrust.normalized(WORLD_UP), max_tilt
        )
        desired_thrust = desired_direction * desired_thrust.length()
        current_thrust = (
            self.thrust_vector
            if self.thrust_vector.length() > 0.1
            else gravity_compensation if self.engine_enabled else Vec3()
        )
        response_rate = (
            self.spec.max_turn_rate_deg / 18.0
            if self.engine_enabled
            else 9.0
        )
        response = 1.0 - math.exp(-response_rate * dt)
        self.thrust_vector = lerp_vec(current_thrust, desired_thrust, response)
        propulsion_acceleration = self.thrust_vector + gravity
        self.engine_output = (
            clamp(
                self.thrust_vector.length()
                / math.sqrt(self.spec.max_accel**2 + 9.81**2),
                0.0,
                1.0,
            )
            if self.engine_enabled
            else 0.0
        )

        self.lift_acceleration = (
            self._aerodynamic_lift(
                self.velocity.normalized(self.forward_direction()),
                self.velocity.length(),
            )
            if not self.engine_enabled
            else Vec3()
        )
        self.drag_acceleration = self._drag_acceleration()
        self.acceleration = (
            self.thrust_vector
            + gravity
            + self.lift_acceleration
            + self.drag_acceleration
        )
        self.velocity = self.velocity + self.acceleration * dt
        self.velocity = self.velocity.clamp_length(self.spec.max_speed)
        self.position = self.position + self.velocity * dt
        self._apply_ground_contact()

        horizontal_velocity = Vec3(self.velocity.x, 0.0, self.velocity.z)
        heading = horizontal_velocity.normalized(self.forward_direction())
        target_yaw = (
            desired_yaw_rad
            if desired_yaw_rad is not None and self.engine_enabled
            else math.atan2(heading.x, heading.z)
        )
        forward_flat = Vec3(math.sin(target_yaw), 0.0, math.cos(target_yaw))
        right_flat = Vec3(forward_flat.z, 0.0, -forward_flat.x)
        target_pitch = clamp(
            -propulsion_acceleration.dot(forward_flat) / 9.81,
            -max_tilt,
            max_tilt,
        )
        target_roll = clamp(
            -propulsion_acceleration.dot(right_flat) / 9.81,
            -max_tilt,
            max_tilt,
        )
        alpha = 1.0 - math.exp(-6.0 * dt)
        self.orientation = Vec3(
            lerp(self.orientation.x, target_pitch, alpha),
            lerp_angle(self.orientation.y, target_yaw, alpha),
            lerp(self.orientation.z, target_roll, alpha),
        )

    def _integrate_directional(self, commanded_acceleration: Vec3, dt: float) -> None:
        """Wing/rocket dynamics: thrust is axial and steering is rate limited."""
        old_velocity = self.velocity
        speed = old_velocity.length()
        forward = (
            old_velocity.normalized(self.forward_direction())
            if speed > 1.0
            else self.forward_direction()
        )
        command = commanded_acceleration.clamp_length(
            max(self.spec.max_accel, self.spec.brake_accel, self.spec.lateral_accel)
        )

        axial_request = command.dot(forward)
        minimum_axial = 0.0 if self.spec.flight_model == "rocket" else -self.spec.brake_accel
        maximum_axial = self.spec.max_accel if self.engine_enabled else 0.0
        axial = clamp(axial_request, minimum_axial, maximum_axial)
        lateral = (command - forward * axial_request).clamp_length(
            self.spec.lateral_accel
        )

        commanded_turn_rate = lateral.length() / max(speed, 5.0)
        allowed_turn_rate = min(
            math.radians(self.spec.max_turn_rate_deg),
            commanded_turn_rate,
        )
        if lateral.length() > 1e-6:
            turn_target = (forward + lateral.normalized() * 0.8).normalized(forward)
            new_forward = rotate_towards(
                forward, turn_target, allowed_turn_rate * dt
            )
        else:
            new_forward = forward

        drag = self._drag_acceleration()
        self.drag_acceleration = drag
        drag_along = drag.dot(forward)
        new_speed = clamp(
            speed + (axial + drag_along) * dt,
            0.0,
            self.spec.max_speed,
        )
        base_velocity = new_forward * new_speed
        gravity = WORLD_UP * -9.81
        self.lift_acceleration = (
            self._aerodynamic_lift(new_forward, new_speed)
            if self.spec.flight_model == "fixed_wing"
            else Vec3()
        )
        self.velocity = (
            base_velocity
            + (gravity + self.lift_acceleration) * dt
        ).clamp_length(self.spec.max_speed)
        self.acceleration = (self.velocity - old_velocity) / max(dt, 1e-6)
        self.position = self.position + self.velocity * dt
        self._apply_ground_contact()
        self.thrust_vector = forward * max(0.0, axial)
        self.engine_output = (
            clamp(
                max(0.0, axial) / max(0.001, self.spec.max_accel),
                0.0,
                1.0,
            )
            if self.engine_enabled
            else 0.0
        )

        target_yaw = math.atan2(new_forward.x, new_forward.z)
        target_pitch = -math.asin(clamp(new_forward.y, -1.0, 1.0))
        turn_axis = forward.cross(new_forward)
        bank_limit = math.radians(72.0 if self.spec.flight_model == "fixed_wing" else 28.0)
        target_roll = clamp(-turn_axis.y / max(dt, 1e-6), -bank_limit, bank_limit)
        alpha = 1.0 - math.exp(-7.0 * dt)
        self.orientation = Vec3(
            lerp(self.orientation.x, target_pitch, alpha),
            lerp_angle(self.orientation.y, target_yaw, alpha),
            lerp(self.orientation.z, target_roll, alpha),
        )


def lerp_angle(start: float, end: float, amount: float) -> float:
    delta = (end - start + math.pi) % (2.0 * math.pi) - math.pi
    return start + delta * clamp(amount, 0.0, 1.0)


def maximum_travel_distance(initial_along_speed: float, spec: DroneSpec, time_s: float) -> float:
    """Maximum one-dimensional distance under acceleration and speed constraints."""
    if time_s <= 0.0:
        return 0.0
    speed = max(0.0, initial_along_speed)
    accel = max(0.001, spec.max_accel)
    if speed >= spec.max_speed:
        return spec.max_speed * time_s
    accelerate_time = min(time_s, (spec.max_speed - speed) / accel)
    accelerated = speed * accelerate_time + 0.5 * accel * accelerate_time**2
    cruise = spec.max_speed * (time_s - accelerate_time)
    return accelerated + cruise


def time_to_reach(distance: float, initial_along_speed: float, spec: DroneSpec) -> float:
    """Minimum idealised travel time for a point along a chosen direction."""
    if distance <= 0.0:
        return 0.0
    speed = max(0.0, initial_along_speed)
    accel = max(0.001, spec.max_accel)
    accelerate_time = max(0.0, (spec.max_speed - speed) / accel)
    accelerate_distance = speed * accelerate_time + 0.5 * accel * accelerate_time**2
    if distance <= accelerate_distance:
        return (-speed + math.sqrt(speed * speed + 2.0 * accel * distance)) / accel
    return accelerate_time + (distance - accelerate_distance) / spec.max_speed
