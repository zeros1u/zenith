"""Fixed-timestep drone physics used by the proof of concept."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .math3d import Vec3, WORLD_FORWARD, clamp, lerp
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

    def integrate(self, commanded_acceleration: Vec3, dt: float) -> None:
        if self.crashed:
            gravity = Vec3(0.0, -9.81, 0.0)
            drag = self.velocity * (-0.004 * self.velocity.length())
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

        command = commanded_acceleration.clamp_length(self.spec.max_accel)
        speed = self.velocity.length()
        passive_drag = self.velocity * (-self.spec.drag_coefficient * speed)
        if self.airbrake and speed > 0.01:
            passive_drag = passive_drag - self.velocity.normalized() * self.spec.brake_accel

        self.acceleration = command + passive_drag
        self.velocity = self.velocity + self.acceleration * dt
        self.velocity = self.velocity.clamp_length(self.spec.max_speed)
        self.position = self.position + self.velocity * dt

        if self.velocity.length() > 0.25:
            direction = self.velocity.normalized(WORLD_FORWARD)
            target_yaw = math.atan2(direction.x, direction.z)
            target_pitch = -math.asin(clamp(direction.y, -1.0, 1.0))
            alpha = 1.0 - math.exp(-5.0 * dt)
            self.orientation = Vec3(
                lerp(self.orientation.x, target_pitch, alpha),
                lerp_angle(self.orientation.y, target_yaw, alpha),
                lerp(self.orientation.z, -command.x / max(1.0, self.spec.max_accel) * 0.5, alpha),
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
