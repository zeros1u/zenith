"""Small dependency-free 3D vector and transform helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math


EPSILON = 1e-9


@dataclass(slots=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vec3":
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: float) -> "Vec3":
        return self * scalar

    def __truediv__(self, scalar: float) -> "Vec3":
        if abs(scalar) < EPSILON:
            return Vec3()
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vec3") -> "Vec3":
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def length_squared(self) -> float:
        return self.dot(self)

    def length(self) -> float:
        return math.sqrt(self.length_squared())

    def normalized(self, fallback: "Vec3 | None" = None) -> "Vec3":
        magnitude = self.length()
        if magnitude < EPSILON:
            return fallback if fallback is not None else Vec3()
        return self / magnitude

    def clamp_length(self, maximum: float) -> "Vec3":
        magnitude = self.length()
        if magnitude <= maximum or magnitude < EPSILON:
            return Vec3(self.x, self.y, self.z)
        return self * (maximum / magnitude)

    def distance_to(self, other: "Vec3") -> float:
        return (self - other).length()

    def as_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.z


WORLD_UP = Vec3(0.0, 1.0, 0.0)
WORLD_FORWARD = Vec3(0.0, 0.0, 1.0)
WORLD_RIGHT = Vec3(1.0, 0.0, 0.0)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def lerp(a: float, b: float, amount: float) -> float:
    return a + (b - a) * clamp(amount, 0.0, 1.0)


def lerp_vec(a: Vec3, b: Vec3, amount: float) -> Vec3:
    t = clamp(amount, 0.0, 1.0)
    return a * (1.0 - t) + b * t


def basis_from_forward(forward: Vec3) -> tuple[Vec3, Vec3, Vec3]:
    """Return right, up and forward camera axes."""
    fwd = forward.normalized(WORLD_FORWARD)
    reference_up = WORLD_UP
    if abs(fwd.dot(reference_up)) > 0.98:
        reference_up = WORLD_FORWARD
    right = reference_up.cross(fwd).normalized(WORLD_RIGHT)
    up = fwd.cross(right).normalized(WORLD_UP)
    return right, up, fwd


def rotate_euler(point: Vec3, orientation: Vec3) -> Vec3:
    """Rotate by pitch(X), yaw(Y), then roll(Z), all in radians."""
    cp, sp = math.cos(orientation.x), math.sin(orientation.x)
    cy, sy = math.cos(orientation.y), math.sin(orientation.y)
    cr, sr = math.cos(orientation.z), math.sin(orientation.z)

    pitched = Vec3(point.x, point.y * cp - point.z * sp, point.y * sp + point.z * cp)
    yawed = Vec3(
        pitched.x * cy + pitched.z * sy,
        pitched.y,
        -pitched.x * sy + pitched.z * cy,
    )
    return Vec3(
        yawed.x * cr - yawed.y * sr,
        yawed.x * sr + yawed.y * cr,
        yawed.z,
    )


def camera_coordinates(
    world_point: Vec3,
    camera_position: Vec3,
    camera_forward: Vec3,
) -> Vec3:
    right, up, forward = basis_from_forward(camera_forward)
    relative = world_point - camera_position
    return Vec3(relative.dot(right), relative.dot(up), relative.dot(forward))


def world_direction_from_camera(camera_vector: Vec3, camera_forward: Vec3) -> Vec3:
    right, up, forward = basis_from_forward(camera_forward)
    return (
        right * camera_vector.x + up * camera_vector.y + forward * camera_vector.z
    ).normalized(forward)
