"""Expandable drone and incoming-threat specification catalogue."""

from __future__ import annotations

from dataclasses import dataclass

from .math3d import Vec3


@dataclass(frozen=True, slots=True)
class DroneSpec:
    code: str
    name: str
    color: tuple[int, int, int]
    dimensions: Vec3  # width (X), height (Y), length (Z), metres
    max_speed: float
    max_accel: float
    brake_accel: float
    lateral_accel: float
    drag_coefficient: float
    notes: str
    mesh_id: str = "falcon_quad"
    vehicle_type: str = "drone"
    flight_model: str = "multirotor"
    max_turn_rate_deg: float = 90.0

    @property
    def size_label(self) -> str:
        return (
            f"{self.dimensions.x:.2f} x {self.dimensions.y:.2f} x "
            f"{self.dimensions.z:.2f} m"
        )

    @property
    def collision_radius(self) -> float:
        # Bounding sphere around the current known-model geometry.
        return self.dimensions.length() * 0.5

    @property
    def bounding_volume(self) -> float:
        return self.dimensions.x * self.dimensions.y * self.dimensions.z


DRONE_SPECS: tuple[DroneSpec, ...] = (
    DroneSpec(
        "FX1",
        "FALCON-X1",
        (255, 91, 99),
        Vec3(0.70, 0.22, 0.62),
        70.0,
        18.0,
        22.0,
        15.0,
        0.0016,
        "Agile reconnaissance quad",
        "falcon_quad",
        flight_model="multirotor",
        max_turn_rate_deg=115.0,
    ),
    DroneSpec(
        "WRS",
        "WRAITH-S",
        (166, 106, 255),
        Vec3(1.10, 0.28, 0.90),
        82.0,
        22.0,
        27.0,
        18.0,
        0.0013,
        "Fast swept-wing platform",
        "wraith_wing",
        flight_model="fixed_wing",
        max_turn_rate_deg=82.0,
    ),
    DroneSpec(
        "AQ4",
        "AEGIS-Q4",
        (72, 219, 164),
        Vec3(0.55, 0.18, 0.55),
        55.0,
        14.0,
        19.0,
        12.0,
        0.0020,
        "Compact defensive quad",
        "compact_quad",
        flight_model="multirotor",
        max_turn_rate_deg=125.0,
    ),
    DroneSpec(
        "TLR",
        "TALON-R",
        (255, 174, 73),
        Vec3(0.85, 0.26, 0.72),
        75.0,
        19.0,
        24.0,
        16.0,
        0.0015,
        "High-agility interceptor",
        "talon_delta",
        flight_model="fixed_wing",
        max_turn_rate_deg=108.0,
    ),
    DroneSpec(
        "MNH",
        "MANTA-H",
        (68, 169, 255),
        Vec3(1.35, 0.34, 1.10),
        48.0,
        12.0,
        17.0,
        10.0,
        0.0024,
        "Heavy endurance platform",
        "manta_heavy",
        flight_model="vectored_vtol",
        max_turn_rate_deg=72.0,
    ),
)


ROCKET_SPECS: tuple[DroneSpec, ...] = (
    DroneSpec(
        "SR1",
        "SKYFALL-R1",
        (238, 79, 62),
        Vec3(0.55, 0.55, 2.40),
        96.0,
        26.0,
        0.0,
        7.0,
        0.0007,
        "High-speed unguided rocket",
        "rocket_skyfall",
        "rocket",
        "rocket",
        18.0,
    ),
    DroneSpec(
        "LM2",
        "LANCE-M2",
        (224, 197, 80),
        Vec3(0.70, 0.70, 3.10),
        84.0,
        21.0,
        0.0,
        10.0,
        0.0009,
        "Maneuver-capable defense threat",
        "rocket_lance",
        "rocket",
        "rocket",
        31.0,
    ),
)

TARGET_SPECS: tuple[DroneSpec, ...] = DRONE_SPECS + ROCKET_SPECS
ALL_SPECS: tuple[DroneSpec, ...] = TARGET_SPECS


def get_spec(code: str) -> DroneSpec:
    for spec in ALL_SPECS:
        if spec.code == code:
            return spec
    raise KeyError(f"Unknown vehicle code: {code}")
