"""Bundled low-poly vehicle meshes for the software 3D renderer.

The meshes are deliberately procedural and dependency-free. They give every
catalogue vehicle a recognizable silhouette now, while keeping a clean mesh_id
boundary for a future OBJ/glTF/Blender asset importer.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .math3d import Vec3, rotate_euler


@dataclass(frozen=True, slots=True)
class MeshFace:
    indices: tuple[int, ...]
    material: str = "body"


@dataclass(frozen=True, slots=True)
class VehicleMesh:
    vertices: tuple[Vec3, ...]
    faces: tuple[MeshFace, ...]


class MeshBuilder:
    def __init__(self) -> None:
        self.vertices: list[Vec3] = []
        self.faces: list[MeshFace] = []

    def _append_shape(
        self,
        vertices: list[Vec3],
        faces: list[tuple[int, ...]],
        material: str,
        center: Vec3 = Vec3(),
        orientation: Vec3 = Vec3(),
    ) -> None:
        offset = len(self.vertices)
        self.vertices.extend(
            center + rotate_euler(vertex, orientation) for vertex in vertices
        )
        self.faces.extend(
            MeshFace(tuple(offset + index for index in face), material)
            for face in faces
        )

    def add_box(
        self,
        center: Vec3,
        size: Vec3,
        material: str = "body",
        orientation: Vec3 = Vec3(),
    ) -> None:
        half = size * 0.5
        vertices = [
            Vec3(x, y, z)
            for x in (-half.x, half.x)
            for y in (-half.y, half.y)
            for z in (-half.z, half.z)
        ]
        faces = [
            (0, 1, 3, 2),
            (4, 6, 7, 5),
            (0, 4, 5, 1),
            (2, 3, 7, 6),
            (0, 2, 6, 4),
            (1, 5, 7, 3),
        ]
        self._append_shape(vertices, faces, material, center, orientation)

    def add_beam(
        self,
        start: Vec3,
        end: Vec3,
        width: float,
        height: float,
        material: str = "dark",
    ) -> None:
        delta = end - start
        length = math.hypot(delta.x, delta.z)
        yaw = math.atan2(delta.x, delta.z)
        self.add_box(
            (start + end) * 0.5,
            Vec3(width, height, length),
            material,
            Vec3(0, yaw, 0),
        )

    def add_cylinder(
        self,
        center: Vec3,
        radius: float,
        length: float,
        segments: int = 10,
        material: str = "body",
        orientation: Vec3 = Vec3(),
    ) -> None:
        vertices: list[Vec3] = []
        for z in (-length * 0.5, length * 0.5):
            for index in range(segments):
                angle = math.tau * index / segments
                vertices.append(Vec3(math.cos(angle) * radius, math.sin(angle) * radius, z))
        faces: list[tuple[int, ...]] = []
        for index in range(segments):
            following = (index + 1) % segments
            faces.append((index, following, segments + following, segments + index))
        faces.append(tuple(reversed(range(segments))))
        faces.append(tuple(range(segments, segments * 2)))
        self._append_shape(vertices, faces, material, center, orientation)

    def add_cone(
        self,
        base_center: Vec3,
        radius: float,
        length: float,
        segments: int = 10,
        material: str = "accent",
        orientation: Vec3 = Vec3(),
    ) -> None:
        vertices = [
            Vec3(
                math.cos(math.tau * index / segments) * radius,
                math.sin(math.tau * index / segments) * radius,
                0.0,
            )
            for index in range(segments)
        ]
        vertices.append(Vec3(0, 0, length))
        tip = segments
        faces = [
            (index, (index + 1) % segments, tip)
            for index in range(segments)
        ]
        faces.append(tuple(reversed(range(segments))))
        self._append_shape(vertices, faces, material, base_center, orientation)

    def add_prism_xz(
        self,
        polygon: list[tuple[float, float]],
        thickness: float,
        material: str = "body",
        center: Vec3 = Vec3(),
    ) -> None:
        count = len(polygon)
        vertices = [
            Vec3(x, y, z)
            for y in (-thickness * 0.5, thickness * 0.5)
            for x, z in polygon
        ]
        faces: list[tuple[int, ...]] = [
            tuple(reversed(range(count))),
            tuple(range(count, count * 2)),
        ]
        for index in range(count):
            following = (index + 1) % count
            faces.append((index, following, count + following, count + index))
        self._append_shape(vertices, faces, material, center)

    def build(self) -> VehicleMesh:
        """Normalize each mesh to its exact catalogue bounding dimensions."""
        if not self.vertices:
            return VehicleMesh((), ())
        minimum = Vec3(
            min(vertex.x for vertex in self.vertices),
            min(vertex.y for vertex in self.vertices),
            min(vertex.z for vertex in self.vertices),
        )
        maximum = Vec3(
            max(vertex.x for vertex in self.vertices),
            max(vertex.y for vertex in self.vertices),
            max(vertex.z for vertex in self.vertices),
        )
        center = (minimum + maximum) * 0.5
        span = maximum - minimum
        normalized = tuple(
            Vec3(
                (vertex.x - center.x) / max(span.x, 1e-6),
                (vertex.y - center.y) / max(span.y, 1e-6),
                (vertex.z - center.z) / max(span.z, 1e-6),
            )
            for vertex in self.vertices
        )
        return VehicleMesh(normalized, tuple(self.faces))


def _falcon_quad() -> VehicleMesh:
    mesh = MeshBuilder()
    mesh.add_box(Vec3(0, 0, 0), Vec3(0.28, 0.22, 0.42), "body")
    mesh.add_box(Vec3(0, 0.15, 0.04), Vec3(0.18, 0.09, 0.22), "accent")
    rotor_positions = (
        Vec3(-0.34, 0.03, -0.31),
        Vec3(0.34, 0.03, -0.31),
        Vec3(-0.34, 0.03, 0.31),
        Vec3(0.34, 0.03, 0.31),
    )
    for position in rotor_positions:
        mesh.add_beam(Vec3(position.x * 0.22, 0, position.z * 0.22), position, 0.045, 0.045)
        mesh.add_cylinder(position, 0.135, 0.025, 12, "rotor", Vec3(math.pi / 2, 0, 0))
        mesh.add_cylinder(position + Vec3(0, -0.035, 0), 0.045, 0.08, 8, "metal", Vec3(math.pi / 2, 0, 0))
    mesh.add_box(Vec3(-0.14, -0.17, 0), Vec3(0.025, 0.20, 0.28), "dark")
    mesh.add_box(Vec3(0.14, -0.17, 0), Vec3(0.025, 0.20, 0.28), "dark")
    return mesh.build()


def _compact_quad() -> VehicleMesh:
    mesh = MeshBuilder()
    mesh.add_cylinder(Vec3(), 0.19, 0.31, 10, "body")
    mesh.add_box(Vec3(0, 0.13, 0.02), Vec3(0.20, 0.08, 0.19), "accent")
    rotor_positions = (
        Vec3(-0.34, 0, -0.34),
        Vec3(0.34, 0, -0.34),
        Vec3(-0.34, 0, 0.34),
        Vec3(0.34, 0, 0.34),
    )
    for position in rotor_positions:
        mesh.add_beam(Vec3(), position, 0.055, 0.045, "metal")
        mesh.add_cylinder(position, 0.12, 0.02, 10, "rotor", Vec3(math.pi / 2, 0, 0))
    return mesh.build()


def _wraith_wing() -> VehicleMesh:
    mesh = MeshBuilder()
    mesh.add_cylinder(Vec3(0, 0, -0.02), 0.10, 0.78, 10, "body")
    mesh.add_cone(Vec3(0, 0, 0.37), 0.10, 0.18, 10, "accent")
    mesh.add_prism_xz([(0, 0.24), (0.50, -0.04), (0.43, -0.23), (0, -0.13)], 0.045)
    mesh.add_prism_xz([(0, 0.24), (-0.50, -0.04), (-0.43, -0.23), (0, -0.13)], 0.045)
    mesh.add_prism_xz([(0, -0.20), (0.23, -0.42), (0, -0.36)], 0.035, "accent")
    mesh.add_prism_xz([(0, -0.20), (-0.23, -0.42), (0, -0.36)], 0.035, "accent")
    mesh.add_box(Vec3(0, 0.11, -0.30), Vec3(0.035, 0.21, 0.18), "dark", Vec3(0, 0, -0.25))
    mesh.add_cylinder(Vec3(0, 0, -0.43), 0.07, 0.07, 10, "glow")
    return mesh.build()


def _talon_delta() -> VehicleMesh:
    mesh = MeshBuilder()
    mesh.add_cylinder(Vec3(0, 0, 0.02), 0.085, 0.82, 9, "metal")
    mesh.add_cone(Vec3(0, 0, 0.41), 0.085, 0.13, 9, "accent")
    mesh.add_prism_xz(
        [(0, 0.30), (0.48, -0.31), (0.19, -0.24), (0, -0.39)],
        0.055,
        "body",
    )
    mesh.add_prism_xz(
        [(0, 0.30), (-0.48, -0.31), (-0.19, -0.24), (0, -0.39)],
        0.055,
        "body",
    )
    mesh.add_prism_xz([(0, -0.20), (0.08, -0.44), (0, -0.39)], 0.22, "dark")
    mesh.add_cylinder(Vec3(0, 0, -0.43), 0.06, 0.08, 9, "glow")
    return mesh.build()


def _manta_heavy() -> VehicleMesh:
    mesh = MeshBuilder()
    mesh.add_prism_xz(
        [
            (0, 0.48),
            (0.50, 0.02),
            (0.43, -0.31),
            (0.18, -0.22),
            (0, -0.39),
            (-0.18, -0.22),
            (-0.43, -0.31),
            (-0.50, 0.02),
        ],
        0.11,
        "body",
    )
    mesh.add_box(Vec3(0, 0.11, 0.02), Vec3(0.24, 0.14, 0.39), "accent")
    for x in (-0.25, 0.25):
        mesh.add_cylinder(Vec3(x, 0.02, -0.04), 0.12, 0.07, 12, "rotor", Vec3(math.pi / 2, 0, 0))
        mesh.add_cylinder(Vec3(x, 0.02, -0.04), 0.055, 0.09, 10, "glow", Vec3(math.pi / 2, 0, 0))
    mesh.add_box(Vec3(-0.32, 0.09, -0.26), Vec3(0.025, 0.17, 0.17), "dark", Vec3(0, 0, -0.2))
    mesh.add_box(Vec3(0.32, 0.09, -0.26), Vec3(0.025, 0.17, 0.17), "dark", Vec3(0, 0, 0.2))
    return mesh.build()


def _rocket(long_body: bool = False) -> VehicleMesh:
    mesh = MeshBuilder()
    radius = 0.13 if not long_body else 0.11
    mesh.add_cylinder(Vec3(0, 0, -0.05), radius, 0.70, 12, "body")
    mesh.add_cone(Vec3(0, 0, 0.30), radius, 0.22, 12, "accent")
    mesh.add_cylinder(Vec3(0, 0, -0.43), radius * 0.72, 0.08, 12, "metal")
    fin_reach = 0.34 if not long_body else 0.29
    fin_polygon = [(0, -0.24), (fin_reach, -0.43), (fin_reach * 0.82, -0.49), (0, -0.41)]
    mesh.add_prism_xz(fin_polygon, 0.035, "accent")
    mesh.add_prism_xz([(-x, z) for x, z in fin_polygon], 0.035, "accent")
    # The second pair of fins is represented by a vertical tail box.
    mesh.add_box(Vec3(0, 0, -0.37), Vec3(0.035, fin_reach * 1.8, 0.22), "accent")
    mesh.add_cone(Vec3(0, 0, -0.47), radius * 0.70, -0.12, 12, "glow")
    return mesh.build()


MESHES: dict[str, VehicleMesh] = {
    "falcon_quad": _falcon_quad(),
    "wraith_wing": _wraith_wing(),
    "compact_quad": _compact_quad(),
    "talon_delta": _talon_delta(),
    "manta_heavy": _manta_heavy(),
    "rocket_skyfall": _rocket(False),
    "rocket_lance": _rocket(True),
}


def get_mesh(mesh_id: str) -> VehicleMesh:
    try:
        return MESHES[mesh_id]
    except KeyError as exc:
        raise KeyError(f"Unknown vehicle mesh: {mesh_id}") from exc


def transformed_vertices(
    mesh_id: str,
    dimensions: Vec3,
    orientation: Vec3,
    position: Vec3 = Vec3(),
) -> list[Vec3]:
    mesh = get_mesh(mesh_id)
    return [
        position
        + rotate_euler(
            Vec3(
                vertex.x * dimensions.x,
                vertex.y * dimensions.y,
                vertex.z * dimensions.z,
            ),
            orientation,
        )
        for vertex in mesh.vertices
    ]
