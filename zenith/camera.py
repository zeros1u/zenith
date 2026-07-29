"""Monocular pinhole-camera sensing and range estimation."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .math3d import (
    Vec3,
    basis_from_forward,
    camera_coordinates,
    clamp,
    rotate_euler,
    world_direction_from_camera,
)
from .models import DroneSpec
from .meshes import transformed_vertices
from .physics import DroneState


@dataclass(frozen=True, slots=True)
class CameraModel:
    width_px: int = 1920
    height_px: int = 1080
    horizontal_fov_deg: float = 90.0

    @property
    def focal_px(self) -> float:
        return self.width_px / (
            2.0 * math.tan(math.radians(self.horizontal_fov_deg) * 0.5)
        )

    @property
    def vertical_fov_deg(self) -> float:
        return math.degrees(
            2.0 * math.atan(self.height_px / (2.0 * self.focal_px))
        )


@dataclass(slots=True)
class Detection:
    visible: bool
    bbox: tuple[float, float, float, float]
    center_px: tuple[float, float]
    width_px: float
    height_px: float
    bearing_x_deg: float
    bearing_y_deg: float
    confidence: float
    camera_depth: float
    projected_vertices: tuple[tuple[float, float], ...]


@dataclass(slots=True)
class RangeEstimate:
    distance_m: float
    optical_depth_m: float
    sigma_m: float
    naive_distance_m: float
    physical_span_x_m: float
    physical_span_y_m: float


def box_vertices(spec: DroneSpec, orientation: Vec3, position: Vec3) -> list[Vec3]:
    """Known physical bounding-box vertices used by the camera estimator."""
    half = spec.dimensions * 0.5
    return [
        position + rotate_euler(Vec3(x, y, z), orientation)
        for x in (-half.x, half.x)
        for y in (-half.y, half.y)
        for z in (-half.z, half.z)
    ]


def model_vertices(spec: DroneSpec, orientation: Vec3, position: Vec3) -> list[Vec3]:
    """Detailed render vertices for the selected visual mesh."""
    return transformed_vertices(spec.mesh_id, spec.dimensions, orientation, position)


def detect_box(
    target: DroneState,
    camera_position: Vec3,
    camera_forward: Vec3,
    camera: CameraModel,
) -> Detection:
    focal = camera.focal_px
    projected: list[tuple[float, float]] = []
    depths: list[float] = []
    for vertex in box_vertices(target.spec, target.orientation, target.position):
        point = camera_coordinates(vertex, camera_position, camera_forward)
        if point.z <= 0.05:
            continue
        projected.append(
            (
                camera.width_px * 0.5 + focal * point.x / point.z,
                camera.height_px * 0.5 - focal * point.y / point.z,
            )
        )
        depths.append(point.z)

    center_cam = camera_coordinates(target.position, camera_position, camera_forward)
    if len(projected) != 8 or center_cam.z <= 0.05:
        return Detection(False, (0, 0, 0, 0), (0, 0), 0, 0, 0, 0, 0, center_cam.z, ())

    left = min(point[0] for point in projected)
    right = max(point[0] for point in projected)
    top = min(point[1] for point in projected)
    bottom = max(point[1] for point in projected)
    width = max(0.0, right - left)
    height = max(0.0, bottom - top)
    center = ((left + right) * 0.5, (top + bottom) * 0.5)
    # Bearing is derived strictly from the detected image box, not the
    # simulation's 3D target-center coordinate.
    bearing_x = math.degrees(
        math.atan2(center[0] - camera.width_px * 0.5, focal)
    )
    bearing_y = math.degrees(
        math.atan2(camera.height_px * 0.5 - center[1], focal)
    )
    in_sensor = (
        right >= 0
        and left <= camera.width_px
        and bottom >= 0
        and top <= camera.height_px
    )
    size_confidence = clamp((max(width, height) - 2.0) / 16.0, 0.0, 1.0)
    edge_factor = clamp(
        1.0
        - max(
            abs(center[0] - camera.width_px * 0.5) / camera.width_px,
            abs(center[1] - camera.height_px * 0.5) / camera.height_px,
        ),
        0.0,
        1.0,
    )
    confidence = size_confidence * edge_factor
    return Detection(
        in_sensor and max(width, height) >= 2.0,
        (left, top, right, bottom),
        center,
        width,
        height,
        bearing_x,
        bearing_y,
        confidence,
        center_cam.z,
        tuple(projected),
    )


def projected_physical_spans(
    spec: DroneSpec,
    orientation: Vec3,
    camera_forward: Vec3,
) -> tuple[float, float]:
    """Known-model silhouette spans after a visual pose estimate."""
    right, up, _ = basis_from_forward(camera_forward)
    xs: list[float] = []
    ys: list[float] = []
    half = spec.dimensions * 0.5
    for x in (-half.x, half.x):
        for y in (-half.y, half.y):
            for z in (-half.z, half.z):
                rotated = rotate_euler(Vec3(x, y, z), orientation)
                xs.append(rotated.dot(right))
                ys.append(rotated.dot(up))
    return max(xs) - min(xs), max(ys) - min(ys)


def estimate_range(
    detection: Detection,
    spec: DroneSpec,
    pose_estimate: Vec3,
    camera_forward: Vec3,
    camera: CameraModel,
) -> RangeEstimate | None:
    if not detection.visible or detection.width_px < 1.0 or detection.height_px < 1.0:
        return None

    span_x, span_y = projected_physical_spans(spec, pose_estimate, camera_forward)
    # The detector reports integer-pixel box edges, like a real image detector.
    pixel_width = max(1.0, round(detection.width_px))
    pixel_height = max(1.0, round(detection.height_px))
    depth_x = camera.focal_px * span_x / pixel_width
    depth_y = camera.focal_px * span_y / pixel_height
    optical_depth = (depth_x + depth_y) * 0.5

    # At terminal range, different corners no longer share quite the same depth.
    # Refine the first-order size estimate by fitting the complete known 3D box
    # through the same pinhole projection. At long range this converges to W*f/p.
    right_axis, up_axis, forward_axis = basis_from_forward(camera_forward)
    local_camera_vertices: list[Vec3] = []
    half = spec.dimensions * 0.5
    for x in (-half.x, half.x):
        for y in (-half.y, half.y):
            for z in (-half.z, half.z):
                rotated = rotate_euler(Vec3(x, y, z), pose_estimate)
                local_camera_vertices.append(
                    Vec3(
                        rotated.dot(right_axis),
                        rotated.dot(up_axis),
                        rotated.dot(forward_axis),
                    )
                )
    tangent_x = math.tan(math.radians(detection.bearing_x_deg))
    tangent_y = math.tan(math.radians(detection.bearing_y_deg))

    def projected_ratio(candidate_depth: float) -> float:
        pxs: list[float] = []
        pys: list[float] = []
        for vertex in local_camera_vertices:
            depth = max(0.01, candidate_depth + vertex.z)
            pxs.append(camera.focal_px * (tangent_x * candidate_depth + vertex.x) / depth)
            pys.append(camera.focal_px * (tangent_y * candidate_depth + vertex.y) / depth)
        predicted_w = max(pxs) - min(pxs)
        predicted_h = max(pys) - min(pys)
        return 0.5 * (predicted_w / pixel_width + predicted_h / pixel_height)

    low = max(spec.dimensions.length() * 0.55, optical_depth * 0.35)
    high = max(low * 1.2, optical_depth * 2.2)
    for _ in range(10):
        middle = (low + high) * 0.5
        if projected_ratio(middle) > 1.0:
            low = middle
        else:
            high = middle
    optical_depth = (low + high) * 0.5

    ray_scale = math.sqrt(
        1.0
        + math.tan(math.radians(detection.bearing_x_deg)) ** 2
        + math.tan(math.radians(detection.bearing_y_deg)) ** 2
    )
    distance = optical_depth * ray_scale
    naive_depth = camera.focal_px * spec.dimensions.x / pixel_width
    naive_distance = naive_depth * ray_scale

    quantization_fraction = 0.5 * (
        0.5 / pixel_width + 0.5 / pixel_height
    )
    pose_fraction = math.radians(1.5 + 8.0 * (1.0 - detection.confidence)) * 0.08
    sigma = distance * math.sqrt(quantization_fraction**2 + pose_fraction**2)
    return RangeEstimate(
        distance,
        optical_depth,
        max(0.02, sigma),
        naive_distance,
        span_x,
        span_y,
    )


def position_from_detection(
    observer_position: Vec3,
    camera_forward: Vec3,
    detection: Detection,
    distance_m: float,
) -> Vec3:
    camera_ray = Vec3(
        math.tan(math.radians(detection.bearing_x_deg)),
        math.tan(math.radians(detection.bearing_y_deg)),
        1.0,
    )
    world_ray = world_direction_from_camera(camera_ray, camera_forward)
    return observer_position + world_ray * distance_m


def range_from_apparent_size(real_size_m: float, apparent_px: float, focal_px: float) -> float:
    if apparent_px <= 0.0:
        return math.inf
    return real_size_m * focal_px / apparent_px


def minimum_horizontal_resolution(
    distance_m: float,
    real_size_m: float,
    horizontal_fov_deg: float,
    required_pixels: float,
) -> int:
    """Horizontal pixels needed to give a known object the requested pixel span."""
    if real_size_m <= 0.0:
        return 0
    resolution = (
        required_pixels
        * 2.0
        * distance_m
        * math.tan(math.radians(horizontal_fov_deg) * 0.5)
        / real_size_m
    )
    return math.ceil(resolution)
