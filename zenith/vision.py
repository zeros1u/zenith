"""Pixel-only sensor rendering and optional Ultralytics YOLO inference.

The simulation owns the physical scene, so this module is allowed to use its
vehicle states while rasterising a virtual camera image.  Only the resulting
pixels cross into ``UltralyticsYOLODetector``.  Runtime guidance never receives
the annotation bounds returned by the separate dataset-generation method.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import importlib.util
import math
from pathlib import Path
import time
from typing import Iterable, Sequence

import pygame

from .camera import CameraModel, Detection, model_vertices
from .math3d import Vec3, camera_coordinates, clamp
from .meshes import get_mesh
from .physics import DroneState


DEFAULT_YOLO_WEIGHTS = (
    Path(__file__).resolve().parent.parent / "models" / "zenith_yolo.pt"
)
YOLO_CLASS_NAME = "aerial_target"


class VisionBackendUnavailable(RuntimeError):
    """Raised when a requested trained image backend cannot be loaded."""


@dataclass(frozen=True, slots=True)
class FrameDetection:
    """One object box produced from pixels by an image model."""

    bbox: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str

    @property
    def center_px(self) -> tuple[float, float]:
        left, top, right, bottom = self.bbox
        return ((left + right) * 0.5, (top + bottom) * 0.5)

    @property
    def width_px(self) -> float:
        return max(0.0, self.bbox[2] - self.bbox[0])

    @property
    def height_px(self) -> float:
        return max(0.0, self.bbox[3] - self.bbox[1])


@dataclass(frozen=True, slots=True)
class DetectorMetrics:
    backend_name: str
    inference_ms: float
    detection_count: int
    device: str

    @property
    def inference_fps(self) -> float:
        return 1000.0 / self.inference_ms if self.inference_ms > 1e-6 else 0.0


class SensorFrameRenderer:
    """Small software renderer used only for the rigid onboard sensor."""

    def __init__(self) -> None:
        pygame.init()
        self._backgrounds: dict[tuple[int, int, int], pygame.Surface] = {}

    @staticmethod
    def _camera_model(
        camera: CameraModel,
        output_size: tuple[int, int],
    ) -> CameraModel:
        return CameraModel(
            output_size[0],
            output_size[1],
            camera.horizontal_fov_deg,
        )

    def _background(
        self,
        size: tuple[int, int],
        style_seed: int,
    ) -> pygame.Surface:
        variant = style_seed % 9
        cache_key = (size[0], size[1], variant)
        cached = self._backgrounds.get(cache_key)
        if cached is not None:
            return cached.copy()
        width, height = size
        surface = pygame.Surface(size)
        top = (
            15 + variant * 2,
            45 + (variant * 7) % 25,
            72 + (variant * 11) % 36,
        )
        horizon = (
            75 + (variant * 5) % 35,
            120 + (variant * 3) % 28,
            145 + (variant * 7) % 30,
        )
        for y in range(height):
            amount = y / max(1, height - 1)
            color = tuple(
                int(top[channel] + (horizon[channel] - top[channel]) * amount)
                for channel in range(3)
            )
            pygame.draw.line(surface, color, (0, y), (width, y))

        ground_y = int(height * (0.78 + (variant - 4) * 0.006))
        pygame.draw.rect(
            surface,
            (
                32 + variant * 2,
                66 + variant * 2,
                61 + variant,
            ),
            (0, ground_y, width, height - ground_y),
        )
        # Low-contrast cloud and terrain shapes make the detector learn the
        # vehicle silhouette instead of a single flat background colour.
        for index in range(7):
            cloud_x = int((index * 0.173 + variant * 0.071) % 1.0 * width)
            cloud_y = int((0.10 + (index % 3) * 0.12) * height)
            radius = max(4, int(width * (0.018 + (index % 2) * 0.008)))
            cloud_color = (126 + variant * 3, 155 + variant * 2, 170 + variant)
            pygame.draw.ellipse(
                surface,
                cloud_color,
                (
                    cloud_x - radius * 2,
                    cloud_y - radius // 2,
                    radius * 4,
                    radius,
                ),
            )
        ridge = [
            (0, ground_y),
            (int(width * 0.17), ground_y - int(height * 0.07)),
            (int(width * 0.34), ground_y - int(height * 0.02)),
            (int(width * 0.55), ground_y - int(height * 0.10)),
            (int(width * 0.73), ground_y - int(height * 0.035)),
            (width, ground_y - int(height * 0.08)),
            (width, ground_y),
        ]
        pygame.draw.polygon(surface, (42, 76, 72), ridge)
        self._backgrounds[cache_key] = surface.copy()
        if len(self._backgrounds) > 24:
            self._backgrounds.pop(next(iter(self._backgrounds)))
        return surface

    @staticmethod
    def _project_vertices(
        state: DroneState,
        camera_position: Vec3,
        camera_forward: Vec3,
        camera: CameraModel,
    ) -> tuple[list[Vec3], list[tuple[float, float, float]]] | None:
        vertices = model_vertices(
            state.spec,
            state.orientation,
            state.position,
        )
        projected: list[tuple[float, float, float]] = []
        for vertex in vertices:
            point = camera_coordinates(
                vertex,
                camera_position,
                camera_forward,
            )
            if point.z <= 0.05:
                return None
            projected.append(
                (
                    camera.width_px * 0.5
                    + camera.focal_px * point.x / point.z,
                    camera.height_px * 0.5
                    - camera.focal_px * point.y / point.z,
                    point.z,
                )
            )
        return vertices, projected

    @staticmethod
    def _material_color(
        state: DroneState,
        material: str,
        shade: float,
    ) -> tuple[int, int, int]:
        if material == "rotor":
            base = (22, 29, 34)
        elif material == "dark":
            base = (31, 45, 53)
        elif material == "metal":
            base = (116, 132, 139)
        elif material == "glow":
            base = (255, 132, 34)
            shade = 1.0
        elif material == "accent":
            base = tuple(min(255, value + 52) for value in state.spec.color)
        else:
            base = state.spec.color
        return tuple(int(clamp(value * shade, 0.0, 255.0)) for value in base)

    def _draw_vehicle(
        self,
        surface: pygame.Surface,
        state: DroneState,
        camera_position: Vec3,
        camera_forward: Vec3,
        camera: CameraModel,
    ) -> tuple[float, float, float, float] | None:
        projection = self._project_vertices(
            state,
            camera_position,
            camera_forward,
            camera,
        )
        if projection is None:
            return None
        vertices, projected = projection
        xs = [point[0] for point in projected]
        ys = [point[1] for point in projected]
        unclipped = (min(xs), min(ys), max(xs), max(ys))
        width, height = surface.get_size()
        if (
            unclipped[2] < 0.0
            or unclipped[0] >= width
            or unclipped[3] < 0.0
            or unclipped[1] >= height
        ):
            return None

        mesh = get_mesh(state.spec.mesh_id)
        light = Vec3(-0.35, 0.82, -0.45).normalized()
        sortable = []
        for face in mesh.faces:
            depth = sum(projected[index][2] for index in face.indices) / len(
                face.indices
            )
            sortable.append((depth, face))
        for _, face in sorted(sortable, reverse=True, key=lambda item: item[0]):
            first, second, third = (
                vertices[index] for index in face.indices[:3]
            )
            normal = (second - first).cross(third - first).normalized()
            shade = 0.46 + 0.62 * abs(normal.dot(light))
            color = self._material_color(state, face.material, shade)
            points = [
                (
                    int(clamp(projected[index][0], -32760.0, 32760.0)),
                    int(clamp(projected[index][1], -32760.0, 32760.0)),
                )
                for index in face.indices
            ]
            pygame.draw.polygon(surface, color, points)
            if max(
                unclipped[2] - unclipped[0],
                unclipped[3] - unclipped[1],
            ) >= 7.0:
                pygame.draw.polygon(surface, (17, 24, 29), points, 1)

        return (
            max(0.0, unclipped[0]),
            max(0.0, unclipped[1]),
            min(float(width - 1), unclipped[2]),
            min(float(height - 1), unclipped[3]),
        )

    def _render(
        self,
        vehicles: Iterable[DroneState],
        camera_position: Vec3,
        camera_forward: Vec3,
        camera: CameraModel,
        output_size: tuple[int, int],
        style_seed: int,
        occluded: bool,
    ) -> tuple[object, tuple[tuple[float, float, float, float], ...]]:
        import numpy as np

        output_camera = self._camera_model(camera, output_size)
        surface = self._background(output_size, style_seed)
        annotations: list[tuple[float, float, float, float]] = []
        if not occluded:
            projected_states = []
            for state in vehicles:
                center = camera_coordinates(
                    state.position,
                    camera_position,
                    camera_forward,
                )
                if center.z > 0.05:
                    projected_states.append((center.z, state))
            for _, state in sorted(projected_states, reverse=True):
                bounds = self._draw_vehicle(
                    surface,
                    state,
                    camera_position,
                    camera_forward,
                    output_camera,
                )
                if bounds is not None:
                    annotations.append(bounds)
        else:
            surface.fill((2, 6, 8))

        # Pygame exposes W,H,C RGB; Ultralytics accepts H,W,C BGR uint8.
        rgb = pygame.surfarray.array3d(surface).swapaxes(0, 1)
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        return bgr, tuple(annotations)

    def render_bgr(
        self,
        vehicles: Iterable[DroneState],
        camera_position: Vec3,
        camera_forward: Vec3,
        camera: CameraModel,
        output_size: tuple[int, int],
        *,
        style_seed: int = 0,
        occluded: bool = False,
    ) -> object:
        """Return only pixels; runtime inference cannot access truth labels."""
        frame, _ = self._render(
            vehicles,
            camera_position,
            camera_forward,
            camera,
            output_size,
            style_seed,
            occluded,
        )
        return frame

    def render_labeled_bgr(
        self,
        vehicles: Iterable[DroneState],
        camera_position: Vec3,
        camera_forward: Vec3,
        camera: CameraModel,
        output_size: tuple[int, int],
        *,
        style_seed: int = 0,
    ) -> tuple[object, tuple[tuple[float, float, float, float], ...]]:
        """Dataset-only rendering path returning exact simulator annotations."""
        return self._render(
            vehicles,
            camera_position,
            camera_forward,
            camera,
            output_size,
            style_seed,
            False,
        )


class UltralyticsYOLODetector:
    """Thin adapter from an Ultralytics model to ZENITH frame detections."""

    name = "YOLO CUSTOM"

    def __init__(
        self,
        weights_path: str | Path = DEFAULT_YOLO_WEIGHTS,
        *,
        confidence: float = 0.05,
        image_size: int = 960,
        model: object | None = None,
    ) -> None:
        self.weights_path = Path(weights_path).resolve()
        self.confidence = confidence
        self.image_size = image_size
        self.device = "cpu"
        self.last_metrics = DetectorMetrics(self.name, 0.0, 0, self.device)
        self._frame_index = 0
        self._consecutive_misses = 0
        if model is not None:
            self.model = model
            return
        if not self.weights_path.is_file():
            raise VisionBackendUnavailable(
                f"Custom YOLO weights not found: {self.weights_path}"
            )
        try:
            import torch
            from ultralytics import YOLO
        except (ImportError, OSError) as exc:
            raise VisionBackendUnavailable(
                "YOLO needs a working PyTorch/Ultralytics environment. "
                "Run the project through run_zenith.bat after setup."
            ) from exc
        self.device = "0" if torch.cuda.is_available() else "cpu"
        try:
            self.model = YOLO(str(self.weights_path), task="detect")
        except Exception as exc:
            raise VisionBackendUnavailable(
                f"Could not load custom YOLO weights: {exc}"
            ) from exc

    def _inference_views(
        self,
        frame_bgr: object,
    ) -> tuple[
        tuple[object, tuple[float, float]],
        ...,
    ]:
        """Return a boresight crop, periodically paired with the full FOV.

        The crop is an ordinary region of the same camera image, not a
        truth-directed camera or simulator measurement. It preserves small
        center-frame targets that would otherwise disappear when a 1920/3840
        sensor frame is resized to the nano model's 960-pixel input.
        """
        shape = getattr(frame_bgr, "shape", ())
        if len(shape) < 2:
            return ((frame_bgr, (0.0, 0.0)),)
        frame_height, frame_width = int(shape[0]), int(shape[1])
        crop_width = min(frame_width, self.image_size)
        crop_height = min(
            frame_height,
            max(
                1,
                round(crop_width * frame_height / max(1, frame_width)),
            ),
        )
        if crop_width >= frame_width and crop_height >= frame_height:
            return ((frame_bgr, (0.0, 0.0)),)
        left = max(0, (frame_width - crop_width) // 2)
        top = max(0, (frame_height - crop_height) // 2)
        crop = frame_bgr[
            top : top + crop_height,
            left : left + crop_width,
        ]
        crop_view = (crop, (float(left), float(top)))
        # The target is normally kept near the boresight by image guidance.
        # A periodic full-FOV view preserves honest search coverage without
        # paying for the redundant large view on every 30 Hz detector sample.
        if self._frame_index % 5 == 0 or self._consecutive_misses > 0:
            return (
                (frame_bgr, (0.0, 0.0)),
                crop_view,
            )
        return (crop_view,)

    @staticmethod
    def _iou(
        first: tuple[float, float, float, float],
        second: tuple[float, float, float, float],
    ) -> float:
        left = max(first[0], second[0])
        top = max(first[1], second[1])
        right = min(first[2], second[2])
        bottom = min(first[3], second[3])
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        first_area = max(0.0, first[2] - first[0]) * max(
            0.0,
            first[3] - first[1],
        )
        second_area = max(0.0, second[2] - second[0]) * max(
            0.0,
            second[3] - second[1],
        )
        union = first_area + second_area - intersection
        return intersection / union if union > 1e-9 else 0.0

    @classmethod
    def _same_image_object(
        cls,
        first: FrameDetection,
        second: FrameDetection,
    ) -> bool:
        if cls._iou(first.bbox, second.bbox) >= 0.30:
            return True
        center_distance = math.hypot(
            first.center_px[0] - second.center_px[0],
            first.center_px[1] - second.center_px[1],
        )
        local_scale = max(
            5.0,
            first.width_px,
            first.height_px,
            second.width_px,
            second.height_px,
        )
        return center_distance <= local_scale * 0.65

    @classmethod
    def _merge_overlapping(
        cls,
        detections: Iterable[FrameDetection],
    ) -> tuple[FrameDetection, ...]:
        merged: list[FrameDetection] = []
        for detection in sorted(
            detections,
            key=lambda item: item.confidence,
            reverse=True,
        ):
            if any(
                cls._same_image_object(detection, existing)
                for existing in merged
            ):
                continue
            merged.append(detection)
        return tuple(merged)

    def detect_frame(
        self,
        frame_bgr: object,
        timestamp_s: float,
    ) -> tuple[FrameDetection, ...]:
        del timestamp_s
        started = time.perf_counter()
        views = self._inference_views(frame_bgr)
        try:
            results = self.model.predict(
                source=[view[0] for view in views],
                imgsz=self.image_size,
                conf=self.confidence,
                iou=0.45,
                max_det=12,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            raise VisionBackendUnavailable(
                f"YOLO inference failed: {exc}"
            ) from exc
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        detections: list[FrameDetection] = []
        for result, (_, offset) in zip(results or (), views):
            boxes = getattr(result, "boxes", None)
            names = getattr(result, "names", {}) or {}
            if boxes is not None:
                xyxy = boxes.xyxy.detach().cpu().tolist()
                confidences = boxes.conf.detach().cpu().tolist()
                classes = boxes.cls.detach().cpu().tolist()
                for coordinates, confidence, class_value in zip(
                    xyxy,
                    confidences,
                    classes,
                ):
                    class_id = int(class_value)
                    class_name = str(names.get(class_id, class_id))
                    if class_name != YOLO_CLASS_NAME:
                        continue
                    detections.append(
                        FrameDetection(
                            (
                                float(coordinates[0]) + offset[0],
                                float(coordinates[1]) + offset[1],
                                float(coordinates[2]) + offset[0],
                                float(coordinates[3]) + offset[1],
                            ),
                            float(confidence),
                            class_id,
                            class_name,
                        )
                    )
        merged = self._merge_overlapping(detections)
        self._frame_index += 1
        self._consecutive_misses = (
            0 if merged else self._consecutive_misses + 1
        )
        self.last_metrics = DetectorMetrics(
            self.name,
            max(0.0, elapsed_ms),
            len(merged),
            str(self.device),
        )
        return merged


@lru_cache(maxsize=1)
def yolo_package_present() -> bool:
    """Return whether the bundled weights and imports are actually usable."""
    if (
        not DEFAULT_YOLO_WEIGHTS.is_file()
        or importlib.util.find_spec("ultralytics") is None
        or importlib.util.find_spec("torch") is None
    ):
        return False
    try:
        import torch  # noqa: F401
        from ultralytics import YOLO  # noqa: F401
    except (ImportError, OSError):
        return False
    return True


def frame_detection_to_camera(
    detection: FrameDetection,
    frame_size: tuple[int, int],
    camera: CameraModel,
) -> Detection:
    """Map a YOLO pixel box into the configured sensor coordinate system."""
    frame_width, frame_height = frame_size
    scale_x = camera.width_px / max(1, frame_width)
    scale_y = camera.height_px / max(1, frame_height)
    left, top, right, bottom = (
        detection.bbox[0] * scale_x,
        detection.bbox[1] * scale_y,
        detection.bbox[2] * scale_x,
        detection.bbox[3] * scale_y,
    )
    center = ((left + right) * 0.5, (top + bottom) * 0.5)
    bearing_x = math.degrees(
        math.atan2(center[0] - camera.width_px * 0.5, camera.focal_px)
    )
    bearing_y = math.degrees(
        math.atan2(camera.height_px * 0.5 - center[1], camera.focal_px)
    )
    return Detection(
        True,
        (left, top, right, bottom),
        center,
        max(0.0, right - left),
        max(0.0, bottom - top),
        bearing_x,
        bearing_y,
        clamp(detection.confidence, 0.0, 1.0),
        0.0,
        (),
        None,
    )


def associate_frame_detections(
    expected_centers: Sequence[tuple[float, float] | None],
    detections: Sequence[FrameDetection],
    frame_size: tuple[int, int],
    *,
    maximum_normalized_distance: float = 0.32,
) -> dict[int, int]:
    """Greedy image-only association from track slots to YOLO boxes.

    ``expected_centers`` may come from the previous detector frame or from a
    one-time simulation signal-association seed.  Current target coordinates
    never participate in this matching calculation.
    """
    width, height = frame_size
    normalized_detection_centers = [
        (
            detection.center_px[0] / max(1, width),
            detection.center_px[1] / max(1, height),
        )
        for detection in detections
    ]
    candidates: list[tuple[float, int, int]] = []
    for contact_index, center in enumerate(expected_centers):
        if center is None:
            continue
        expected = (
            center[0] / max(1, width),
            center[1] / max(1, height),
        )
        for detection_index, detected in enumerate(
            normalized_detection_centers
        ):
            distance = math.hypot(
                expected[0] - detected[0],
                expected[1] - detected[1],
            )
            if distance <= maximum_normalized_distance:
                candidates.append(
                    (distance, contact_index, detection_index)
                )

    assignments: dict[int, int] = {}
    used_detections: set[int] = set()
    for _, contact_index, detection_index in sorted(candidates):
        if contact_index in assignments or detection_index in used_detections:
            continue
        assignments[contact_index] = detection_index
        used_detections.add(detection_index)

    remaining_contacts = [
        index
        for index in range(len(expected_centers))
        if index not in assignments
    ]
    remaining_detections = [
        index
        for index in range(len(detections))
        if index not in used_detections
    ]
    # Deterministic initial acquisition. Contact seed centers are generated
    # once by the simulated signal-association boundary, then subsequent
    # association is driven by previous YOLO box centers.
    remaining_contacts.sort(
        key=lambda index: (
            expected_centers[index][0]
            if expected_centers[index] is not None
            else math.inf
        )
    )
    remaining_detections.sort(
        key=lambda index: detections[index].center_px[0]
    )
    for contact_index, detection_index in zip(
        remaining_contacts,
        remaining_detections,
    ):
        assignments[contact_index] = detection_index
    return assignments
