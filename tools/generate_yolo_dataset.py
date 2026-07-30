"""Generate a deterministic YOLO detection dataset from ZENITH sensor pixels."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import random
import sys

import cv2
import pygame

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from zenith.camera import CameraModel
from zenith.math3d import Vec3
from zenith.models import ALL_SPECS
from zenith.physics import DroneState
from zenith.vision import SensorFrameRenderer, YOLO_CLASS_NAME


def parse_size(value: str) -> tuple[int, int]:
    try:
        width_text, height_text = value.lower().split("x", 1)
        width, height = int(width_text), int(height_text)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError("size must look like 960x540") from exc
    if width < 320 or height < 180:
        raise argparse.ArgumentTypeError("dataset images must be at least 320x180")
    return width, height


def build_scene(
    rng: random.Random,
    camera: CameraModel,
    object_count: int,
) -> list[DroneState]:
    states: list[DroneState] = []
    horizontal_tangent = math.tan(
        math.radians(camera.horizontal_fov_deg) * 0.5
    )
    vertical_tangent = camera.height_px / (2.0 * camera.focal_px)
    for _ in range(object_count):
        spec = rng.choice(ALL_SPECS)
        size_bucket = rng.random()
        if size_bucket < 0.55:
            desired_span_px = rng.uniform(3.0, 22.0)
        elif size_bucket < 0.88:
            desired_span_px = rng.uniform(22.0, 95.0)
        else:
            desired_span_px = rng.uniform(95.0, 280.0)
        depth = camera.focal_px * spec.dimensions.length() / desired_span_px
        depth *= rng.uniform(0.88, 1.15)
        normalized_x = rng.uniform(-0.90, 0.90)
        normalized_y = rng.uniform(-0.78, 0.72)
        position = Vec3(
            normalized_x * horizontal_tangent * depth,
            -normalized_y * vertical_tangent * depth,
            depth,
        )
        orientation = Vec3(
            rng.uniform(-0.65, 0.65),
            rng.uniform(-math.pi, math.pi),
            rng.uniform(-math.pi, math.pi),
        )
        states.append(
            DroneState(
                spec,
                position,
                Vec3(),
                orientation=orientation,
            )
        )
    return states


def write_example(
    renderer: SensorFrameRenderer,
    rng: random.Random,
    split: str,
    index: int,
    output: Path,
    size: tuple[int, int],
) -> tuple[int, float]:
    camera = CameraModel(size[0], size[1], 90.0)
    negative = index % 13 == 0
    object_count = 0 if negative else rng.choices((1, 2, 3), (0.68, 0.22, 0.10))[0]
    states = build_scene(rng, camera, object_count)
    bgr, boxes = renderer.render_labeled_bgr(
        states,
        Vec3(),
        Vec3(0.0, 0.0, 1.0),
        camera,
        size,
        style_seed=rng.randrange(0, 10_000),
    )
    stem = f"{split}_{index:05d}"
    image_path = output / "images" / split / f"{stem}.jpg"
    label_path = output / "labels" / split / f"{stem}.txt"
    cv2.imwrite(
        str(image_path),
        bgr,
        [int(cv2.IMWRITE_JPEG_QUALITY), 92],
    )
    labels: list[str] = []
    minimum_span = math.inf
    for left, top, right, bottom in boxes:
        box_width = max(0.0, right - left)
        box_height = max(0.0, bottom - top)
        if box_width < 1.0 or box_height < 1.0:
            continue
        center_x = (left + right) * 0.5 / size[0]
        center_y = (top + bottom) * 0.5 / size[1]
        normalized_width = box_width / size[0]
        normalized_height = box_height / size[1]
        labels.append(
            f"0 {center_x:.8f} {center_y:.8f} "
            f"{normalized_width:.8f} {normalized_height:.8f}"
        )
        minimum_span = min(minimum_span, max(box_width, box_height))
    label_path.write_text(
        "\n".join(labels) + ("\n" if labels else ""),
        encoding="utf-8",
    )
    return len(labels), minimum_span


def generate(
    output: Path,
    train_count: int,
    val_count: int,
    size: tuple[int, int],
    seed: int,
) -> None:
    pygame.init()
    for split in ("train", "val"):
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)
    renderer = SensorFrameRenderer()
    rng = random.Random(seed)
    total_labels = 0
    smallest_span = math.inf
    for split, count in (("train", train_count), ("val", val_count)):
        for index in range(count):
            labels, minimum_span = write_example(
                renderer,
                rng,
                split,
                index,
                output,
                size,
            )
            total_labels += labels
            smallest_span = min(smallest_span, minimum_span)
            if (index + 1) % 100 == 0 or index + 1 == count:
                print(f"{split}: {index + 1}/{count}", flush=True)

    dataset_yaml = output / "zenith.yaml"
    yaml_path = output.resolve().as_posix()
    dataset_yaml.write_text(
        "\n".join(
            (
                f"path: {yaml_path}",
                "train: images/train",
                "val: images/val",
                "names:",
                f"  0: {YOLO_CLASS_NAME}",
                "",
            )
        ),
        encoding="utf-8",
    )
    metadata = {
        "generator": "ZENITH SensorFrameRenderer",
        "seed": seed,
        "train_images": train_count,
        "validation_images": val_count,
        "image_size": [size[0], size[1]],
        "class_names": [YOLO_CLASS_NAME],
        "labels": total_labels,
        "smallest_labeled_span_px": (
            round(smallest_span, 3)
            if math.isfinite(smallest_span)
            else None
        ),
        "truth_boundary": (
            "Simulator annotations are used only to write offline training "
            "labels; runtime YOLO receives BGR pixels only."
        ),
    }
    (output / "metadata.json").write_text(
        json.dumps(metadata, indent=2),
        encoding="utf-8",
    )
    pygame.quit()
    print(f"Dataset: {dataset_yaml.resolve()}")
    print(json.dumps(metadata, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=REPOSITORY_ROOT / "datasets" / "zenith_yolo",
    )
    parser.add_argument("--train", type=int, default=1200)
    parser.add_argument("--val", type=int, default=240)
    parser.add_argument("--size", type=parse_size, default=(960, 540))
    parser.add_argument("--seed", type=int, default=260730)
    args = parser.parse_args()
    generate(
        args.output,
        max(1, args.train),
        max(1, args.val),
        args.size,
        args.seed,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
