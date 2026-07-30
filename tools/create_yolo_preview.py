"""Render a reproducible montage of held-out custom-YOLO predictions."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--weights",
        type=Path,
        default=REPOSITORY_ROOT / "models" / "zenith_yolo.pt",
    )
    parser.add_argument(
        "--images",
        type=Path,
        default=REPOSITORY_ROOT
        / "datasets"
        / "zenith_yolo"
        / "images"
        / "val",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPOSITORY_ROOT
        / "artifacts"
        / "yolo_inference_preview.jpg",
    )
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    if not args.weights.is_file():
        raise SystemExit(f"YOLO weights not found: {args.weights}")
    image_paths = sorted(args.images.glob("*.jpg"))
    if len(image_paths) < 6:
        raise SystemExit(
            f"Need at least six generated validation images in {args.images}"
        )
    selected_indices = (5, 37, 79, 121, 181, 229)
    selected = [
        image_paths[min(index, len(image_paths) - 1)]
        for index in selected_indices
    ]

    from ultralytics import YOLO

    model = YOLO(str(args.weights), task="detect")
    results = model.predict(
        [str(path) for path in selected],
        imgsz=960,
        conf=0.05,
        iou=0.55,
        device=args.device,
        verbose=False,
    )

    tile_width, tile_height = 480, 270
    margin = 12
    header = 34
    canvas = cv2.imread(str(selected[0]))
    if canvas is None:
        raise SystemExit(f"Could not load {selected[0]}")
    canvas = cv2.resize(
        canvas,
        (
            tile_width * 3 + margin * 4,
            (tile_height + header) * 2 + margin * 3,
        ),
    )
    canvas[:] = (15, 24, 31)

    for index, (path, result) in enumerate(zip(selected, results)):
        row, column = divmod(index, 3)
        left = margin + column * (tile_width + margin)
        top = margin + row * (tile_height + header + margin)
        plotted = result.plot(
            conf=True,
            labels=True,
            line_width=2,
            font_size=11,
        )
        plotted = cv2.resize(
            plotted,
            (tile_width, tile_height),
            interpolation=cv2.INTER_AREA,
        )
        canvas[
            top + header : top + header + tile_height,
            left : left + tile_width,
        ] = plotted
        cv2.putText(
            canvas,
            f"held-out {path.stem} // detections {len(result.boxes)}",
            (left + 5, top + 23),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (224, 239, 242),
            1,
            cv2.LINE_AA,
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), canvas):
        raise SystemExit(f"Could not write preview: {args.output}")
    print(args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
