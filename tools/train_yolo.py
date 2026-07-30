"""Train and export the custom ZENITH aerial-target YOLO model."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def scalar(value: object) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def portable_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPOSITORY_ROOT).as_posix()
    except ValueError:
        return str(resolved)


def copy_portable_checkpoint(source: Path, destination: Path) -> None:
    """Copy the best checkpoint without machine-specific training paths."""
    import torch

    checkpoint = torch.load(
        source,
        map_location="cpu",
        weights_only=False,
    )
    training_args = checkpoint.get("train_args")
    if isinstance(training_args, dict):
        training_args["data"] = "datasets/zenith_yolo/zenith.yaml"
        training_args["project"] = "runs"
        training_args["name"] = "zenith_yolo"
    torch.save(checkpoint, destination)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        type=Path,
        default=REPOSITORY_ROOT
        / "datasets"
        / "zenith_yolo"
        / "zenith.yaml",
    )
    parser.add_argument("--base", default="yolo11n.pt")
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument(
        "--output",
        type=Path,
        default=REPOSITORY_ROOT / "models" / "zenith_yolo.pt",
    )
    args = parser.parse_args()

    if not args.data.is_file():
        raise SystemExit(
            f"Dataset YAML not found: {args.data}. Run "
            "tools/generate_yolo_dataset.py first."
        )

    from ultralytics import YOLO

    model = YOLO(args.base)
    training = model.train(
        data=str(args.data.resolve()),
        epochs=max(1, args.epochs),
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=max(0, args.workers),
        project=str((REPOSITORY_ROOT / "runs").resolve()),
        name="zenith_yolo",
        exist_ok=True,
        pretrained=True,
        patience=12,
        seed=260730,
        deterministic=True,
        plots=True,
        verbose=True,
        close_mosaic=8,
    )
    save_dir = Path(training.save_dir)
    best = save_dir / "weights" / "best.pt"
    if not best.is_file():
        raise SystemExit(f"Training did not produce best weights: {best}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    copy_portable_checkpoint(best, args.output)
    training_plot = save_dir / "results.png"
    if training_plot.is_file():
        artifact = (
            REPOSITORY_ROOT
            / "artifacts"
            / "yolo_training_results.png"
        )
        artifact.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(training_plot, artifact)

    validation_model = YOLO(str(args.output))
    validation = validation_model.val(
        data=str(args.data.resolve()),
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=max(0, args.workers),
        plots=True,
        project=str((REPOSITORY_ROOT / "runs").resolve()),
        name="zenith_yolo_validation",
        exist_ok=True,
    )
    results_dict = getattr(validation, "results_dict", {}) or {}
    metrics = {
        "weights": portable_path(args.output),
        "base_model": args.base,
        "epochs_requested": args.epochs,
        "image_size": args.imgsz,
        "dataset": portable_path(args.data),
        "sha256": sha256_file(args.output),
        "precision": scalar(results_dict.get("metrics/precision(B)")),
        "recall": scalar(results_dict.get("metrics/recall(B)")),
        "map50": scalar(results_dict.get("metrics/mAP50(B)")),
        "map50_95": scalar(results_dict.get("metrics/mAP50-95(B)")),
        "fitness": scalar(getattr(validation, "fitness", None)),
    }
    metrics_path = args.output.with_name("zenith_yolo_metrics.json")
    metrics_path.write_text(
        json.dumps(metrics, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
