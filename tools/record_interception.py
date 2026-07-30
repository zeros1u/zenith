"""Render a deterministic INTERCEPTRON interception clip through FFmpeg."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame

from zenith.rendering import WorldRenderer
from zenith.simulation import (
    InterceptionSimulation,
    SCENARIOS,
    SimulationConfig,
)


FIXED_STEP = 1.0 / 60.0


def parse_size(value: str) -> tuple[int, int]:
    """Parse WIDTHxHEIGHT command-line values."""
    try:
        width_text, height_text = value.lower().split("x", 1)
        width, height = int(width_text), int(height_text)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("size must look like 1280x720") from exc
    if width < 640 or height < 480:
        raise argparse.ArgumentTypeError("size must be at least 640x480")
    return width, height


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=(
            REPOSITORY_ROOT
            / "artifacts"
            / "interceptron_interception_10s.mp4"
        ),
    )
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--simulation-duration", type=float, default=14.0)
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--size", type=parse_size, default=(1280, 720))
    parser.add_argument(
        "--scenario",
        choices=tuple(key for key, _ in SCENARIOS),
        default="evasive",
    )
    return parser.parse_args()


def render_clip(args: argparse.Namespace) -> None:
    if args.duration <= 0.0 or args.simulation_duration <= 0.0 or args.fps <= 0:
        raise SystemExit("duration, simulation duration, and FPS must be positive")

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("FFmpeg was not found on PATH")

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    width, height = args.size
    frame_count = round(args.duration * args.fps)
    playback_scale = args.simulation_duration / args.duration

    target_code = {
        "rocket_attack": "SR1",
        "tricky": "SEV",
    }.get(args.scenario, "FX1")
    simulation = InterceptionSimulation(
        SimulationConfig(scenario=args.scenario, target_code=target_code)
    )

    pygame.init()
    surface = pygame.Surface((width, height))
    renderer = WorldRenderer()
    hit_view_selected = False

    command = [
        ffmpeg,
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s:v",
        f"{width}x{height}",
        "-r",
        str(args.fps),
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert process.stdin is not None

    try:
        for frame_index in range(frame_count):
            requested_time = (
                (frame_index + 1)
                / frame_count
                * args.simulation_duration
            )
            while simulation.time_s + 1e-9 < requested_time:
                simulation.step(FIXED_STEP)
                renderer.update_trails(simulation, FIXED_STEP)
                if simulation.hit and not hit_view_selected:
                    renderer.view_mode = 1
                    hit_view_selected = True

            renderer.draw_world(surface, simulation)
            renderer.draw_hud(
                surface,
                simulation,
                playback_scale,
                float(args.fps),
            )
            process.stdin.write(pygame.image.tobytes(surface, "RGB"))
    except BrokenPipeError as exc:
        stderr = (
            process.stderr.read().decode("utf-8", errors="replace")
            if process.stderr is not None
            else ""
        )
        raise SystemExit(f"FFmpeg stopped while encoding:\n{stderr}") from exc
    finally:
        process.stdin.close()
        pygame.quit()

    stderr = (
        process.stderr.read().decode("utf-8", errors="replace")
        if process.stderr is not None
        else ""
    )
    return_code = process.wait()
    if return_code:
        raise SystemExit(f"FFmpeg failed with exit code {return_code}:\n{stderr}")
    if not simulation.hit:
        raise SystemExit(
            "The requested simulation interval ended before interception"
        )

    print(f"Saved: {output}")
    print(
        f"Video: {args.duration:.2f} s, {args.fps} FPS, "
        f"{width}x{height}, playback x{playback_scale:.2f}"
    )
    print(f"Impact: simulation T+{simulation.hit_time_s:.3f} s")


if __name__ == "__main__":
    render_clip(parse_args())
