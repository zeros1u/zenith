"""Repeatable CPU benchmark for the INTERCEPTRON simulation and software renderer."""

from __future__ import annotations

import argparse
import gc
import os
import statistics
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from zenith.rendering import WorldRenderer
from zenith.simulation import InterceptionSimulation, SimulationConfig


FIXED_STEP = 1.0 / 60.0
BENCHMARK_SIZE = (1050, 700)


def simulation_ticks_per_second(ticks: int, enemy_count: int = 1) -> float:
    simulation = InterceptionSimulation(
        SimulationConfig(scenario="evasive", enemy_count=enemy_count)
    )
    started = time.perf_counter()
    for _ in range(ticks):
        simulation.step(FIXED_STEP)
    elapsed = time.perf_counter() - started
    return ticks / max(elapsed, 1e-9)


def complete_frames_per_second(frames: int, enemy_count: int = 1) -> float:
    simulation = InterceptionSimulation(
        SimulationConfig(scenario="evasive", enemy_count=enemy_count)
    )
    for _ in range(120):
        simulation.step(FIXED_STEP)
    renderer = WorldRenderer()
    surface = pygame.Surface(BENCHMARK_SIZE)
    started = time.perf_counter()
    for _ in range(frames):
        simulation.step(FIXED_STEP)
        renderer.draw_world(surface, simulation)
        renderer.draw_hud(surface, simulation, 0.5, 60.0)
    elapsed = time.perf_counter() - started
    return frames / max(elapsed, 1e-9)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark INTERCEPTRON without opening a window"
    )
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--ticks", type=int, default=300)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument(
        "--enemies",
        type=int,
        choices=(1, 2, 3),
        default=1,
        help="number of independently tracked enemy contacts",
    )
    args = parser.parse_args()
    if min(args.repeats, args.ticks, args.frames) <= 0:
        parser.error("repeats, ticks, and frames must be positive")

    pygame.init()
    gc.disable()
    try:
        tick_results = [
            simulation_ticks_per_second(args.ticks, args.enemies)
            for _ in range(args.repeats)
        ]
        frame_results = [
            complete_frames_per_second(args.frames, args.enemies)
            for _ in range(args.repeats)
        ]
    finally:
        gc.enable()
        pygame.quit()

    tick_median = statistics.median(tick_results)
    frame_median = statistics.median(frame_results)
    print(
        f"physics + guidance: {tick_median:.1f} ticks/s "
        f"({tick_median / 60.0:.1f}x required 60 Hz), "
        f"{args.enemies} contact(s)"
    )
    print(
        f"world + HUD + one tick: {frame_median:.1f} frames/s "
        f"at {BENCHMARK_SIZE[0]}x{BENCHMARK_SIZE[1]}"
    )
    print(
        "individual tick runs: "
        + ", ".join(f"{value:.1f}" for value in tick_results)
    )
    print(
        "individual frame runs: "
        + ", ".join(f"{value:.1f}" for value in frame_results)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
