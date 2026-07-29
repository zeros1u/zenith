"""Deterministic command-line verification across every target behavior."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics

from zenith.simulation import InterceptionSimulation, SCENARIOS, SimulationConfig


def verify_scenario(scenario: str, duration_s: float) -> dict[str, object]:
    target_code = {
        "rocket_attack": "SR1",
        "tricky": "SEV",
    }.get(scenario, "FX1")
    sim = InterceptionSimulation(
        SimulationConfig(scenario=scenario, target_code=target_code)
    )
    minimum_range = math.inf
    for _ in range(round(duration_s * 60)):
        sim.step()
        minimum_range = min(minimum_range, sim.true_range_m)
        if sim.finished:
            break
    errors = [
        sample.range_error_m
        for sample in sim.telemetry
        if sample.range_error_m is not None
    ]
    rmse = math.sqrt(statistics.fmean(error * error for error in errors)) if errors else math.nan
    mean_absolute = statistics.fmean(abs(error) for error in errors) if errors else math.nan
    return {
        "scenario": scenario,
        "identity": sim.identity_confirmed,
        "hit": sim.hit,
        "hit_time_s": round(sim.hit_time_s, 3) if sim.hit_time_s else "",
        "minimum_range_m": round(minimum_range, 3),
        "range_mae_m": round(mean_absolute, 3),
        "range_rmse_m": round(rmse, 3),
        "samples": len(errors),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--csv", type=Path, help="optional path for a machine-readable report")
    args = parser.parse_args()
    rows = [verify_scenario(key, args.duration) for key, _ in SCENARIOS]

    headers = (
        "scenario",
        "identity",
        "hit",
        "hit_time_s",
        "minimum_range_m",
        "range_mae_m",
        "range_rmse_m",
        "samples",
    )
    print(
        f"{'SCENARIO':<14} {'ID':>3} {'HIT':>4} {'HIT(s)':>8} "
        f"{'MIN(m)':>8} {'MAE(m)':>8} {'RMSE(m)':>9} {'N':>5}"
    )
    print("-" * 71)
    for row in rows:
        print(
            f"{row['scenario']:<14} {str(row['identity']):>3} {str(row['hit']):>4} "
            f"{str(row['hit_time_s']):>8} {row['minimum_range_m']:>8} "
            f"{row['range_mae_m']:>8} {row['range_rmse_m']:>9} {row['samples']:>5}"
        )

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=headers)
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nSaved: {args.csv.resolve()}")

    return 0 if all(bool(row["identity"]) and bool(row["hit"]) for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
