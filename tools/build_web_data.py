"""
build_web_data.py
-----------------
Build the reference dataset consumed by the static demo in ``docs/``.

The demo is a browser-only re-implementation of three artefacts from the
repository, so it must be able to check itself against numbers produced by
Python.  This script is the Python side of that contract: it runs

1. ``tools/coverage_model.py`` — the portable grid-world exploration model
   (same rules as ``code/multi_robot_exploration_sim.py``, reproducible in JS),
2. ``code/multi_robot_exploration_sim.py`` — the original NumPy implementation,
   kept as a cross-check of the portable model,
3. ``analysis/plot_latency_vs_nav_error.py`` — the dead-band latency model,
4. ``analysis/plot_bandwidth_vs_update_rate.py`` — the LiDAR link-budget model,

and writes the results to ``docs/data/reference.json`` together with a checksum.
``docs/engine.js`` re-derives the same quantities in the browser and reports any
mismatch in the page's verification panel.

Usage:
    python tools/build_web_data.py [--out docs/data/reference.json]
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tools"))

import coverage_model


def load_module(name: str, relative_path: str):
    """Import a repository module from its file path."""
    path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {relative_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_latency_section() -> dict:
    """Dead-band latency model, driven by the repository's analysis script."""
    latency = load_module("plot_latency_vs_nav_error", "analysis/plot_latency_vs_nav_error.py")

    speeds = [0.5, 1.0, 1.5, 2.0]
    rtt_ms = list(range(0, 501, 10))
    jitter_sigma_ms = 20
    acceptable_error_m = 0.1

    return {
        "model": "dead-band displacement v * tau",
        "source": "analysis/plot_latency_vs_nav_error.py",
        "rtt_ms": rtt_ms,
        "speeds_mps": speeds,
        "error_m": {
            f"{speed:.1f}": [latency.compute_deadband_error(speed, tau) for tau in rtt_ms]
            for speed in speeds
        },
        "jitter": {
            "speed_mps": 1.0,
            "sigma_ms": jitter_sigma_ms,
            "lower_m": [
                latency.compute_deadband_error(1.0, max(0, tau - jitter_sigma_ms))
                for tau in rtt_ms
            ],
            "upper_m": [
                latency.compute_deadband_error(1.0, tau + jitter_sigma_ms)
                for tau in rtt_ms
            ],
        },
        "acceptable_error_m": acceptable_error_m,
        "max_rtt_within_acceptable_ms": {
            f"{speed:.1f}": 1000.0 * acceptable_error_m / speed for speed in speeds
        },
        "caveat": (
            "First-order kinematic model: the robot is assumed to keep its velocity "
            "until the delayed update arrives, and no controller, estimator or "
            "environment interaction is simulated. Values are analytical, not measured."
        ),
    }


def build_link_budget_section() -> dict:
    """LiDAR link-budget model, driven by the repository's analysis script."""
    link = load_module("plot_bandwidth_vs_update_rate", "analysis/plot_bandwidth_vs_update_rate.py")
    strategies = list(link.LiDARCompression)

    rates = list(range(1, 21))
    bandwidths = [0.5, 1.0, 2.0, 5.0, 10.0, 30.0]

    curves = {}
    for compression in strategies:
        curves[compression.value] = {
            "base_mbps_at_10hz": link.estimate_lidar_bandwidth(10, compression),
            "required_mbps": [link.estimate_lidar_bandwidth(rate, compression) for rate in rates],
        }

    achievable = []
    for bandwidth in bandwidths:
        entry = {"bandwidth_mbps": bandwidth}
        for compression in strategies:
            max_rate = 0
            for rate in range(1, 31):
                if link.estimate_lidar_bandwidth(rate, compression) <= bandwidth:
                    max_rate = rate
                else:
                    break
            entry[compression.value] = max_rate
        achievable.append(entry)

    return {
        "source": "analysis/plot_bandwidth_vs_update_rate.py",
        "rates_hz": rates,
        "bandwidth_options_mbps": bandwidths,
        "curves": curves,
        "achievable_rate_hz": achievable,
        "caveat": (
            "The three per-strategy rates (50 / 5 / 0.5 Mbps at 10 Hz) are "
            "order-of-magnitude figures taken from LiDAR data-sheet and interface "
            "budgets, and are linear in the update rate by construction. They are "
            "planning assumptions, not measurements from a testbed."
        ),
    }


def build_numpy_reference() -> dict:
    """Run the original NumPy simulation as a cross-check of the portable model."""
    sim = load_module("multi_robot_exploration_sim", "code/multi_robot_exploration_sim.py")
    import numpy as np

    rows = getattr(sim, "GRID_ROWS", 30)
    cols = getattr(sim, "GRID_COLS", 30)
    steps = getattr(sim, "MAX_STEPS", 300)
    diff_budget = getattr(sim, "DIFF_BUDGET_CELLS_PER_STEP", 45)
    seed = getattr(sim, "RNG_SEED", 0)

    grid = sim.build_grid(rows, cols, obstacle_prob=0.15, rng=np.random.default_rng(seed))
    free = sim.free_cells(grid)
    starts = [free[0], free[len(free) // 2]]

    full = sim.run_simulation(
        strategy="full", grid=grid, start_positions=starts, n_steps=steps,
        budget_cells_per_step=rows * cols, rng=np.random.default_rng(seed),
    )
    diff = sim.run_simulation(
        strategy="diff", grid=grid, start_positions=starts, n_steps=steps,
        budget_cells_per_step=diff_budget, rng=np.random.default_rng(seed),
    )

    return {
        "source": "code/multi_robot_exploration_sim.py",
        "description": (
            "Original NumPy implementation: sensor-free random walk (a robot marks "
            "only the cells it walks over), two strategies, partial diffs chosen as "
            "a random subset."
        ),
        "rows": rows,
        "cols": cols,
        "steps": steps,
        "obstacle_prob": 0.15,
        "seed": seed,
        "diff_budget_cells_per_step": diff_budget,
        "reachable_cells": len(free),
        "final_coverage_full": full[-1],
        "final_coverage_diff": diff[-1],
    }


def canonical(payload: dict) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def build_dataset() -> dict:
    exploration = coverage_model.build_dataset()
    dataset = {
        "meta": {
            "project": "From Signal to System",
            "repository": (
                "https://github.com/TeWei02/"
                "From-Signal-to-System-Communications-Sensing-and-Robotic-Navigation"
            ),
            "generated_by": "tools/build_web_data.py",
            "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "python": platform.python_version(),
            "scope": (
                "Reference numbers for the browser demo. Everything below is produced "
                "by the repository's own Python models; no experimental measurement, "
                "no ROS 2 run and no hardware result is included."
            ),
        },
        "exploration": exploration,
        "latency": build_latency_section(),
        "link_budget": build_link_budget_section(),
        "numpy_reference": build_numpy_reference(),
    }
    # The checksum covers the numbers only: ``meta`` carries a build timestamp
    # and the local Python version, so including it would make the digest differ
    # between two runs that produced identical data.
    numbers_only = {key: value for key, value in dataset.items() if key != "meta"}
    dataset["checksum"] = hashlib.sha256(canonical(numbers_only).encode()).hexdigest()
    return dataset


def main() -> int:
    parser = argparse.ArgumentParser(description="build the demo reference dataset")
    parser.add_argument("--out", default="docs/data/reference.json",
                        help="output path (relative to the repository root)")
    args = parser.parse_args()

    dataset = build_dataset()
    out_path = REPO_ROOT / args.out
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(dataset, indent=1, sort_keys=False) + "\n")

    size_kb = out_path.stat().st_size / 1024
    print(f"wrote {out_path} ({size_kb:.1f} kB)")
    print(f"checksum {dataset['checksum']}")
    for name, run in dataset["exploration"]["runs"].items():
        print(f"  {name:>9}: server coverage {run['final_coverage']:.4f} "
              f"| local {run['final_local_coverage']:.4f} "
              f"| transmitted {run['transmitted_cells']} cells")
    print(f"  numpy reference: full {dataset['numpy_reference']['final_coverage_full']:.4f} "
          f"| diff {dataset['numpy_reference']['final_coverage_diff']:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
