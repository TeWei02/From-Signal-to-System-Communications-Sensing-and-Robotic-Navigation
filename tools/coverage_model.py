"""
coverage_model.py
-----------------
Portable, seedable re-implementation of the grid-world exploration experiment.

The reference experiment lives in ``code/multi_robot_exploration_sim.py`` and draws
its random numbers from NumPy.  NumPy's bit-generator cannot be reproduced in a
browser, so the interactive demo on the published site needs a model whose random
stream is defined by an ordinary 32-bit integer recurrence.  This module defines
that model: a Mulberry32 generator (a dozen lines of arithmetic that transpile one
to one into JavaScript) driving an exploration experiment whose rules are stated
below.  The browser engine in ``docs/engine.js`` mirrors this file line for line,
so the two implementations agree exactly rather than approximately.

Model
-----
* Grid: 40 x 40 cells, border free, interior obstacle with probability 0.15.
* Robots: two, starting at two fixed free cells, carrying a local occupancy map.
* Sensing: each step a robot observes every free cell within Chebyshev radius 3
  of its position (a 7 x 7 window, i.e. a very small fraction of a real LiDAR
  scan); about 2.6 new free cells per robot and step are learned on average.  Observations are accumulated into the robot's local map.
* Motion: frontier-biased random walk.  With probability 0.25 the robot steps to a
  uniformly random free neighbour; otherwise it steps to the free neighbour whose
  sensing window contains the most cells it has not yet observed, ties broken by
  the random stream.  This is the cheapest stand-in for frontier-based
  exploration; it is not the frontier planner described in the design documents.
* Link: every step each robot decides what to transmit to the base station.
  - ``full``: the whole local map (1600 cells) is uploaded.
  - ``diff``: only cells the base station has not yet received are uploaded, and
    at most ``budget`` of them per robot per step.  Cells that do not fit stay in
    the backlog and are retried on later steps.
* Metric: the *base-station* coverage, i.e. the fraction of reachable free cells
  the server has received.  With a tight budget the server's map lags behind the
  robots' own maps; that gap is the quantity the experiment is about.  The
  robots' local coverage is recorded as well so the two can be compared.

Usage:
    python tools/coverage_model.py --json docs/data/reference.json
    python tools/coverage_model.py --check docs/data/reference.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Environment definition (shared with the browser engine, do not drift)
# ---------------------------------------------------------------------------

ROWS = 40
COLS = 40
OBSTACLE_PROB = 0.15
N_ROBOTS = 2
STEPS = 200
SEED = 20260922
SENSOR_RADIUS = 3
RANDOM_STEP_PROB = 0.25
ENGINE_VERSION = "1.0.0"

# Per-step transmission budgets, in grid cells, for the sparse-diff strategy.
# 1600 cells = a full 40 x 40 map, so these span 0.06 % to 1 % of it.
# The sweep below covers 1..20 cells/step, which is the range in which the link
# actually binds: the robots observe about 2.6 new cells per robot per step.
SWEEP_BUDGETS = list(range(1, 21))
DIFF_BUDGETS = {"diff_1": 1, "diff_2": 2, "diff_4": 4, "diff_8": 8, "diff_16": 16}
FULL_BUDGET_CELLS = ROWS * COLS

NEIGHBOUR_ORDER = [(-1, 0), (1, 0), (0, -1), (0, 1)]

TWO32 = 1 << 32


# ---------------------------------------------------------------------------
# Mulberry32: the random stream shared by Python and JavaScript
# ---------------------------------------------------------------------------

class Mulberry32:
    """A 32-bit PRNG that is trivial to mirror in JavaScript.

    The JS implementation this mirrors is::

        function mulberry32(a) {
          return function () {
            a = (a + 0x6D2B79F5) | 0;
            let t = Math.imul(a ^ (a >>> 15), 1 | a);
            t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
            return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
          };
        }
    """

    def __init__(self, seed: int):
        self.state = seed & 0xFFFFFFFF

    def next_float(self) -> float:
        self.state = (self.state + 0x6D2B79F5) & 0xFFFFFFFF
        a = self.state
        t = (a ^ (a >> 15)) & 0xFFFFFFFF
        t = (t * (1 | a)) & 0xFFFFFFFF
        t = ((t + ((t ^ (t >> 7)) * (61 | t)) & 0xFFFFFFFF) & 0xFFFFFFFF) ^ t
        t &= 0xFFFFFFFF
        return ((t ^ (t >> 14)) & 0xFFFFFFFF) / TWO32

    def index(self, n: int) -> int:
        """Uniform integer in [0, n) using the same mapping as the browser."""
        if n <= 0:
            return 0
        return min(int(self.next_float() * n), n - 1)


# ---------------------------------------------------------------------------
# Grid world
# ---------------------------------------------------------------------------

def build_grid(rng: Mulberry32) -> list[list[int]]:
    """Occupancy grid; border cells are free, interior cells are obstacles with
    probability ``OBSTACLE_PROB``.  One draw per cell, row-major."""
    grid = [[0] * COLS for _ in range(ROWS)]
    for r in range(ROWS):
        for c in range(COLS):
            occupied = 1 if rng.next_float() < OBSTACLE_PROB else 0
            if r == 0 or r == ROWS - 1 or c == 0 or c == COLS - 1:
                occupied = 0
            grid[r][c] = occupied
    return grid


def free_cells(grid: list[list[int]]) -> list[tuple[int, int]]:
    return [(r, c) for r in range(ROWS) for c in range(COLS) if grid[r][c] == 0]


def sensing_window(grid: list[list[int]], pos: tuple[int, int]) -> list[tuple[int, int]]:
    """Free cells within Chebyshev radius ``SENSOR_RADIUS`` of ``pos``."""
    r0, c0 = pos
    window = []
    for r in range(max(0, r0 - SENSOR_RADIUS), min(ROWS, r0 + SENSOR_RADIUS + 1)):
        for c in range(max(0, c0 - SENSOR_RADIUS), min(COLS, c0 + SENSOR_RADIUS + 1)):
            if grid[r][c] == 0:
                window.append((r, c))
    return window


def neighbours(grid: list[list[int]], pos: tuple[int, int]) -> list[tuple[int, int]]:
    r, c = pos
    return [
        (r + dr, c + dc)
        for dr, dc in NEIGHBOUR_ORDER
        if 0 <= r + dr < ROWS and 0 <= c + dc < COLS and grid[r + dr][c + dc] == 0
    ]


# ---------------------------------------------------------------------------
# Exploration experiment
# ---------------------------------------------------------------------------

def run_experiment(grid: list[list[int]], strategy: str, budget_cells: int,
                   seed: int) -> dict:
    """Run one transmission strategy and record coverage and traffic.

    Args:
        grid: occupancy grid (0 = free, 1 = obstacle).
        strategy: ``"full"`` (upload the whole local map) or ``"diff"``
            (upload at most ``budget_cells`` not-yet-received cells per step).
        budget_cells: per-robot, per-step cell budget; ignored for ``"full"``.
        seed: seed for the Mulberry32 stream of this run.

    Returns:
        dict with the base-station coverage series, the robots' local coverage
        series, the transmitted cell count and the final base-station coverage.
    """
    if strategy not in {"full", "diff"}:
        raise ValueError(f"unknown strategy: {strategy}")

    rng = Mulberry32(seed)
    rng.next_float()  # one draw of separation from the grid-generation stream

    free = free_cells(grid)
    reachable = len(free)

    starts = [free[i * (reachable // N_ROBOTS)] for i in range(N_ROBOTS)]

    local_maps = [set() for _ in starts]
    received = [set() for _ in starts]   # what the base station got from robot i
    positions = list(starts)

    for i, pos in enumerate(positions):
        local_maps[i] |= set(sensing_window(grid, pos))

    coverage: list[float] = []
    local_coverage: list[float] = []
    transmitted_cells = 0
    backlog_cells = 0

    for _ in range(STEPS):
        for i in range(N_ROBOTS):
            # --- motion: frontier-biased random walk -------------------------
            candidates = neighbours(grid, positions[i])
            if candidates:
                if rng.next_float() < RANDOM_STEP_PROB or len(candidates) == 1:
                    choice = candidates[rng.index(len(candidates))]
                else:
                    scores = [
                        sum(1 for cell in sensing_window(grid, cand) if cell not in local_maps[i])
                        for cand in candidates
                    ]
                    best = max(scores)
                    best_set = [c for c, s in zip(candidates, scores) if s == best]
                    choice = best_set[rng.index(len(best_set))]
                positions[i] = choice
            # --- sensing ------------------------------------------------------
            local_maps[i] |= set(sensing_window(grid, positions[i]))

            # --- transmission -------------------------------------------------
            if strategy == "full":
                received[i] = set(local_maps[i])
                transmitted_cells += FULL_BUDGET_CELLS
            else:
                pending = [cell for cell in sorted(local_maps[i]) if cell not in received[i]]
                backlog_cells += max(0, len(pending) - budget_cells)
                for cell in pending[:budget_cells]:
                    received[i].add(cell)
                transmitted_cells += min(len(pending), budget_cells)

        server_map = received[0] | received[1]
        local_view = local_maps[0] | local_maps[1]
        coverage.append(round(len(server_map) / reachable, 12))
        local_coverage.append(round(len(local_view) / reachable, 12))

    return {
        "strategy": strategy,
        "budget_cells_per_step": FULL_BUDGET_CELLS if strategy == "full" else budget_cells,
        "coverage": coverage,
        "local_coverage": local_coverage,
        "transmitted_cells": transmitted_cells,
        "backlog_cells": backlog_cells,
        "final_coverage": coverage[-1],
        "final_local_coverage": local_coverage[-1],
    }


def build_dataset() -> dict:
    """Build the complete reference dataset used by the published demo."""
    grid = build_grid(Mulberry32(SEED))

    runs = {"full": run_experiment(grid, "full", FULL_BUDGET_CELLS, SEED + 1)}
    for name in sorted(DIFF_BUDGETS):
        runs[name] = run_experiment(grid, "diff", DIFF_BUDGETS[name], SEED + 1)

    free = free_cells(grid)
    reachable = len(free)
    starts = [free[i * (reachable // N_ROBOTS)] for i in range(N_ROBOTS)]

    sweep = []
    for budget in SWEEP_BUDGETS:
        run = run_experiment(grid, "diff", budget, SEED + 1)
        sweep.append({"budget_cells_per_step": budget,
                      "final_coverage": run["final_coverage"],
                      "transmitted_cells": run["transmitted_cells"]})

    return {
        "sweep": sweep,
        "meta": {
            "engine_version": ENGINE_VERSION,
            "generated_by": "tools/coverage_model.py",
            "seed": SEED,
            "steps": STEPS,
            "rows": ROWS,
            "cols": COLS,
            "obstacle_prob": OBSTACLE_PROB,
            "robots": N_ROBOTS,
            "sensor_radius": SENSOR_RADIUS,
            "random_step_prob": RANDOM_STEP_PROB,
            "prng": "mulberry32",
            "note": (
                "Portable re-implementation of the reference grid-world experiment. "
                "Every number here is produced by this file; the browser engine "
                "re-derives them from the same seed and must agree exactly."
            ),
        },
        "grid": "".join(str(grid[r][c]) for r in range(ROWS) for c in range(COLS)),
        "reachable_cells": reachable,
        "start_indices": [s[0] * COLS + s[1] for s in starts],
        "runs": runs,
    }


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def compare_datasets(committed: dict, derived: dict) -> list[str]:
    """Return a list of human-readable differences between two datasets."""
    problems: list[str] = []
    if committed.get("grid") != derived.get("grid"):
        problems.append("grid differs")
    if committed.get("reachable_cells") != derived.get("reachable_cells"):
        problems.append("reachable_cells differs")
    if committed.get("start_indices") != derived.get("start_indices"):
        problems.append("start_indices differs")
    for name, run in derived["runs"].items():
        other = committed.get("runs", {}).get(name)
        if other is None:
            problems.append(f"{name}: missing from committed file")
            continue
        if other.get("transmitted_cells") != run["transmitted_cells"]:
            problems.append(
                f"{name}: transmitted_cells {other.get('transmitted_cells')} != "
                f"{run['transmitted_cells']}"
            )
        for key in ("coverage", "local_coverage"):
            series_a = other.get(key, [])
            series_b = run[key]
            if len(series_a) != len(series_b):
                problems.append(f"{name}.{key}: length {len(series_a)} != {len(series_b)}")
                continue
            deltas = [abs(x - y) for x, y in zip(series_a, series_b)]
            if deltas and max(deltas) > 0.0:
                problems.append(f"{name}.{key}: max |delta| {max(deltas):.3e}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description="grid-world coverage model")
    parser.add_argument("--json", type=str, default="",
                        help="write the dataset to this path")
    parser.add_argument("--check", type=str, default="",
                        help="re-derive the dataset and compare it with this file")
    args = parser.parse_args()

    dataset = build_dataset()

    if args.check:
        committed = json.loads(Path(args.check).read_text())
        problems = compare_datasets(committed, dataset)
        for line in problems:
            print(line)
        if problems:
            print(f"check failed: {len(problems)} difference(s)")
            return 1
        print(f"check passed: {args.check} matches the model output")
        return 0

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(dataset, indent=1) + "\n")
        print(f"wrote {args.json}")
    else:
        for name, run in dataset["runs"].items():
            print(
                f"{name:>10}: server coverage {run['final_coverage']:.4f} "
                f"| local coverage {run['final_local_coverage']:.4f} "
                f"| {run['transmitted_cells']} cells / {STEPS} steps "
                f"| backlog {run['backlog_cells']}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
