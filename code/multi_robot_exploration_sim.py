"""
multi_robot_exploration_sim.py
--------------------------------
A simple discrete-time, grid-world simulation of two robots exploring an
unknown indoor environment under a constrained wireless channel.

Two strategies are compared:
  A) Full map upload  — robots send their complete local occupancy map at
                        every time step (high bandwidth, always-consistent
                        global map on the server).
  B) Sparse diff      — robots send only changed cells since the last
                        transmission (low bandwidth, eventual consistency).

Output metric: coverage fraction over time (fraction of reachable cells explored).

Figure produced: figures/multi_robot_coverage.pdf
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false

import numpy as np
import matplotlib.pyplot as plt
import os
from typing import Optional


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "figures")

GRID_ROWS = 30
GRID_COLS = 30
N_ROBOTS = 2
MAX_STEPS = 300
RNG_SEED = 0

# Bandwidth parameters (fraction of map cells transmittable per step)
FULL_MAP_BUDGET_CELLS_PER_STEP = GRID_ROWS * GRID_COLS  # unlimited (full map)
DIFF_BUDGET_CELLS_PER_STEP = int(0.05 * GRID_ROWS * GRID_COLS)  # 5% of cells


# ---------------------------------------------------------------------------
# Grid world helpers
# ---------------------------------------------------------------------------

def build_grid(rows: int, cols: int, obstacle_prob: float = 0.15,
               rng: Optional[np.random.Generator] = None) -> np.ndarray:
    """
    Create a binary occupancy grid (0 = free, 1 = obstacle).
    Border cells are always free; interior cells are obstacles with
    probability `obstacle_prob`.

    Parameters
    ----------
    rows, cols : int
        Grid dimensions.
    obstacle_prob : float
        Probability of each interior cell being an obstacle.
    rng : np.random.Generator
        Random number generator.

    Returns
    -------
    np.ndarray of shape (rows, cols), dtype int
        Occupancy grid (0 = free, 1 = obstacle).
    """
    if rng is None:
        rng = np.random.default_rng(RNG_SEED)
    grid = (rng.random((rows, cols)) < obstacle_prob).astype(int)
    # Ensure border is free for robot spawning
    grid[0, :] = 0
    grid[-1, :] = 0
    grid[:, 0] = 0
    grid[:, -1] = 0
    return grid


def free_cells(grid: np.ndarray) -> list[tuple[int, int]]:
    """Return list of (row, col) tuples for all free cells."""
    return list(zip(*np.where(grid == 0)))


def neighbours(pos: tuple[int, int], grid: np.ndarray) -> list[tuple[int, int]]:
    """Return list of free 4-connected neighbour positions of `pos`."""
    r, c = pos
    rows, cols = grid.shape
    candidates = [(r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)]
    return [
        (nr, nc)
        for nr, nc in candidates
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] == 0
    ]


# ---------------------------------------------------------------------------
# Robot
# ---------------------------------------------------------------------------

class Robot:
    """
    A simple random-walk explorer that marks cells as explored.

    Attributes
    ----------
    pos : tuple (row, col)
        Current position on the grid.
    local_map : np.ndarray
        Binary array: 1 if the cell has been observed by this robot, else 0.
    last_sent_map : np.ndarray
        Snapshot of local_map at the time of the last transmission.
    """

    def __init__(self, start_pos: tuple[int, int], grid: np.ndarray):
        self.pos = start_pos
        rows, cols = grid.shape
        self.local_map = np.zeros((rows, cols), dtype=np.uint8)
        self.local_map[start_pos] = 1
        self.last_sent_map = self.local_map.copy()
        self._grid = grid

    def step(self, rng: np.random.Generator) -> None:
        """
        Move to a random free neighbour (random walk).
        TODO: replace with a frontier-based exploration policy.
        """
        nbrs = neighbours(self.pos, self._grid)
        if nbrs:
            self.pos = nbrs[rng.integers(len(nbrs))]
            self.local_map[self.pos] = 1

    def get_diff(self) -> np.ndarray:
        """
        Return the binary diff between current local_map and last_sent_map.

        Returns
        -------
        np.ndarray
            Boolean array where True indicates a newly explored cell.
        """
        return self.local_map.astype(bool) & ~self.last_sent_map.astype(bool)

    def acknowledge_sent(self) -> None:
        """Update last_sent_map to current local_map after a transmission."""
        self.last_sent_map = self.local_map.copy()


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------

class EdgeServer:
    """
    Central coordinator that merges robot maps and assigns frontiers.

    Attributes
    ----------
    global_map : np.ndarray
        Merged occupancy map (1 = explored by any robot, 0 = unknown).
    """

    def __init__(self, rows: int, cols: int):
        self.global_map = np.zeros((rows, cols), dtype=np.uint8)

    def receive_full_map(self, local_map: np.ndarray) -> None:
        """Merge a full local map into the global map."""
        self.global_map |= local_map

    def receive_diff(self, diff: np.ndarray) -> None:
        """Merge a sparse differential update into the global map."""
        self.global_map[diff] = 1

    def coverage(self, reachable_count: int) -> float:
        """
        Return the fraction of reachable cells that have been explored.

        Parameters
        ----------
        reachable_count : int
            Total number of free (reachable) cells in the environment.
        """
        if reachable_count == 0:
            return 0.0
        return float(np.sum(self.global_map)) / reachable_count


# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------

def run_simulation(
    strategy: str,
    grid: np.ndarray,
    start_positions: list[tuple[int, int]],
    n_steps: int,
    budget_cells_per_step: int,
    rng: np.random.Generator,
) -> list[float]:
    """
    Run the exploration simulation for a given strategy.

    Parameters
    ----------
    strategy : str
        'full' for Strategy A (full map upload) or 'diff' for Strategy B.
    grid : np.ndarray
        Binary occupancy grid (0 = free, 1 = obstacle).
    start_positions : list of (row, col)
        Starting positions for each robot.
    n_steps : int
        Number of simulation steps.
    budget_cells_per_step : int
        Maximum number of cells transmittable per robot per step.
    rng : np.random.Generator
        Random number generator.

    Returns
    -------
    list of float
        Coverage fraction at each time step.
    """
    rows, cols = grid.shape
    robots = [Robot(pos, grid) for pos in start_positions]
    server = EdgeServer(rows, cols)
    reachable = len(free_cells(grid))

    coverage_history = []

    for _ in range(n_steps):
        for robot in robots:
            robot.step(rng)

            if strategy == "full":
                # TODO: model serialisation/compression overhead
                server.receive_full_map(robot.local_map)
                robot.acknowledge_sent()

            elif strategy == "diff":
                diff = robot.get_diff()
                n_changed = int(np.sum(diff))
                if n_changed > 0:
                    # Transmit only up to the bandwidth budget
                    if n_changed <= budget_cells_per_step:
                        server.receive_diff(diff)
                        robot.acknowledge_sent()
                    else:
                        # Partial transmission: pick a random subset
                        # TODO: prioritise frontier cells over interior cells
                        changed_indices = np.argwhere(diff)
                        chosen = changed_indices[
                            rng.choice(len(changed_indices),
                                       budget_cells_per_step,
                                       replace=False)
                        ]
                        partial_diff = np.zeros_like(diff)
                        partial_diff[tuple(chosen.T)] = True
                        server.receive_diff(partial_diff)
                        # Do NOT acknowledge; unsent cells remain in next diff

        coverage_history.append(server.coverage(reachable))

    return coverage_history


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_coverage(
    coverage_full: list[float],
    coverage_diff: list[float],
    n_steps: int,
) -> None:
    """
    Plot coverage fraction over time for both strategies.

    Parameters
    ----------
    coverage_full : list of float
        Coverage history for Strategy A (full map upload).
    coverage_diff : list of float
        Coverage history for Strategy B (sparse diff).
    n_steps : int
        Total simulation steps.
    """
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    steps = np.arange(1, n_steps + 1)
    fig, ax = plt.subplots(figsize=(7, 4))

    ax.plot(steps, coverage_full, linewidth=2, label="Strategy A: Full map upload")
    ax.plot(steps, coverage_diff, linewidth=2, linestyle="--",
            label="Strategy B: Sparse diff update")

    ax.set_xlabel("Simulation Step")
    ax.set_ylabel("Coverage Fraction")
    ax.set_title("Multi-Robot Exploration Coverage: Full Upload vs. Sparse Diff")
    ax.set_ylim(0.0, 1.05)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    out_path = os.path.join(OUTPUT_DIR, "multi_robot_coverage.pdf")
    fig.tight_layout()
    fig.savefig(out_path)
    print(f"Saved figure to {out_path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    rng_main = np.random.default_rng(RNG_SEED)

    # Build environment
    grid = build_grid(GRID_ROWS, GRID_COLS, obstacle_prob=0.15, rng=rng_main)

    # Choose start positions (two corners of the free space)
    all_free = free_cells(grid)
    start_positions = [all_free[0], all_free[len(all_free) // 2]]

    # Strategy A: full map upload (unlimited bandwidth)
    cov_full = run_simulation(
        strategy="full",
        grid=grid,
        start_positions=start_positions,
        n_steps=MAX_STEPS,
        budget_cells_per_step=FULL_MAP_BUDGET_CELLS_PER_STEP,
        rng=np.random.default_rng(RNG_SEED),
    )

    # Strategy B: sparse diff (5% bandwidth budget)
    cov_diff = run_simulation(
        strategy="diff",
        grid=grid,
        start_positions=start_positions,
        n_steps=MAX_STEPS,
        budget_cells_per_step=DIFF_BUDGET_CELLS_PER_STEP,
        rng=np.random.default_rng(RNG_SEED),
    )

    plot_coverage(cov_full, cov_diff, MAX_STEPS)

    # Print a brief summary
    print(f"Final coverage — Full upload: {cov_full[-1]:.2%}  |  "
          f"Sparse diff: {cov_diff[-1]:.2%}")
