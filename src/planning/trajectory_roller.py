"""
trajectory_roller.py
---------------------
Forward-simulates a set of candidate trajectory arcs and selects the admissible
trajectory with the lowest composite cost.

Used by the local planner as a trajectory evaluation helper; can also be invoked
standalone for offline sweep experiments.

TODO:
    - Add centripetal acceleration constraint.
    - Support model predictive control (MPC) formulation with a finite-horizon cost.
    - Vectorise arc simulation with numpy for 10× speedup.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class ArcConfig:
    """Configuration for trajectory arc sampling."""
    v_min: float = 0.0
    v_max: float = 0.5
    w_min: float = -1.0
    w_max: float = 1.0
    n_v_samples: int = 5
    n_w_samples: int = 11
    sim_time: float = 2.0
    dt: float = 0.05
    robot_radius: float = 0.20


@dataclass
class ArcResult:
    """A scored candidate arc trajectory."""
    v: float
    omega: float
    poses: np.ndarray     # (T, 3) [x, y, yaw]
    cost: float = float("inf")
    feasible: bool = True
    reason: str = ""      # why infeasible, if applicable


def roll_arc(x0: float, y0: float, yaw0: float,
             v: float, omega: float,
             sim_time: float, dt: float) -> np.ndarray:
    """Simulate a single constant-(v, ω) arc.

    Args:
        x0, y0, yaw0: Initial pose.
        v: Linear velocity (m/s).
        omega: Angular velocity (rad/s).
        sim_time: Simulation horizon (s).
        dt: Timestep (s).

    Returns:
        (T, 3) pose array [x, y, yaw].
    """
    n = int(sim_time / dt)
    poses = np.empty((n, 3))
    x, y, yaw = x0, y0, yaw0
    for i in range(n):
        x   += v * math.cos(yaw) * dt
        y   += v * math.sin(yaw) * dt
        yaw += omega * dt
        poses[i] = [x, y, yaw]
    return poses


def check_arc_feasibility(poses: np.ndarray,
                            costmap: np.ndarray,
                            resolution: float,
                            origin_x: float,
                            origin_y: float,
                            robot_radius: float,
                            lethal_threshold: int = 90) -> Tuple[bool, str]:
    """Check whether an arc trajectory is collision-free.

    Returns:
        (feasible, reason) tuple.

    TODO:
        - Check footprint polygon rather than a single radius circle.
    """
    rows, cols = costmap.shape
    for pose in poses:
        col = int((pose[0] - origin_x) / resolution)
        row = int((pose[1] - origin_y) / resolution)
        if not (0 <= col < cols and 0 <= row < rows):
            return False, "out_of_bounds"
        if costmap[row, col] >= lethal_threshold:
            return False, "lethal_cost"
    return True, ""


def score_arc(poses: np.ndarray,
              goal: Tuple[float, float],
              v: float,
              costmap: np.ndarray,
              resolution: float,
              origin_x: float,
              origin_y: float,
              w_goal: float = 1.0,
              w_clearance: float = 0.5,
              w_velocity: float = 0.2) -> float:
    """Compute a scalar cost for a trajectory arc.

    Lower cost = better trajectory.

    Args:
        poses: (T, 3) simulated trajectory.
        goal: (gx, gy) goal position.
        v: Linear velocity of the trajectory.
        costmap: 2-D costmap.
        resolution, origin_x, origin_y: Map geometry.
        w_goal, w_clearance, w_velocity: Objective weights.

    Returns:
        Scalar cost (lower is better).
    """
    final = poses[-1]
    dist_to_goal = math.hypot(goal[0] - final[0], goal[1] - final[1])
    goal_cost = dist_to_goal

    # Clearance cost: prefer trajectories away from obstacles
    rows, cols = costmap.shape
    max_cost = 0
    for pose in poses:
        c = int((pose[0] - origin_x) / resolution)
        r = int((pose[1] - origin_y) / resolution)
        if 0 <= c < cols and 0 <= r < rows:
            max_cost = max(max_cost, int(costmap[r, c]))
    clearance_cost = max_cost / 100.0

    # Velocity cost: penalise slow motion
    velocity_cost = 1.0 - (v / 0.5) if v >= 0 else 1.0

    return w_goal * goal_cost + w_clearance * clearance_cost + w_velocity * velocity_cost


class TrajectoryRoller:
    """Samples, simulates, and scores arc trajectories to select the best command.

    TODO:
        - Cache feasible arcs to avoid recomputing unchanged candidates.
        - Add a recovery behaviour when all arcs are infeasible.
    """

    def __init__(self, cfg: Optional[ArcConfig] = None) -> None:
        self._cfg = cfg or ArcConfig()

    def roll(self,
             pose: Tuple[float, float, float],
             goal: Tuple[float, float],
             costmap: np.ndarray,
             resolution: float = 0.05,
             origin: Tuple[float, float] = (-20.0, -20.0)) -> Optional[ArcResult]:
        """Roll all candidate arcs and return the best one.

        Args:
            pose: Current (x, y, yaw).
            goal: Target (gx, gy).
            costmap: 2-D costmap.
            resolution: Metres per cell.
            origin: (ox, oy) map origin.

        Returns:
            Best ArcResult, or None if no feasible trajectory exists.
        """
        cfg = self._cfg
        v_values = np.linspace(cfg.v_min, cfg.v_max, cfg.n_v_samples)
        w_values = np.linspace(cfg.w_min, cfg.w_max, cfg.n_w_samples)

        best: Optional[ArcResult] = None
        for v in v_values:
            for w in w_values:
                poses = roll_arc(*pose, v, w, cfg.sim_time, cfg.dt)
                feasible, reason = check_arc_feasibility(
                    poses, costmap, resolution, origin[0], origin[1],
                    cfg.robot_radius
                )
                cost = float("inf")
                if feasible:
                    cost = score_arc(poses, goal, v, costmap, resolution,
                                     origin[0], origin[1])
                arc = ArcResult(v=v, omega=w, poses=poses, cost=cost,
                                 feasible=feasible, reason=reason)
                if feasible and (best is None or arc.cost < best.cost):
                    best = arc

        return best
