"""
local_planner.py
-----------------
Local planner: short-horizon trajectory optimization using a Dynamic Window
Approach (DWA) inspired method.

Given the current robot pose, velocity, and a costmap, generates a set of
candidate arc trajectories, scores them against an objective function, and
returns the best (v, ω) velocity command.

The objective function balances:
    - Goal heading alignment
    - Obstacle clearance (from costmap)
    - Velocity (prefer faster motion)
    - Path deviation (stay close to global reference path)

References:
    Fox, Burgard, Thrun, "The Dynamic Window Approach to Collision Avoidance",
    IEEE Robotics & Automation Magazine, 1997.
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false, reportPossiblyUnboundVariable=false, reportUnusedImport=false

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

ros2_available = False
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, Path, Odometry
    from geometry_msgs.msg import Twist, PoseStamped
    ros2_available = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class DWAConfig:
    """Dynamic window approach parameters."""
    # Kinematic limits
    max_v: float = 0.5           # m/s
    min_v: float = 0.0           # m/s (allow in-place rotation only when negative)
    max_w: float = 1.0           # rad/s
    max_accel: float = 0.3       # m/s²
    max_ang_accel: float = 1.5   # rad/s²

    # Sampling
    v_resolution: int = 10       # samples in linear velocity window
    w_resolution: int = 20       # samples in angular velocity window
    sim_time: float = 2.0        # seconds to forward-simulate each trajectory
    dt: float = 0.1              # simulation time step (s)

    # Objective weights
    w_heading: float = 0.15
    w_clearance: float = 0.10
    w_velocity: float = 0.05
    w_path: float = 0.20

    # Safety
    robot_radius: float = 0.20   # metres (used for collision checking)
    min_clearance: float = 0.05  # discard trajectories with clearance below this


# ---------------------------------------------------------------------------
# Trajectory simulation
# ---------------------------------------------------------------------------

@dataclass
class Trajectory:
    """A simulated arc trajectory."""
    v: float                          # linear velocity (m/s)
    omega: float                      # angular velocity (rad/s)
    poses: np.ndarray                 # (T, 3) array of [x, y, yaw] waypoints
    min_clearance: float = 0.0        # minimum obstacle clearance along trajectory
    score: float = -float("inf")


def simulate_trajectory(x: float, y: float, yaw: float,
                         v: float, omega: float,
                         cfg: DWAConfig) -> Trajectory:
    """Forward-simulate a constant-(v, ω) arc trajectory.

    Args:
        x, y, yaw: Current robot pose.
        v: Candidate linear velocity (m/s).
        omega: Candidate angular velocity (rad/s).
        cfg: DWA configuration.

    Returns:
        Trajectory with simulated poses.

    TODO:
        - Replace kinematic model with a more accurate bicycle model.
        - Add centripetal acceleration limit check.
    """
    n_steps = int(cfg.sim_time / cfg.dt)
    poses = np.zeros((n_steps, 3))
    cx, cy, cyaw = x, y, yaw

    for i in range(n_steps):
        cx   += v * math.cos(cyaw) * cfg.dt
        cy   += v * math.sin(cyaw) * cfg.dt
        cyaw += omega * cfg.dt
        poses[i] = [cx, cy, cyaw]

    return Trajectory(v=v, omega=omega, poses=poses)


# ---------------------------------------------------------------------------
# Costmap lookup
# ---------------------------------------------------------------------------

def get_clearance(poses: np.ndarray,
                  costmap: np.ndarray,
                  resolution: float,
                  origin_x: float,
                  origin_y: float) -> float:
    """Compute the minimum obstacle clearance along a trajectory.

    Args:
        poses: (T, 3) array of [x, y, yaw].
        costmap: 2-D int32 costmap array (0–100, -1 unknown).
        resolution: Metres per cell.
        origin_x, origin_y: Map origin in metres.

    Returns:
        Minimum clearance in cells (0 = collision, larger = safer).

    TODO:
        - Interpolate between cells for smoother clearance estimates.
    """
    rows, cols = costmap.shape
    min_cost = 0
    for pose in poses:
        col = int((pose[0] - origin_x) / resolution)
        row = int((pose[1] - origin_y) / resolution)
        if 0 <= col < cols and 0 <= row < rows:
            cost = int(costmap[row, col])
            if cost > min_cost:
                min_cost = cost
        else:
            return 0.0  # out-of-bounds = lethal

    # Clearance = 1 - normalised cost
    return 1.0 - min_cost / 100.0


# ---------------------------------------------------------------------------
# DWA local planner
# ---------------------------------------------------------------------------

class DWALocalPlanner:
    """Dynamic window approach local planner.

    Args:
        cfg: DWA configuration.

    TODO:
        - Add oscillation suppression to prevent the robot getting stuck.
        - Implement time-elastic band (TEB) as an alternative local planner.
    """

    def __init__(self, cfg: Optional[DWAConfig] = None) -> None:
        self._cfg = cfg or DWAConfig()

    def compute_velocity_command(
        self,
        pose: Tuple[float, float, float],
        velocity: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: np.ndarray,
        costmap_resolution: float = 0.05,
        costmap_origin: Tuple[float, float] = (-20.0, -20.0),
        global_path: Optional[np.ndarray] = None,
    ) -> Tuple[float, float]:
        """Compute the best (v, ω) command using DWA.

        Args:
            pose: Current robot pose (x, y, yaw).
            velocity: Current robot velocity (v, omega).
            goal: Target position (gx, gy).
            costmap: 2-D costmap array.
            costmap_resolution: Metres per cell.
            costmap_origin: (ox, oy) origin of the costmap.
            global_path: Optional (N, 2) array of global path waypoints for path-following.

        Returns:
            Tuple (v_cmd, omega_cmd) — best velocity command.
        """
        cfg = self._cfg
        x, y, yaw = pose
        v_curr, w_curr = velocity

        # Dynamic window: reachable (v, ω) given current velocity and limits
        v_min = max(cfg.min_v, v_curr - cfg.max_accel)
        v_max = min(cfg.max_v, v_curr + cfg.max_accel)
        w_min = max(-cfg.max_w, w_curr - cfg.max_ang_accel)
        w_max = min( cfg.max_w, w_curr + cfg.max_ang_accel)

        best_score = -float("inf")
        best_v, best_w = 0.0, 0.0

        v_samples = np.linspace(v_min, v_max, cfg.v_resolution)
        w_samples = np.linspace(w_min, w_max, cfg.w_resolution)

        for v in v_samples:
            for w in w_samples:
                traj = simulate_trajectory(x, y, yaw, v, w, cfg)
                clearance = get_clearance(
                    traj.poses, costmap, costmap_resolution,
                    costmap_origin[0], costmap_origin[1]
                )
                if clearance < cfg.min_clearance:
                    continue  # unsafe trajectory

                # Heading score: alignment of trajectory endpoint with goal
                final_pose = traj.poses[-1]
                angle_to_goal = math.atan2(goal[1] - final_pose[1],
                                            goal[0] - final_pose[0])
                heading_score = 1.0 - abs(angle_to_goal - final_pose[2]) / math.pi

                # Velocity score: prefer higher forward speed
                vel_score = v / cfg.max_v if cfg.max_v > 0 else 0.0

                # Path score: distance of trajectory end from global path
                path_score = 0.0
                if global_path is not None and len(global_path) > 0:
                    dists = np.linalg.norm(global_path - final_pose[:2], axis=1)
                    path_score = 1.0 - min(dists) / 5.0  # normalise by 5 m

                score = (cfg.w_heading * heading_score
                         + cfg.w_clearance * clearance
                         + cfg.w_velocity * vel_score
                         + cfg.w_path * path_score)

                if score > best_score:
                    best_score = score
                    best_v, best_w = v, w

        return best_v, best_w


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

if ros2_available:

    class LocalPlannerNode(Node):  # pyright: ignore[reportUntypedBaseClass]
        """ROS2 node wrapping the DWA local planner.

        Subscribes to /costmap, /robot/odom/fused, /robot/global_path.
        Publishes /cmd_vel (geometry_msgs/Twist) at 10 Hz.

        TODO:
            - Wire up actual subscriptions and goal management.
            - Add goal-reached check and publish success status.
        """

        def __init__(self) -> None:
            super().__init__("local_planner")
            self._planner = DWALocalPlanner()
            self._costmap: Optional[np.ndarray] = None
            self._current_pose: Optional[Tuple[float, float, float]] = None
            self._current_vel: Tuple[float, float] = (0.0, 0.0)
            self._goal: Optional[Tuple[float, float]] = None
            self._global_path: Optional[np.ndarray] = None

            self._sub_costmap = self.create_subscription(
                OccupancyGrid, "costmap", self._costmap_callback, 10
            )
            self._sub_odom = self.create_subscription(
                Odometry, "odom/fused", self._odom_callback, 50
            )
            self._sub_path = self.create_subscription(
                Path, "global_path", self._path_callback, 10
            )
            self._pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
            self._timer = self.create_timer(0.1, self._plan_callback)
            self.get_logger().info("LocalPlannerNode initialised.")

        def _costmap_callback(self, msg: OccupancyGrid) -> None:
            rows, cols = msg.info.height, msg.info.width
            self._costmap = np.array(msg.data, dtype=np.int32).reshape(rows, cols)

        def _odom_callback(self, msg: Odometry) -> None:
            p = msg.pose.pose.position
            # TODO: extract yaw from quaternion
            self._current_pose = (p.x, p.y, 0.0)
            t = msg.twist.twist
            self._current_vel = (t.linear.x, t.angular.z)

        def _path_callback(self, msg: Path) -> None:
            if msg.poses:
                self._global_path = np.array(
                    [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
                )

        def _plan_callback(self) -> None:
            """Timer callback: compute and publish velocity command."""
            if (self._costmap is None or self._current_pose is None
                    or self._goal is None):
                return
            v, w = self._planner.compute_velocity_command(
                pose=self._current_pose,
                velocity=self._current_vel,
                goal=self._goal,
                costmap=self._costmap,
                global_path=self._global_path,
            )
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self._pub_cmd.publish(cmd)


def main() -> None:
    if not ros2_available:
        print("rclpy not available.")
        return
    rclpy.init()
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
