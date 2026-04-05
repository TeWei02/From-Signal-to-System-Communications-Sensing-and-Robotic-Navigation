"""
global_planner_stub.py
-----------------------
Global path planner using A* on the 2-D costmap.

Finds a collision-free path from the current robot pose to a goal pose on the
inflated costmap.  The path is returned as a sequence of (x, y) waypoints in
the map frame and published as a nav_msgs/Path.

TODO:
    - Add Theta* (any-angle A*) to reduce the staircase effect on diagonal paths.
    - Support RRT* as an alternative for high-dimensional spaces.
    - Cache last plan and only replan when costmap changes significantly.
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false, reportPossiblyUnboundVariable=false, reportUnusedImport=false

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

ros2_available = False
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, Path
    from geometry_msgs.msg import PoseStamped
    ros2_available = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# A* implementation
# ---------------------------------------------------------------------------

def astar(grid: np.ndarray,
          start: Tuple[int, int],
          goal: Tuple[int, int],
          lethal_threshold: int = 90) -> Optional[List[Tuple[int, int]]]:
    """A* path search on a 2-D costmap grid.

    Args:
        grid: 2-D int32 costmap (0 = free, 100 = lethal, -1 = unknown).
        start: (col, row) start cell.
        goal: (col, row) goal cell.
        lethal_threshold: Cells with cost ≥ this value are impassable.

    Returns:
        List of (col, row) cells from start to goal, or None if no path found.

    TODO:
        - Use a bi-directional A* for faster search on large maps.
        - Replace 8-connected grid with a continuous-space RRT for smoother paths.
    """
    rows, cols = grid.shape

    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def neighbours(cell: Tuple[int, int]):
        c, r = cell
        for dc, dr in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cols and 0 <= nr < rows:
                cost = int(grid[nr, nc])
                if cost < lethal_threshold and cost >= 0:
                    step_cost = math.hypot(dc, dr) * (1.0 + cost / 100.0)
                    yield (nc, nr), step_cost

    open_heap: List[Tuple[float, Tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start))
    came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == goal:
            # Reconstruct path
            path = []
            node = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            return list(reversed(path))

        for neighbour, step_cost in neighbours(current):
            tentative_g = g_score[current] + step_cost
            if tentative_g < g_score.get(neighbour, float("inf")):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f = tentative_g + heuristic(neighbour, goal)
                heapq.heappush(open_heap, (f, neighbour))

    return None  # no path found


def cells_to_world_path(cells: List[Tuple[int, int]],
                         resolution: float,
                         origin_x: float,
                         origin_y: float) -> np.ndarray:
    """Convert cell path to world-frame (x, y) waypoints.

    Args:
        cells: List of (col, row) cell indices.
        resolution: Metres per cell.
        origin_x, origin_y: Map origin.

    Returns:
        (N, 2) float array of world-frame waypoints.
    """
    waypoints = []
    for col, row in cells:
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        waypoints.append([x, y])
    return np.array(waypoints) if waypoints else np.empty((0, 2))


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

if ros2_available:

    class GlobalPlannerNode(Node):  # pyright: ignore[reportUntypedBaseClass]
        """ROS2 node exposing A* global planning as a service and topic.

        Subscribes to /costmap (OccupancyGrid) and /robot/goal (PoseStamped).
        Publishes /robot/global_path (Path) whenever a new goal is received or
        the costmap changes significantly.

        TODO:
            - Implement ComputePathToPose action server (Nav2-compatible).
            - Add path smoothing (e.g., Bezier spline through waypoints).
        """

        def __init__(self) -> None:
            super().__init__("global_planner")
            self._costmap: Optional[np.ndarray] = None
            self._costmap_info: Optional[dict[str, float]] = None
            self._current_pose: Optional[Tuple[float, float]] = None
            self._goal: Optional[Tuple[float, float]] = None

            self._sub_costmap = self.create_subscription(
                OccupancyGrid, "costmap", self._costmap_callback, 10
            )
            self._sub_goal = self.create_subscription(
                PoseStamped, "goal", self._goal_callback, 10
            )
            self._pub_path = self.create_publisher(Path, "global_path", 10)
            self.get_logger().info("GlobalPlannerNode initialised.")

        def _costmap_callback(self, msg: OccupancyGrid) -> None:
            rows, cols = msg.info.height, msg.info.width
            self._costmap = np.array(msg.data, dtype=np.int32).reshape(rows, cols)
            self._costmap_info = {
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
            }
            self._try_plan()

        def _goal_callback(self, msg: PoseStamped) -> None:
            self._goal = (msg.pose.position.x, msg.pose.position.y)
            self._try_plan()

        def _try_plan(self) -> None:
            """Attempt to plan a path and publish it."""
            if (self._costmap is None or self._goal is None
                    or self._current_pose is None or self._costmap_info is None):
                return

            info = self._costmap_info
            res = info["resolution"]
            ox, oy = info["origin_x"], info["origin_y"]

            start_cell = (
                int((self._current_pose[0] - ox) / res),
                int((self._current_pose[1] - oy) / res),
            )
            goal_cell = (
                int((self._goal[0] - ox) / res),
                int((self._goal[1] - oy) / res),
            )

            cell_path = astar(self._costmap, start_cell, goal_cell)
            if cell_path is None:
                self.get_logger().warn("A*: no path found to goal.")
                return

            waypoints = cells_to_world_path(cell_path, res, ox, oy)
            path_msg = Path()
            path_msg.header.frame_id = "map"
            for wp in waypoints:
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.pose.position.x = float(wp[0])
                ps.pose.position.y = float(wp[1])
                path_msg.poses.append(ps)

            self._pub_path.publish(path_msg)
            self.get_logger().info(
                f"Published global path with {len(waypoints)} waypoints."
            )


def main() -> None:
    if not ros2_available:
        print("rclpy not available.")
        return
    rclpy.init()
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
