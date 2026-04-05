"""
occupancy_grid_mapper.py
-------------------------
Maintains a 2-D probabilistic occupancy grid and updates it incrementally using
LiDAR range measurements via Bresenham ray-casting.

The grid stores log-odds values for numerical stability.  Each cell can be queried
for its probability of occupancy P(occ | observations).

The mapper also supports:
    - Serialisation to nav_msgs/OccupancyGrid for ROS2 publishing.
    - Incremental diff computation (only cells changed since last transmission).
    - Map merging from multiple robots at the base station.

References:
    Thrun, Burgard, Fox, "Probabilistic Robotics", Chapter 9 (2005).
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false, reportPossiblyUnboundVariable=false, reportUnusedImport=false

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

ros2_available = False
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    ros2_available = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Log-odds constants
# ---------------------------------------------------------------------------

LOG_ODDS_FREE     = -0.85   # sensor model: log P(occ | free ray) / (1 - P)
LOG_ODDS_OCCUPIED =  2.19   # sensor model: log P(occ | hit) / (1 - P)
LOG_ODDS_MIN      = -10.0   # clamping bounds
LOG_ODDS_MAX      =  10.0


def log_odds(p: float) -> float:
    """Convert probability to log-odds."""
    return float(np.log(p / (1.0 - p + 1e-12)))


def probability(l: float | np.ndarray) -> float | np.ndarray:
    """Convert log-odds to probability."""
    return 1.0 / (1.0 + np.exp(-l))


# ---------------------------------------------------------------------------
# Bresenham ray-casting
# ---------------------------------------------------------------------------

def bresenham_line(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """Return all integer grid cells on the line from (x0,y0) to (x1,y1).

    Uses Bresenham's line algorithm.  The endpoint (x1, y1) is excluded
    because it corresponds to the obstacle hit point and is updated separately.

    Args:
        x0, y0: Start cell (robot position).
        x1, y1: End cell (range measurement endpoint).

    Returns:
        List of (col, row) tuples for the free-space cells along the ray.

    TODO:
        - Extend to 3-D voxel grid using 3-D Bresenham (DDA algorithm).
    """
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    cx, cy = x0, y0
    while (cx, cy) != (x1, y1):
        cells.append((cx, cy))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy

    return cells


# ---------------------------------------------------------------------------
# Occupancy grid
# ---------------------------------------------------------------------------

@dataclass
class GridConfig:
    """Configuration for the occupancy grid."""
    width_m: float = 40.0        # physical width in metres
    height_m: float = 40.0       # physical height in metres
    resolution: float = 0.05     # metres per cell
    origin_x: float = -20.0      # map origin (lower-left) in metres
    origin_y: float = -20.0


class OccupancyGridMapper:
    """2-D probabilistic occupancy grid updated via Bresenham ray-casting.

    Args:
        cfg: Grid geometry configuration.

    Attributes:
        log_odds_grid: Float array of shape (rows, cols) storing log-odds values.
    """

    def __init__(self, cfg: Optional[GridConfig] = None) -> None:
        self._cfg = cfg or GridConfig()
        self._cols = int(self._cfg.width_m  / self._cfg.resolution)
        self._rows = int(self._cfg.height_m / self._cfg.resolution)
        self.log_odds_grid = np.zeros((self._rows, self._cols), dtype=np.float32)

        # Track which cells changed since last diff serialisation
        self._dirty_mask = np.zeros((self._rows, self._cols), dtype=bool)
        self._prev_snapshot: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ #
    # Coordinate helpers
    # ------------------------------------------------------------------ #

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world (x, y) in metres to (col, row) grid indices."""
        col = int((x - self._cfg.origin_x) / self._cfg.resolution)
        row = int((y - self._cfg.origin_y) / self._cfg.resolution)
        return col, row

    def cell_to_world(self, col: int, row: int) -> Tuple[float, float]:
        """Convert (col, row) indices to world-frame cell centre coordinates."""
        x = self._cfg.origin_x + (col + 0.5) * self._cfg.resolution
        y = self._cfg.origin_y + (row + 0.5) * self._cfg.resolution
        return x, y

    def _in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self._cols and 0 <= row < self._rows

    # ------------------------------------------------------------------ #
    # Map update
    # ------------------------------------------------------------------ #

    def update_with_scan(self,
                          robot_x: float,
                          robot_y: float,
                          hit_points: np.ndarray) -> int:
        """Update the grid with a 2-D LiDAR scan.

        Args:
            robot_x, robot_y: Robot position in metres.
            hit_points: Array of shape (N, 2) with [x, y] obstacle hit points.

        Returns:
            Number of cells updated.

        TODO:
            - Parallelise ray-casting with numba or C++ extension.
            - Weight updates by measurement uncertainty (range-dependent noise).
        """
        rc = self.world_to_cell(robot_x, robot_y)
        n_updated = 0

        for hit in hit_points:
            hc = self.world_to_cell(float(hit[0]), float(hit[1]))
            if not self._in_bounds(*hc):
                continue

            # Mark free cells along the ray
            for (cc, cr) in bresenham_line(rc[0], rc[1], hc[0], hc[1]):
                if self._in_bounds(cc, cr):
                    self.log_odds_grid[cr, cc] = np.clip(
                        self.log_odds_grid[cr, cc] + LOG_ODDS_FREE,
                        LOG_ODDS_MIN, LOG_ODDS_MAX
                    )
                    self._dirty_mask[cr, cc] = True
                    n_updated += 1

            # Mark obstacle cell
            self.log_odds_grid[hc[1], hc[0]] = np.clip(
                self.log_odds_grid[hc[1], hc[0]] + LOG_ODDS_OCCUPIED,
                LOG_ODDS_MIN, LOG_ODDS_MAX
            )
            self._dirty_mask[hc[1], hc[0]] = True
            n_updated += 1

        return n_updated

    # ------------------------------------------------------------------ #
    # Serialisation
    # ------------------------------------------------------------------ #

    def to_ros_occupancy_grid(self, frame_id: str = "map") -> "OccupancyGrid":
        """Serialise the full grid to a nav_msgs/OccupancyGrid message.

        Cell values: -1 = unknown, 0 = free, 100 = occupied.

        TODO:
            - Set header stamp from ROS clock.
        """
        if not ros2_available:
            raise RuntimeError("ROS2 not available.")
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = self._cfg.resolution
        msg.info.width = self._cols
        msg.info.height = self._rows
        msg.info.origin.position.x = self._cfg.origin_x
        msg.info.origin.position.y = self._cfg.origin_y

        prob = np.asarray(probability(self.log_odds_grid), dtype=np.float32)  # (rows, cols) in [0, 1]
        # Map to ROS convention: -1 unknown, 0–100 probability × 100
        ros_data = np.where(
            np.abs(self.log_odds_grid) < 0.01,
            -1,
            np.clip((prob * 100).astype(np.int8), 0, 100),
        ).astype(np.int8).flatten().tolist()
        msg.data = ros_data
        return msg

    def get_diff(self) -> dict[str, object]:
        """Return cells changed since the last call to get_diff().

        Returns:
            Dictionary with:
                'indices': list of (col, row) tuples of changed cells
                'values' : corresponding int8 occupancy values (0/100/-1)
                'n_cells': number of changed cells

        TODO:
            - Encode as run-length compressed bytes for efficient transmission.
        """
        dirty_coords = list(zip(*np.where(self._dirty_mask)))
        values = []
        for (row, col) in dirty_coords:
            lo = self.log_odds_grid[row, col]
            if abs(lo) < 0.01:
                values.append(-1)
            else:
                values.append(int(np.clip(probability(lo) * 100, 0, 100)))

        self._dirty_mask[:] = False  # reset after diff
        return {
            "indices": [(col, row) for (row, col) in dirty_coords],
            "values": values,
            "n_cells": len(values),
        }

    # ------------------------------------------------------------------ #
    # Map merging (base station)
    # ------------------------------------------------------------------ #

    @staticmethod
    def merge_grids(grids: List["OccupancyGridMapper"]) -> "OccupancyGridMapper":
        """Merge multiple robots' grids into a single global map.

        Uses simple log-odds addition; assumes all grids share the same geometry.

        TODO:
            - Handle grids with different origins via coordinate transform.
            - Weight contributions by estimated sensor reliability.
        """
        assert grids, "Need at least one grid to merge."
        merged = OccupancyGridMapper(grids[0]._cfg)
        for g in grids:
            merged.log_odds_grid += g.log_odds_grid
        np.clip(merged.log_odds_grid, LOG_ODDS_MIN, LOG_ODDS_MAX, out=merged.log_odds_grid)
        return merged


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

if ros2_available:

    class OccupancyGridMapperNode(Node):  # pyright: ignore[reportUntypedBaseClass]
        """ROS2 node wrapping OccupancyGridMapper.

        TODO:
            - Subscribe to /robot/scan/filtered and /robot/odom/fused.
            - Project 3-D scan to 2-D hit points (slice at robot height ± 0.3 m).
            - Call update_with_scan() and publish /map and /map/diff periodically.
        """

        def __init__(self) -> None:
            super().__init__("occupancy_grid_mapper")
            self._mapper = OccupancyGridMapper()
            self._pub_map = self.create_publisher(OccupancyGrid, "map", 10)
            self._pub_diff = self.create_publisher(OccupancyGrid, "map/diff", 10)
            self._pub_timer = self.create_timer(0.5, self._publish_map)
            self.get_logger().info("OccupancyGridMapperNode initialised.")

        def _publish_map(self) -> None:
            """Publish the full map at a fixed rate.

            TODO: also publish diff for comms edge client.
            """
            msg = self._mapper.to_ros_occupancy_grid()
            self._pub_map.publish(msg)


def main() -> None:
    if not ros2_available:
        print("rclpy not available.")
        return
    rclpy.init()
    node = OccupancyGridMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
