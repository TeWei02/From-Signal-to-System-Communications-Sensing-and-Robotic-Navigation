"""
costmap_generator.py
---------------------
Generates a 2-D costmap from the occupancy grid for use by the local and global
planners.  Applies obstacle inflation (binary dilation), adds a distance-transform
cost layer, and optionally fuses semantic layers (e.g., floor type, restricted zones).

The costmap is updated whenever the underlying occupancy grid changes and published
at up to 2 Hz.

TODO:
    - Implement rolling window costmap that tracks the robot's local neighbourhood.
    - Add a semantic layer that marks restricted zones from a mission config.
    - Support 3-D costmap slices for multi-floor environments.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class CostmapConfig:
    """Parameters for costmap generation."""
    inflation_radius_m: float = 0.25      # robot footprint + safety margin
    resolution: float = 0.05             # must match occupancy grid
    lethal_cost: int = 100               # cost value for occupied + inflated cells
    inscribed_cost: int = 99             # cost within robot footprint radius
    decay_exponent: float = 2.5          # exponential cost decay outside inflation
    unknown_cost: int = -1               # cost for unknown cells


# ---------------------------------------------------------------------------
# Costmap generation
# ---------------------------------------------------------------------------

def generate_costmap(occ_grid: np.ndarray,
                     cfg: Optional[CostmapConfig] = None) -> np.ndarray:
    """Convert an occupancy grid to a costmap with obstacle inflation.

    Args:
        occ_grid: 2-D int8 array (values: -1 unknown, 0 free, 100 occupied).
        cfg: Costmap generation parameters.

    Returns:
        2-D int32 costmap array of the same shape.

    TODO:
        - Use a proper distance-function-based inflation (like Nav2's inflation layer).
        - Handle unknown cells (−1) separately: propagate cost from occupied into unknown.
    """
    cfg = cfg or CostmapConfig()
    rows, cols = occ_grid.shape
    costmap = np.zeros((rows, cols), dtype=np.int32)

    # Binary obstacle mask
    obstacle_mask = (occ_grid == 100)
    unknown_mask  = (occ_grid == -1)

    # Compute distance transform (distance of each free cell to nearest obstacle)
    # distance_transform_edt returns distances in pixels; scale by resolution.
    dist_pixels = distance_transform_edt(~obstacle_mask)
    dist_metres = dist_pixels * cfg.resolution

    inflation_pixels = cfg.inflation_radius_m / cfg.resolution

    # Assign costs
    # Lethal zone: occupied cell or within half a resolution of it
    costmap[obstacle_mask] = cfg.lethal_cost

    # Inflation zone: smooth cost decay from inscribed_cost to 0
    inflation_zone = (dist_metres > 0) & (dist_metres <= cfg.inflation_radius_m)
    decay = np.exp(
        -cfg.decay_exponent * (dist_metres[inflation_zone] / cfg.inflation_radius_m - 0.0)
    )
    costmap[inflation_zone] = np.clip(
        (decay * cfg.inscribed_cost).astype(np.int32), 1, cfg.inscribed_cost
    )

    # Unknown cells
    costmap[unknown_mask] = cfg.unknown_cost

    return costmap


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

if _ROS2_AVAILABLE:

    class CostmapGeneratorNode(Node):
        """ROS2 node that generates a costmap from the occupancy grid.

        Subscribes to /map (nav_msgs/OccupancyGrid) and publishes /costmap
        (also as nav_msgs/OccupancyGrid for compatibility with Nav2).

        TODO:
            - Subscribe to /map_updates for incremental updates instead of full map.
            - Add service to update inflation radius at runtime.
            - Publish static and dynamic costmap layers separately.
        """

        def __init__(self) -> None:
            super().__init__("costmap_generator")
            self._cfg = CostmapConfig()

            self._sub = self.create_subscription(
                OccupancyGrid, "map", self._map_callback, 10
            )
            self._pub = self.create_publisher(OccupancyGrid, "costmap", 10)
            self.get_logger().info("CostmapGeneratorNode initialised.")

        def _map_callback(self, msg: OccupancyGrid) -> None:
            """Recompute and publish the costmap whenever the map changes.

            TODO:
                - Convert OccupancyGrid data to numpy.
                - Call generate_costmap().
                - Wrap result in OccupancyGrid and publish.
            """
            rows = msg.info.height
            cols = msg.info.width
            occ = np.array(msg.data, dtype=np.int8).reshape(rows, cols)
            costmap_data = generate_costmap(occ, self._cfg)

            out_msg = OccupancyGrid()
            out_msg.header = msg.header
            out_msg.info = msg.info
            out_msg.data = costmap_data.astype(np.int8).flatten().tolist()
            self._pub.publish(out_msg)


def main() -> None:
    if not _ROS2_AVAILABLE:
        print("rclpy not available.")
        return
    rclpy.init()
    node = CostmapGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
