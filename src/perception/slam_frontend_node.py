"""
slam_frontend_node.py
---------------------
ROS2 SLAM front-end node.

Subscribes to the filtered LiDAR scan, fused odometry, and optionally IMU data.
Performs iterative scan-matching (ICP / GICP stub) to estimate the incremental
robot motion, maintains a local submap, and publishes:

  /robot/slam/pose        geometry_msgs/PoseWithCovarianceStamped
  /robot/slam/submap      sensor_msgs/PointCloud2
  /robot/map/diff         nav_msgs/OccupancyGrid   (incremental diff for transmission)
  /tf                     base_link → odom → map transforms

Loop closure and global map optimisation are delegated to a separate back-end
node (not implemented here) that subscribes to /robot/slam/submap.

References:
    Koide et al., "A Portable 3D LIDAR-based System for Long-term and Wide-area
    People Behavior Measurement", Advanced Robotics, 2019.  (GICP-based front-end)
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false, reportPossiblyUnboundVariable=false, reportUnusedImport=false

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, List

import numpy as np

ros2_available = False
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    import message_filters
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from tf2_ros import TransformBroadcaster
    ros2_available = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Scan-matching stub
# ---------------------------------------------------------------------------

@dataclass
class ScanMatchResult:
    """Result of one scan-matching iteration."""
    transform: np.ndarray = field(default_factory=lambda: np.eye(4))  # 4×4 SE(3)
    fitness: float = 0.0       # fraction of inlier correspondences
    rmse: float = float("inf") # root mean square point-to-point error
    converged: bool = False
    n_iterations: int = 0


def icp_scan_match(source: np.ndarray,
                   target: np.ndarray,
                   initial_guess: np.ndarray,
                   max_correspondence_dist: float = 0.3,
                   max_iterations: int = 50,
                   convergence_threshold: float = 1e-4) -> ScanMatchResult:
    """Iterative Closest Point scan matching stub.

    Args:
        source: Source point cloud (N, 3).
        target: Target point cloud (M, 3).
        initial_guess: Initial SE(3) transform estimate (4×4).
        max_correspondence_dist: Maximum point correspondence distance.
        max_iterations: ICP iteration limit.
        convergence_threshold: Convergence criterion on transform delta.

    Returns:
        ScanMatchResult with the estimated transform.

    TODO:
        - Implement point-to-plane ICP for better accuracy on planar surfaces.
        - Replace with GICP (Generalized ICP) using pre-computed covariances.
        - Add robust kernel (Huber) to handle dynamic object outliers.
        - Vectorise with numpy or delegate to open3d.registration.
    """
    # Stub: return identity transform with a plausible fitness score
    result = ScanMatchResult()
    result.transform = initial_guess.copy()
    result.converged = True
    result.fitness = 0.85
    result.rmse = 0.02
    result.n_iterations = 15
    return result


# ---------------------------------------------------------------------------
# Submap management
# ---------------------------------------------------------------------------

class Submap:
    """Accumulates a fixed number of aligned scans into a local point cloud.

    Args:
        capacity: Number of scans to accumulate before the submap is finalised.

    TODO:
        - Implement voxel-based submap downsampling to bound memory.
        - Publish finished submaps to the SLAM back-end for loop closure.
    """

    def __init__(self, capacity: int = 100) -> None:
        self._capacity = capacity
        self._points: List[np.ndarray] = []
        self._pose_history: List[np.ndarray] = []

    @property
    def is_full(self) -> bool:
        return len(self._points) >= self._capacity

    def add_scan(self, scan: np.ndarray, pose: np.ndarray) -> None:
        """Append a scan (in map frame) to the submap."""
        self._points.append(scan)
        self._pose_history.append(pose)

    def get_pointcloud(self) -> np.ndarray:
        """Merge all accumulated scans into one array.

        Returns:
            Concatenated point cloud (N_total, 3) or empty (0, 3) array.

        TODO:
            - Apply downsampling before returning.
        """
        if not self._points:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(self._points)

    def reset(self) -> None:
        self._points.clear()
        self._pose_history.clear()


# ---------------------------------------------------------------------------
# ROS2 SLAM front-end node
# ---------------------------------------------------------------------------

if ros2_available:

    class SlamFrontendNode(Node):  # pyright: ignore[reportUntypedBaseClass]
        """SLAM front-end: scan-matching, submap management, pose publishing.

        Parameters (ROS2 param server):
            scan_matching_method : str   — 'icp' or 'gicp' (default: 'gicp')
            submap_size          : int   — scans per submap (default: 100)
            map_frame            : str   — 'map'
            odom_frame           : str   — 'odom'
            base_frame           : str   — 'base_link'

        TODO:
            - Implement map diff serialisation for the comms edge client.
            - Add /save_map service that dumps the accumulated PCD to disk.
            - Integrate loop closure signal from SLAM back-end.
        """

        def __init__(self) -> None:
            super().__init__("slam_frontend")

            # Parameters
            self.declare_parameter("scan_matching_method", "gicp")
            self.declare_parameter("submap_size", 100)
            self.declare_parameter("map_frame", "map")
            self.declare_parameter("odom_frame", "odom")
            self.declare_parameter("base_frame", "base_link")

            submap_size = self.get_parameter("submap_size").value
            self._submap = Submap(capacity=submap_size)

            # Current pose estimate (4×4 SE3 in map frame)
            self._T_map_base = np.eye(4)
            self._T_odom_base_prev = np.eye(4)
            self._prev_scan: Optional[np.ndarray] = None

            self._tf_broadcaster = TransformBroadcaster(self)

            # Subscribers — approximate time sync on scan + odom
            scan_sub = message_filters.Subscriber(self, PointCloud2, "scan/filtered")
            odom_sub = message_filters.Subscriber(self, Odometry, "odom/fused")
            self._sync = message_filters.ApproximateTimeSynchronizer(
                [scan_sub, odom_sub], queue_size=10, slop=0.05
            )
            self._sync.registerCallback(self._sync_callback)

            # Publishers
            self._pub_pose = self.create_publisher(
                PoseWithCovarianceStamped, "slam/pose", 10
            )
            self._pub_submap = self.create_publisher(PointCloud2, "slam/submap", 10)
            self._pub_map_diff = self.create_publisher(OccupancyGrid, "map/diff", 10)

            self.get_logger().info("SlamFrontendNode initialised.")

        def _sync_callback(self, scan_msg: PointCloud2,
                           odom_msg: Odometry) -> None:
            """Process one synchronised (scan, odometry) pair.

            TODO:
                1. Convert scan_msg to numpy (N, 3) array.
                2. Extract odometry delta as SE(3) initial guess.
                3. Call icp_scan_match() with previous scan as target.
                4. Update self._T_map_base.
                5. Add scan to self._submap.
                6. If submap full, publish submap and reset.
                7. Publish updated pose.
                8. Broadcast TF: map → odom → base_link.
                9. Compute and publish occupancy grid diff.
            """
            now = self.get_clock().now()
            self.get_logger().debug(
                f"Sync callback at t={now.nanoseconds * 1e-9:.3f}",
                throttle_duration_sec=1.0,
            )

        def _broadcast_tf(self, stamp: "Time") -> None:
            """Broadcast the map→odom transform from the current pose estimate.

            TODO: compute and publish the correct TF from self._T_map_base.
            """
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp.to_msg()
            tf_msg.header.frame_id = self.get_parameter("map_frame").value
            tf_msg.child_frame_id = self.get_parameter("odom_frame").value
            self._tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    if not ros2_available:
        print("rclpy not available.")
        return
    rclpy.init()
    node = SlamFrontendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
