"""
lidar_preprocess.py
-------------------
LiDAR point cloud preprocessing node.

Receives raw PointCloud2 messages from the LiDAR driver, applies a sequence of
filters (voxel downsampling → statistical outlier removal → ground-plane
segmentation → normal estimation), and publishes the cleaned cloud on
/robot/scan/filtered.

In a full implementation this module would be wrapped in a ROS2 lifecycle node
so that the filter pipeline can be reconfigured at runtime without restarting.

Dependencies (pip / apt):
    open3d >= 0.17  (or use sensor_msgs_py + numpy for a lighter alternative)
    rclpy
    sensor_msgs
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# ROS2 imports — guarded so the module can be imported outside a ROS2 context
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

# Open3D is used for point cloud processing; fall back gracefully if absent.
try:
    import open3d as o3d
    _O3D_AVAILABLE = True
except ImportError:
    _O3D_AVAILABLE = False


# ---------------------------------------------------------------------------
# Configuration dataclass
# ---------------------------------------------------------------------------

@dataclass
class LidarPreprocessConfig:
    """Parameters for the LiDAR preprocessing pipeline.

    All distances are in metres; thresholds match the defaults in sensors.yaml.
    """
    voxel_leaf_size: float = 0.05
    outlier_k_neighbours: int = 20
    outlier_std_ratio: float = 1.0
    ransac_distance_threshold: float = 0.03
    ransac_max_iterations: int = 100
    range_min: float = 0.3
    range_max: float = 30.0
    estimate_normals: bool = True
    normal_search_radius: float = 0.2


# ---------------------------------------------------------------------------
# Core processing functions
# ---------------------------------------------------------------------------

def load_pointcloud_from_array(xyz: np.ndarray) -> "o3d.geometry.PointCloud":
    """Convert an (N, 3) numpy array to an Open3D point cloud.

    Args:
        xyz: Float array of shape (N, 3) with columns [x, y, z].

    Returns:
        Open3D PointCloud object.

    TODO:
        - Handle intensity and ring-index channels carried in extra columns.
        - Support structured numpy arrays from sensor_msgs_py.read_points().
    """
    if not _O3D_AVAILABLE:
        raise RuntimeError("open3d is not installed; cannot create PointCloud.")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    return pcd


def range_filter(pcd: "o3d.geometry.PointCloud",
                 range_min: float,
                 range_max: float) -> "o3d.geometry.PointCloud":
    """Remove points outside [range_min, range_max] distance from the origin.

    Args:
        pcd: Input point cloud.
        range_min: Minimum valid range in metres.
        range_max: Maximum valid range in metres.

    Returns:
        Filtered point cloud.
    """
    pts = np.asarray(pcd.points)
    dists = np.linalg.norm(pts, axis=1)
    mask = (dists >= range_min) & (dists <= range_max)
    return pcd.select_by_index(np.where(mask)[0])


def voxel_downsample(pcd: "o3d.geometry.PointCloud",
                     leaf_size: float) -> "o3d.geometry.PointCloud":
    """Reduce point density via voxel grid downsampling.

    Args:
        pcd: Input point cloud.
        leaf_size: Voxel edge length in metres.

    Returns:
        Downsampled point cloud.

    TODO:
        - Preserve per-voxel intensity statistics for downstream classification.
    """
    return pcd.voxel_down_sample(voxel_size=leaf_size)


def statistical_outlier_removal(pcd: "o3d.geometry.PointCloud",
                                 k: int = 20,
                                 std_ratio: float = 1.0
                                 ) -> "o3d.geometry.PointCloud":
    """Remove sparse outlier points using a statistical test.

    For each point, computes the mean distance to its k nearest neighbours.
    Points whose mean distance exceeds (global_mean + std_ratio * global_std)
    are removed.

    Args:
        pcd: Input point cloud.
        k: Number of nearest neighbours.
        std_ratio: Multiplier on the standard deviation threshold.

    Returns:
        Inlier point cloud.
    """
    cleaned, _ = pcd.remove_statistical_outlier(nb_neighbors=k, std_ratio=std_ratio)
    return cleaned


def segment_ground_plane(pcd: "o3d.geometry.PointCloud",
                          distance_threshold: float = 0.03,
                          max_iterations: int = 100
                          ) -> tuple["o3d.geometry.PointCloud",
                                     "o3d.geometry.PointCloud"]:
    """Separate ground and non-ground points via RANSAC plane fitting.

    Args:
        pcd: Input point cloud.
        distance_threshold: RANSAC inlier distance in metres.
        max_iterations: Maximum RANSAC iterations.

    Returns:
        Tuple of (ground_cloud, obstacle_cloud).

    TODO:
        - Extend to multi-planar segmentation for ramp / stair detection.
        - Label ground points rather than discarding them.
    """
    plane_model, inlier_idxs = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=max_iterations,
    )
    ground = pcd.select_by_index(inlier_idxs)
    obstacles = pcd.select_by_index(inlier_idxs, invert=True)
    return ground, obstacles


def estimate_normals(pcd: "o3d.geometry.PointCloud",
                     search_radius: float = 0.2) -> "o3d.geometry.PointCloud":
    """Estimate surface normals for each point using a radius-neighbourhood search.

    Args:
        pcd: Input point cloud (will be modified in-place).
        search_radius: Neighbourhood radius in metres.

    Returns:
        Point cloud with normals populated.

    TODO:
        - Orient normals consistently towards the sensor origin.
        - Use KD-tree hybrid search for speed on dense clouds.
    """
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=search_radius, max_nn=30
        )
    )
    return pcd


def preprocess_scan(xyz: np.ndarray,
                    cfg: LidarPreprocessConfig) -> dict:
    """Full preprocessing pipeline for a single LiDAR scan.

    Args:
        xyz: Raw point cloud as (N, 3) float32 array.
        cfg: Preprocessing configuration.

    Returns:
        Dictionary with keys:
            'filtered'   : open3d.PointCloud  — cleaned obstacle points
            'ground'     : open3d.PointCloud  — segmented ground points
            'n_raw'      : int                — input point count
            'n_filtered' : int                — output point count
            'latency_ms' : float              — processing time
    """
    t0 = time.perf_counter()

    pcd = load_pointcloud_from_array(xyz)
    pcd = range_filter(pcd, cfg.range_min, cfg.range_max)
    pcd = voxel_downsample(pcd, cfg.voxel_leaf_size)
    pcd = statistical_outlier_removal(pcd, cfg.outlier_k_neighbours, cfg.outlier_std_ratio)
    ground, obstacles = segment_ground_plane(
        pcd, cfg.ransac_distance_threshold, cfg.ransac_max_iterations
    )
    if cfg.estimate_normals:
        obstacles = estimate_normals(obstacles, cfg.normal_search_radius)

    latency_ms = (time.perf_counter() - t0) * 1000.0
    return {
        "filtered": obstacles,
        "ground": ground,
        "n_raw": len(xyz),
        "n_filtered": len(obstacles.points),
        "latency_ms": latency_ms,
    }


# ---------------------------------------------------------------------------
# ROS2 node wrapper
# ---------------------------------------------------------------------------

if _ROS2_AVAILABLE:

    class LidarPreprocessNode(Node):
        """ROS2 node that wraps the LiDAR preprocessing pipeline.

        Subscribes to /robot/scan/raw (PointCloud2) and publishes to
        /robot/scan/filtered (PointCloud2) at the same rate.

        TODO:
            - Convert PointCloud2 bytes to numpy via sensor_msgs_py.
            - Convert filtered open3d cloud back to PointCloud2 for publishing.
            - Expose pipeline parameters via ROS2 parameter server.
            - Add latency diagnostics publisher.
        """

        def __init__(self, cfg: Optional[LidarPreprocessConfig] = None) -> None:
            super().__init__("lidar_preprocess")
            self._cfg = cfg or LidarPreprocessConfig()

            self._sub = self.create_subscription(
                PointCloud2,
                "scan_raw",
                self._scan_callback,
                qos_profile=10,
            )
            self._pub = self.create_publisher(PointCloud2, "scan_filtered", 10)
            self.get_logger().info("LidarPreprocessNode initialised.")

        def _scan_callback(self, msg: PointCloud2) -> None:
            """Process one incoming scan and publish the filtered result.

            TODO:
                - Deserialise msg to numpy array using sensor_msgs_py.
                - Call preprocess_scan().
                - Serialise open3d cloud back to PointCloud2.
            """
            # Placeholder: just republish as-is until full conversion is implemented.
            self.get_logger().debug(
                f"Received scan with {msg.width * msg.height} points."
            )
            self._pub.publish(msg)


def main() -> None:
    if not _ROS2_AVAILABLE:
        print("rclpy not available — cannot run as a ROS2 node.")
        return
    rclpy.init()
    node = LidarPreprocessNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
