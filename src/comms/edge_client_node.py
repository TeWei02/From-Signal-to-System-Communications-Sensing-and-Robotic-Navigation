"""
edge_client_node.py
-------------------
Robot-side ROS2 node that manages uplink data transmission and downlink planning
reception from the edge server / base station.

Subscriptions:
  /robot/map (nav_msgs/OccupancyGrid)      — local occupancy grid from mapper
  /robot/pose (geometry_msgs/PoseStamped)  — current robot pose
  /robot/sensor_status (sensor_msgs/...)   — diagnostics info

Publications:
  /robot/map/diff (custom_msgs/OccupancyGridDiff) — sparse map updates
  /robot/telemetry (...)                   — compressed sensor summary
  /robot/waypoint_ref (geometry_msgs/Path) — reference path from edge

The node:
  1. Subscribes to local mapping and sensor outputs
  2. Compresses and applies QoS profile (bandwidth-limited transmission)
  3. Uses link_simulator to schedule packets
  4. Acknowledges receipt of planning updates from edge
  5. Logs all comm events for post-hoc analysis

This is a skeleton; full implementation would include:
  - Configurable compression codecs
  - Adaptive QoS re-negotiation
  - Reliable transport (MQTT, DDS) or ROS topic logging
  - Graceful degradation under poor link conditions
"""

# pyright: reportMissingImports=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

# Pseudo-ROS2 skeleton
class NodeBase:
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        pass

    def create_subscription(self, *args: Any, **kwargs: Any) -> Any:
        return None

    def create_publisher(self, *args: Any, **kwargs: Any) -> Any:
        return None

    class _Logger:
        def info(self, *args: Any, **kwargs: Any) -> None:
            pass

        def debug(self, *args: Any, **kwargs: Any) -> None:
            pass

    def get_logger(self) -> "NodeBase._Logger":
        return NodeBase._Logger()


class _GridInfo:
    width: int = 0
    height: int = 0


class OccupancyGrid:
    info: _GridInfo = _GridInfo()


class PoseStamped:
    pass


class Path:
    pass


rclpy = None
ros2_available = False

try:
    import rclpy as _rclpy

    rclpy = _rclpy
    ros2_available = True
except ImportError:
    pass

from comms.link_simulator import WirelessLink
from comms.qos_models import AdaptiveQoSManager


@dataclass
class CommEvent:
    """Log entry for communication events."""
    timestamp: float
    direction: str           # "upload" or "download"
    msg_type: str           # "occupancy_grid", "waypoint_path", etc.
    bytes_transmitted: int
    latency_ms: float
    success: bool
    msg_id: str = ""


class EdgeClientNode(NodeBase):
    """
    Robot-side communication client.
    
    Manages:
      - Rate-limited data uplink (perception → edge)
      - Downlink reception (edge planning → local execution)
      - QoS adaptation based on link conditions
    """
    
    def __init__(self):
        super().__init__("edge_client_node")
        
        # TODO: Load comm config from yaml
        self.link = WirelessLink(bandwidth_mbps=2.0, latency_ms=100, packet_loss_rate=0.01)
        self.qos_mgr = AdaptiveQoSManager()
        
        # Subscription to local map
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            "/robot/map",
            self.map_callback,
            qos_profile_name="mapping_medium"  # TODO: Proper QoS handling
        )
        
        # Subscription to pose
        self.sub_pose = self.create_subscription(
            PoseStamped,
            "/robot/pose",
            self.pose_callback,
            10
        )

        # Publication of map diffs
        self.pub_map_diff = self.create_publisher(OccupancyGrid, "/robot/map/diff", 10)
        
        # Publication of waypoint references
        self.pub_waypoints = self.create_publisher(Path, "/robot/waypoint_ref", 10)
        
        # TODO: Timer to process uplink queue
        # TODO: Timer to check for downlink messages
        
        self.comm_log: list[CommEvent] = []
        self.last_map_sequence = 0
        
        self.get_logger().info("EdgeClientNode initialized")
    
    def map_callback(self, msg: OccupancyGrid):
        """
        On map update, compress and attempt to transmit diff to edge.
        
        TODO: Implement occupancy grid diffing (only changed cells)
        """
        # TODO: Compare msg against last sent map
        # TODO: Extract cells that changed
        # TODO: Compress diff
        
        payload_bytes = self.estimate_diff_size(msg)
        success, delay_ms = self.link.transmit(
            payload_bytes=payload_bytes,
            priority=1,
            msg_id=f"map_diff_{self.last_map_sequence}",
            deadline_ms=1000
        )
        
        self.last_map_sequence += 1
        
        event = CommEvent(
            timestamp=time.time(),
            direction="upload",
            msg_type="occupancy_grid_diff",
            bytes_transmitted=payload_bytes if success else 0,
            latency_ms=delay_ms,
            success=success,
            msg_id=f"map_diff_{self.last_map_sequence - 1}"
        )
        self.comm_log.append(event)
        
        self.get_logger().debug(f"Map diff upload: success={success}, delay={delay_ms:.1f}ms")
    
    def pose_callback(self, msg: PoseStamped):
        """
        Optionally send pose updates (e.g., for edge-side localization/tracking).
        
        TODO: Selective transmission based on motion thresholds
        """
        # Stub: only log
        pass
    
    def estimate_diff_size(self, msg: OccupancyGrid) -> int:
        """
        Estimate the size of a compressed occupancy grid diff.
        
        Simple approximation: full grid is ~n_cells bytes; diff is typically 5-20% of that.
        """
        grid_size = msg.info.width * msg.info.height
        
        # Assume 10% of cells changed, plus 100 bytes overhead
        diff_size = int(grid_size * 0.1) + 100
        return diff_size
    
    def process_downlink(self):
        """
        Check for planning updates from edge server.
        
        TODO: Listen for waypoint paths, velocity constraints, etc.
        TODO: Publish to /robot/waypoint_ref for local planner
        """
        # TODO: Receive and deserialize messages from edge
        # TODO: Publish waypoints
        pass
    
    def get_comm_log(self) -> list[CommEvent]:
        """Return communication event log for analysis."""
        return self.comm_log
    
    def get_link_stats(self):
        """Return link statistics."""
        return self.link.get_stats()


def main(args: list[str] | None = None) -> None:
    """Main entry point for ROS2 node."""
    if not ros2_available or rclpy is None:
        print("EdgeClientNode requires rclpy. Run within ROS2 environment.")
        return

    try:
        rclpy.init(args=args)
        node = EdgeClientNode()
        rclpy.spin(node)
    except NameError:
        print("EdgeClientNode requires rclpy. Run within ROS2 environment.")
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
