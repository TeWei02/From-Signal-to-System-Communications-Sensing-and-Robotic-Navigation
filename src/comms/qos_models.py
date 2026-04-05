"""
qos_models.py
-------------
Quality-of-Service (QoS) profile definitions for different communication use cases.

Profiles define:
  - Bandwidth allocation
  - Latency budget
  - Packet loss tolerance
  - Deadline constraints
  - Compression strategy

Example profiles:
  - HighBandwidth_LowLatency: Real-time control (sensor -> planning)
  - LowBandwidth_HighLatency: Batch map uploads
  - Adaptive: Adjust based on available bandwidth
"""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


class CompressionStrategy(Enum):
    """Perception data compression approaches."""
    NONE = "raw"                  # Full-resolution LiDAR / camera
    LOSSY_LIGHT = "lossy_light"   # Voxel downsampling, dropped rays
    LOSSY_HEAVY = "lossy_heavy"   # Aggressive downsampling, keyframes only
    GEOMETRY_SPARSE = "sparse"    # Sparse occupancy diff (changed cells only)
    LEARNED = "learned"           # Learned end-to-end codec (stub)


@dataclass
class QoSProfile:
    """Describes a communication QoS profile."""
    
    name: str                              # Profile identifier (e.g., "high_bw_low_lat")
    bandwidth_mbps: float                  # Allocated bandwidth in Mbps
    latency_budget_ms: float               # Maximum acceptable one-way latency
    packet_loss_tolerance: float = 0.01    # Max acceptable loss rate
    compression: CompressionStrategy = CompressionStrategy.NONE
    
    # Data rate constraints
    max_payload_bytes: int = 65536
    priority: int = 0                      # 0 = highest, larger = lower
    
    # Metadata
    description: str = ""
    use_case: str = ""  # e.g., "control", "mapping", "planning"
    
    @classmethod
    def create_ctrl_high_bw(cls) -> QoSProfile:
        """
        High-bandwidth, low-latency for real-time control.
        E.g., sensor fusion + local planning running on-board or low-latency edge.
        """
        return cls(
            name="control_high_bw",
            bandwidth_mbps=10.0,
            latency_budget_ms=50.0,
            packet_loss_tolerance=0.001,
            compression=CompressionStrategy.NONE,
            priority=0,
            use_case="control",
            description="Real-time control: full-rate sensor data, low latency"
        )
    
    @classmethod
    def create_mapping_medium(cls) -> QoSProfile:
        """
        Medium bandwidth for incremental map updates.
        Sparse differential occupancy grid (only changed cells).
        """
        return cls(
            name="mapping_medium",
            bandwidth_mbps=2.0,
            latency_budget_ms=500.0,
            packet_loss_tolerance=0.05,
            compression=CompressionStrategy.GEOMETRY_SPARSE,
            priority=1,
            use_case="mapping",
            description="Incremental mapping: sparse occupancy diffs, medium QoS"
        )
    
    @classmethod
    def create_fullmap_low_bw(cls) -> QoSProfile:
        """
        Low bandwidth for full-map upload (e.g., at mission end or periodically).
        Aggressive lossy compression + best-effort delivery.
        """
        return cls(
            name="fullmap_low_bw",
            bandwidth_mbps=0.5,
            latency_budget_ms=5000.0,
            packet_loss_tolerance=0.1,
            compression=CompressionStrategy.LOSSY_HEAVY,
            priority=3,
            use_case="mapping",
            description="Full map periodic upload: efficient lossy compression"
        )
    
    @classmethod
    def create_planning_teleop_medium(cls) -> QoSProfile:
        """
        Medium bandwidth for planning updates from edge + teleoperation input.
        """
        return cls(
            name="planning_teleop",
            bandwidth_mbps=1.0,
            latency_budget_ms=200.0,
            packet_loss_tolerance=0.02,
            compression=CompressionStrategy.LOSSY_LIGHT,
            priority=1,
            use_case="planning",
            description="Planning: edge waypoints, teleoperation, moderate latency tolerance"
        )
    
    @classmethod
    def create_best_effort_logging(cls) -> QoSProfile:
        """
        Ultra-low priority for best-effort logging / diagnostics.
        Sent only if bandwidth permits.
        """
        return cls(
            name="logging_best_effort",
            bandwidth_mbps=0.1,
            latency_budget_ms=30000.0,
            packet_loss_tolerance=0.5,
            compression=CompressionStrategy.LOSSY_HEAVY,
            priority=10,
            use_case="logging",
            description="Best-effort logging: highest loss tolerance, lowest priority"
        )


class AdaptiveQoSManager:
    """
    Dynamically adjusts QoS profiles based on observed link conditions.
    
    Monitors:
      - Actual bandwidth availability
      - Measured latency
      - Packet loss rate
    
    Adjusts compression level and/or data rate to stay within constraints.
    """
    
    def __init__(self):
        self.profiles: Dict[str, QoSProfile] = {}
        self._register_default_profiles()
    
    def _register_default_profiles(self):
        """Register built-in QoS profiles."""
        profiles = [
            QoSProfile.create_ctrl_high_bw(),
            QoSProfile.create_mapping_medium(),
            QoSProfile.create_fullmap_low_bw(),
            QoSProfile.create_planning_teleop_medium(),
            QoSProfile.create_best_effort_logging(),
        ]
        for p in profiles:
            self.profiles[p.name] = p
    
    def get_profile(self, name: str) -> Optional[QoSProfile]:
        """Retrieve a QoS profile by name."""
        return self.profiles.get(name)
    
    def register_profile(self, profile: QoSProfile):
        """Register a custom QoS profile."""
        self.profiles[profile.name] = profile
    
    def adapt_for_bandwidth(self, available_mbps: float, base_profile_name: str) -> QoSProfile:
        """
        Adapt a base QoS profile to the available bandwidth.
        
        If available bandwidth is low, escalate compression or reduce data rate.
        
        Args:
            available_mbps: Measured available bandwidth
            base_profile_name: Name of the base profile to adapt from
            
        Returns:
            An adapted QoS profile
        """
        base = self.get_profile(base_profile_name)
        if not base:
            raise ValueError(f"Profile '{base_profile_name}' not found")
        
        # TODO: Interpolate to an appropriate profile or modify base profile
        # For now, return base as-is (stub)
        
        if available_mbps < base.bandwidth_mbps * 0.5:
            # Bandwidth has dropped significantly; escalate compression
            adapted = QoSProfile(
                name=f"{base.name}_adapted_compressed",
                bandwidth_mbps=available_mbps,
                latency_budget_ms=base.latency_budget_ms * 1.5,
                packet_loss_tolerance=base.packet_loss_tolerance * 1.5,
                compression=CompressionStrategy.LOSSY_HEAVY,
                description=f"Adapted (available BW: {available_mbps:.2f} Mbps)"
            )
            return adapted
        
        return base
    
    def list_profiles(self):
        """List all registered profiles."""
        return [(name, p.description) for name, p in self.profiles.items()]


if __name__ == "__main__":
    mgr = AdaptiveQoSManager()
    print("Available QoS Profiles:")
    for name, desc in mgr.list_profiles():
        print(f"  {name}: {desc}")
    
    # Example: get a profile and check its parameters
    p = mgr.get_profile("control_high_bw")
    assert p is not None, "Profile 'control_high_bw' should exist"
    print(f"\n{p.name}:")
    print(f"  Bandwidth: {p.bandwidth_mbps} Mbps")
    print(f"  Latency budget: {p.latency_budget_ms} ms")
    print(f"  Compression: {p.compression.value}")
