# Communications Architecture

## Overview

The communications layer is the backbone that connects onboard robot processing to the
base station and, optionally, to an edge-cloud inference server.  Unlike a wired system,
the wireless link introduces non-deterministic latency, variable bandwidth, and packet
loss.  This document describes how the system models those effects, which topics cross
the wireless boundary, and how QoS profiles ensure that critical messages are prioritized.

## Wireless Link Model

The link simulator (`src/comms/link_simulator.py`) models a single-hop wireless channel
with the following parameters:

| Parameter | Symbol | Typical Range | Config Key |
|-----------|--------|---------------|-----------|
| Channel bandwidth | B | 100 kbps – 10 Mbps | `comms.bandwidth_bps` |
| One-way propagation delay | τ | 5 – 100 ms | `comms.base_latency_ms` |
| Delay jitter (std dev) | σ_τ | 1 – 20 ms | `comms.jitter_ms` |
| Packet loss probability | p_loss | 0 – 0.15 | `comms.packet_loss_rate` |
| Max transmission unit | MTU | 1400 bytes | `comms.mtu_bytes` |

The simulator queues outgoing messages, applies a token-bucket rate limiter to enforce
bandwidth, draws a random delay from `Normal(τ, σ_τ)`, and independently drops each
packet with probability `p_loss`.  This is sufficient to reproduce the qualitative
behaviour of Wi-Fi and 4G links in indoor environments.

## Topics Crossing the Wireless Link

The following ROS2 topics are serialized, compressed (where noted), and forwarded across
the simulated wireless link:

### Robot → Base Station (uplink)

| Topic | Message Type | Hz | Compression | Priority |
|-------|-------------|-----|------------|---------|
| `/robot_N/map/diff` | `nav_msgs/OccupancyGrid` | 0.5–2 | zlib | HIGH |
| `/robot_N/odom` | `nav_msgs/Odometry` | 5 | none | HIGH |
| `/robot_N/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 | none | MEDIUM |
| `/robot_N/scan/compressed` | custom binary blob | 1 | zlib | LOW |
| `/robot_N/image/compressed` | `sensor_msgs/CompressedImage` | 2 | JPEG | LOW |

### Base Station → Robot (downlink)

| Topic | Message Type | Hz | Notes |
|-------|-------------|-----|-------|
| `/global_map` | `nav_msgs/OccupancyGrid` | 0.2 | full global map on demand |
| `/robot_N/goal` | `geometry_msgs/PoseStamped` | event | new navigation goal from operator |
| `/robot_N/plan_update` | custom `PlanUpdate` | event | replanned global path |
| `/fleet/status` | custom `FleetStatus` | 1 | positions of all robots |

## QoS Profiles

ROS2 DDS QoS policies control the reliability and durability of each topic.  Three
profiles are defined in `config/comms_qos_profiles.yaml`:

### `reliable_transient_local`
Used for map diffs and goal messages.  Guarantees delivery (RELIABLE reliability) and
keeps the last sample alive for late-joining subscribers (TRANSIENT_LOCAL durability).
Suitable for infrequent, high-value data.

### `best_effort_volatile`
Used for odometry and scan streams.  No delivery guarantee (BEST_EFFORT reliability)
and no persistence (VOLATILE durability).  Minimises per-message overhead on
bandwidth-constrained links.

### `reliable_volatile`
Used for diagnostic and fleet-status messages.  Guaranteed delivery but no persistence.

```yaml
# comms_qos_profiles.yaml excerpt
profiles:
  reliable_transient_local:
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
    history: KEEP_LAST
    depth: 10
  best_effort_volatile:
    reliability: BEST_EFFORT
    durability: VOLATILE
    history: KEEP_LAST
    depth: 1
  reliable_volatile:
    reliability: RELIABLE
    durability: VOLATILE
    history: KEEP_LAST
    depth: 5
```

## Compression and Serialization

- **Map diffs**: only cells whose log-odds value has changed by more than a threshold
  since the last transmission are included.  The diff is encoded as a run-length
  compressed byte string before being wrapped in a ROS2 message.
- **LiDAR scans**: reduced to a compact binary format (x, y, z, intensity as float16)
  before zlib compression.  A full 16-beam scan compressed this way is typically 4–8 KB.
- **Images**: JPEG-compressed at quality 40–60; resolution can be scaled via the
  `perception.image_downscale` config key.

## How the Link Simulator Is Used

In simulation (`sim/sim_runner.py`), each robot's edge client calls
`LinkSimulator.send(topic, payload)`.  The simulator applies the channel model and calls
the registered callback on the base station side.  Both sides run in the same Python
process for simplicity; in a real deployment they would be separate processes connected
over UDP/TCP.

For offline analysis, `analysis/plot_latency_vs_nav_error.py` sweeps the `base_latency_ms`
parameter and records navigation error, reproducing the latency–error trade-off curves.
