# System Design Overview

## Introduction

This document describes the high-level architecture of the *From Signal to System* robot
navigation platform.  The system spans the full stack from raw sensor acquisition through
wireless communication to analysis at a base station.  The goal is to make every design
decision explicit so that the communication, computation, and navigation trade-offs can be
studied in isolation or jointly.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ROBOT ONBOARD                                  │
│                                                                             │
│  ┌──────────┐    ┌───────────┐    ┌────────────┐    ┌──────────────────┐   │
│  │ Sensors  │───▶│Perception │───▶│  Mapping   │───▶│    Planning      │   │
│  │LiDAR/IMU │    │ Pipeline  │    │(Occ. Grid) │    │ (Global+Local)   │   │
│  │ /Camera  │    │           │    │            │    │                  │   │
│  └──────────┘    └───────────┘    └────────────┘    └────────┬─────────┘   │
│                                                               │             │
│                                                    ┌──────────▼─────────┐  │
│                                                    │  Motion Controller │  │
│                                                    │  (velocity cmds)   │  │
│                                                    └──────────┬─────────┘  │
│                                                               │             │
│                                              ┌────────────────▼──────────┐ │
│                                              │   Comms / Edge Client     │ │
│                                              │  (compression + QoS mgmt) │ │
│                                              └────────────────┬──────────┘ │
└───────────────────────────────────────────────────────────────┼────────────┘
                                                                │ wireless link
                                              ┌─────────────────▼──────────┐
                                              │       BASE STATION          │
                                              │  (map fusion, fleet mgmt,  │
                                              │   logging, analysis)        │
                                              └────────────────────────────┘
```

## Block Descriptions

### 1. Sensors
The robot carries three primary sensor modalities:

- **LiDAR** (2-D or 3-D): provides dense range measurements at 10–20 Hz.
- **IMU** (6-DOF): provides high-rate (200 Hz) angular velocity and linear acceleration.
- **Monocular / stereo camera**: used for semantic cues and visual odometry fallback.

Raw data rates can exceed 10 Mb/s for a 16-beam LiDAR, motivating on-board preprocessing
before any wireless transmission.

### 2. Perception Pipeline
`src/perception/` implements the following steps:

- **LiDAR preprocessing** (`lidar_preprocess.py`): voxel downsampling, statistical outlier
  removal, ground-plane segmentation.
- **IMU fusion** (`imu_fusion.py`): complementary filter / EKF stub that combines wheel
  odometry with IMU to produce a smooth odometry estimate.
- **SLAM front-end** (`slam_frontend_node.py`): scan-matching node (ICP / GICP skeleton)
  that publishes pose and a local submap.

### 3. Mapping
`src/mapping/` maintains the global representation:

- **Occupancy grid** (`occupancy_grid_mapper.py`): 2-D probabilistic grid updated with
  Bresenham ray-casting; supports incremental serialization for efficient transmission.
- **Costmap** (`costmap_generator.py`): inflates obstacles by a configurable radius and
  adds semantic layers for planning.

### 4. Planning
`src/planning/` implements a two-level planner:

- **Global planner** (`global_planner_stub.py`): A* on the costmap; planned path is
  replanned whenever the costmap changes significantly.
- **Local planner** (`local_planner.py`): short-horizon trajectory optimizer (DWA-like)
  operating at 10 Hz.
- **Trajectory roller** (`trajectory_roller.py`): forward-simulates candidate arcs and
  selects the minimum-cost admissible trajectory.

### 5. Control
`src/control/` provides the low-level interface:

- `motion_controller.cpp`: PID velocity controller tracking reference trajectories.
- `low_level_interface.cpp`: hardware abstraction for wheel encoders and motor drivers.

### 6. Communication
`src/comms/` wraps all wireless interactions:

- **Link simulator** (`link_simulator.py`): configurable bandwidth, latency, and packet
  loss for offline evaluation.
- **Edge client** (`edge_client_node.py`): compresses and queues outgoing perception data.
- **Base station node** (`base_station_node.py`): receives, decompresses, and fuses data
  from multiple robots; publishes global map updates back to the fleet.

### 7. Analysis
`analysis/` and `notebooks/` provide post-hoc evaluation:

- Coverage metrics, bandwidth utilization, latency-vs-error curves.
- Jupyter notebooks for interactive exploration of `.npz` result files.

## Key System Parameters

| Parameter | Nominal Value | Tunable |
|-----------|--------------|---------|
| LiDAR scan rate | 10 Hz | No |
| Map update rate | 2 Hz | Yes |
| Uplink bandwidth budget | 500 kbps | Yes |
| Round-trip latency | 20–150 ms | Simulated |
| Packet loss rate | 0–10 % | Simulated |
| Planning horizon | 2 s | Yes |

## System Components by Layer

### Hardware Layer
- **Actuators**: Differential-drive motors with PID velocity control
- **Sensors**: 3-D LiDAR, 6-DOF IMU, monocular/stereo camera, wheel encoders
- **Compute**: Arm-based SoC (e.g., NVIDIA Jetson Xavier) or x86 CPU in simulation

### Software Stack

**Onboard (Robot)**:
1. Device drivers (sensor, motor) — low-level hardware abstraction
2. Perception pipeline — sensor fusion, feature extraction, object detection
3. Mapping — incremental occupancy grid maintenance
4. Planning — global path + local trajectory optimization
5. Control — PID velocity tracking
6. Communication client — message compression, QoS management

**Edge/Cloud (Base Station)**:
1. Data ingestion — decompresses and time-aligns multi-robot data streams
2. Map fusion — merges incoming diffs into a global occupancy grid
3. Planning (optional) — global path optimization for multi-robot coordination
4. Telemetry viewer — visualizes robot poses, coverage, link statistics
5. Logging backend — stores all messages for post-hoc analysis

### Simulation Stack
- World simulation (obstacle layout, ground truth poses)
- Sensor simulators (ray-casting for LiDAR, physics for IMU)
- Wireless link simulator (bandwidth, latency, loss)
- Control loop -- runs robot dynamics and perception update

## Execution Profiles

The system supports three operational modes:

1. **Pure simulation** (`sim/sim_runner.py`): Runs all robots, world, and communication
   in a single Python process with step-wise synchronization. Suitable for parameter sweeps
   and bandwidth-latency trade-off studies.

2. **Hardware-in-loop** (TODO): Runs simulation world but communicates with real robot(s)
   over a simulated (bandwidth-limited) link.

3. **Live deployment** (TODO): Robot runs on real hardware, communicates with a real
   base station over a physical wireless link.

## Development Workflow

### Quick Start (Simulation)
```bash
cd sim
python sim_runner.py scenarios/multi_robot_explore_01.yaml --time 300 --output ../data/generated
```

### Analysis
```bash
python ../analysis/coverage_analysis.py ../data/generated/coverage_stats_run_*.npz
jupyter notebook ../notebooks/01_snr_vs_packet_error_rate.ipynb
```

### Build (ROS2 + C++ Control)
```bash
# In a ROS2-sourced environment:
colcon build --symlink-install
source install/setup.bash
ros2 launch from_signal_to_system single_robot_nav.launch.py
```
