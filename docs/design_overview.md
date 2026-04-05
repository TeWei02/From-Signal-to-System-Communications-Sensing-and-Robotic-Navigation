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
