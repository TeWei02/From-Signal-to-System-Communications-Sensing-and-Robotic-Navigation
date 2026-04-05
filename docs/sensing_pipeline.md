# Sensing Pipeline

## Overview

The sensing pipeline converts raw hardware output into structured, time-stamped data
products that downstream perception and mapping modules can consume.  Three modalities
are supported: **LiDAR**, **IMU**, and **monocular camera**.  Each modality has a
dedicated preprocessing stage and a shared time-synchronization mechanism based on
ROS2 message filters (`message_filters.ApproximateTimeSynchronizer`).

## LiDAR Data Path

### Hardware
A rotating 3-D LiDAR (e.g., Velodyne VLP-16 or equivalent simulation in Gazebo) publishes
raw `sensor_msgs/PointCloud2` messages at 10 Hz on `/robot_N/scan/raw`.  Each scan
contains approximately 30 000 points at 16-beam resolution.

### Preprocessing (`src/perception/lidar_preprocess.py`)

1. **Voxel downsampling** — reduces point density using a 0.05 m leaf size, yielding
   roughly 3 000–5 000 points per scan.
2. **Statistical outlier removal** — removes points whose mean distance to their k
   nearest neighbours deviates from the local mean by more than 1 standard deviation.
3. **Ground plane segmentation** — RANSAC plane fit; points classified as ground are
   labelled but not discarded (they are still used for traction estimation).
4. **Normal estimation** — surface normals computed for downstream SLAM feature matching.
5. **Output** — publishes `sensor_msgs/PointCloud2` on `/robot_N/scan/filtered` and
   writes `.pcd` snapshots to `data/maps/` when the `save_pcd` flag is set.

**Key parameters** (from `config/sensors.yaml`):
```yaml
lidar:
  voxel_leaf_size: 0.05       # metres
  outlier_k_neighbours: 20
  outlier_std_ratio: 1.0
  ransac_distance_threshold: 0.03
```

## IMU Data Path

### Hardware
A 6-DOF IMU publishes `sensor_msgs/Imu` at 200 Hz on `/robot_N/imu/raw`.
Wheel encoders provide odometry at 50 Hz on `/robot_N/odom/wheel`.

### Fusion (`src/perception/imu_fusion.py`)

The fusion node implements a complementary filter as a lightweight first-order
approximation:

- **Attitude** (roll, pitch): high-pass-filtered from IMU gyro integration + low-pass
  from accelerometer-based tilt estimate, blended at cutoff frequency 0.05 Hz.
- **Heading (yaw)**: integrated from gyro with a slow drift correction from the SLAM
  pose estimate.
- **Velocity / position**: propagated via wheel odometry; IMU acceleration is used only
  as a short-term predictor during wheel-slip events.

A full Extended Kalman Filter (EKF) stub is provided in the same file for future
replacement; it defines the state vector, process model Jacobians, and measurement
update equations as TODO items.

**Published topics**:
- `/robot_N/odom/fused` — `nav_msgs/Odometry` at 50 Hz
- `/robot_N/imu/filtered` — `sensor_msgs/Imu` at 200 Hz (bias-corrected)

## Camera Data Path

### Hardware
A forward-facing camera publishes `sensor_msgs/Image` at 30 Hz on `/robot_N/camera/raw`
(BGRA8 encoding, 640×480).

### Frontend (`src/perception/vision_frontend.py`)

- **Undistortion**: applies the Brown–Conrady distortion model from `sensors.yaml`.
- **Feature extraction**: ORB keypoint detection and descriptor computation (500 features
  per frame).
- **Temporal matching**: brute-force descriptor matching between consecutive frames for
  visual odometry fallback.
- **Semantic segmentation stub**: placeholder for a lightweight MobileNetV3 inference
  call that would label pixels as floor / wall / obstacle / unknown.

**Published topics**:
- `/robot_N/camera/undistorted` — `sensor_msgs/Image`
- `/robot_N/camera/features` — custom `FeatureArray` message

## Time Synchronization

All three modalities are aligned in the SLAM front-end node
(`src/perception/slam_frontend_node.py`) using `message_filters.ApproximateTimeSynchronizer`
with a 50 ms slop tolerance.  Dropped synchronization windows are counted and exposed
via `/robot_N/diagnostics` so that link-quality degradation can be detected.

## Data Logging

- Raw scans can be saved to ROS2 bag files (`data/logs/sim_run_NNN.bag`) by setting
  `logging.record_raw: true` in `config/robot_params.yaml`.
- Filtered scans are optionally snapshotted as `.pcd` files in `data/maps/`.
- Pre-computed analysis arrays (coverage, bandwidth traces) are stored as `.npz` in
  `data/generated/`.
