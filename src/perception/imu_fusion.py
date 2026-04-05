"""
imu_fusion.py
-------------
IMU + wheel-odometry fusion node.

Implements a complementary filter that blends IMU gyro integration with
accelerometer-based tilt estimation for roll/pitch, and uses wheel odometry for
translational velocity and heading drift correction.

A stub Extended Kalman Filter (EKF) class is provided as a placeholder for a
full 15-state navigation filter that would handle IMU bias estimation and GPS/
SLAM pose corrections.

Subscribes:
    /robot/imu/raw          sensor_msgs/Imu         200 Hz
    /robot/odom/wheel       nav_msgs/Odometry        50 Hz
    /robot/slam/pose_corr   geometry_msgs/PoseStamped  (aperiodic, from SLAM)

Publishes:
    /robot/odom/fused       nav_msgs/Odometry        50 Hz
    /robot/imu/filtered     sensor_msgs/Imu         200 Hz
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


# ---------------------------------------------------------------------------
# Complementary filter for attitude estimation
# ---------------------------------------------------------------------------

@dataclass
class ComplementaryFilterState:
    """Attitude state maintained by the complementary filter."""
    roll: float = 0.0       # rad
    pitch: float = 0.0      # rad
    yaw: float = 0.0        # rad (integrated from gyro only)
    timestamp: float = 0.0  # seconds


class ComplementaryFilter:
    """Lightweight attitude estimator fusing gyro and accelerometer.

    The filter blends high-frequency gyro integration with a low-frequency
    tilt estimate derived from the gravity vector measured by the accelerometer.
    Yaw is integrated from the gyro because accelerometers cannot observe heading.

    Args:
        alpha: Complementary filter coefficient in [0, 1].
               0 = full gyro (drifts), 1 = full accelerometer (noisy).
               Typical value: 0.02 (high-pass cut-off ≈ 0.05 Hz).

    TODO:
        - Add magnetometer yaw correction.
        - Detect and reject accelerometer spikes during high-acceleration events.
    """

    def __init__(self, alpha: float = 0.02) -> None:
        self._alpha = alpha
        self.state = ComplementaryFilterState()

    def update(self,
               gyro: np.ndarray,
               accel: np.ndarray,
               dt: float) -> ComplementaryFilterState:
        """Update the filter with one IMU sample.

        Args:
            gyro:  Angular velocity [gx, gy, gz] in rad/s.
            accel: Linear acceleration [ax, ay, az] in m/s².
            dt:    Time delta since last update in seconds.

        Returns:
            Updated attitude state.
        """
        # Gyro integration
        self.state.roll  += gyro[0] * dt
        self.state.pitch += gyro[1] * dt
        self.state.yaw   += gyro[2] * dt

        # Accelerometer-based tilt estimate (only valid under near-1 g)
        a_norm = np.linalg.norm(accel)
        if 0.5 < a_norm < 1.5 * 9.81:  # sanity check: between 0.5 g and 1.5 g
            roll_accel  = math.atan2(accel[1], accel[2])
            pitch_accel = math.atan2(-accel[0],
                                     math.sqrt(accel[1]**2 + accel[2]**2))
            # Complementary blend
            self.state.roll  = (1 - self._alpha) * self.state.roll  + self._alpha * roll_accel
            self.state.pitch = (1 - self._alpha) * self.state.pitch + self._alpha * pitch_accel

        return self.state


# ---------------------------------------------------------------------------
# EKF stub (15-state navigation filter)
# ---------------------------------------------------------------------------

class NavigationEKF:
    """Stub Extended Kalman Filter for full navigation state estimation.

    State vector (15 elements):
        [px, py, pz,        — position (m)
         vx, vy, vz,        — velocity (m/s)
         roll, pitch, yaw,  — attitude (rad)
         bwx, bwy, bwz,     — gyro bias (rad/s)
         bax, bay, baz]     — accel bias (m/s²)

    TODO:
        - Implement process_model() and its Jacobian F.
        - Implement imu_prediction_step().
        - Implement odometry_update() measurement model.
        - Implement slam_pose_update() measurement model.
        - Handle numerical stability via square-root UKF or Joseph form.
    """

    STATE_DIM = 15

    def __init__(self) -> None:
        self.x = np.zeros(self.STATE_DIM)          # state estimate
        self.P = np.eye(self.STATE_DIM) * 0.1      # covariance
        # Process noise — tuned empirically
        self.Q = np.diag([
            1e-4, 1e-4, 1e-4,   # position
            1e-3, 1e-3, 1e-3,   # velocity
            1e-4, 1e-4, 1e-4,   # attitude
            1e-6, 1e-6, 1e-6,   # gyro bias
            1e-5, 1e-5, 1e-5,   # accel bias
        ])
        self._initialized = False

    def initialize(self, position: np.ndarray, yaw: float) -> None:
        """Seed the filter from a known initial pose."""
        self.x[:3] = position
        self.x[8] = yaw
        self._initialized = True

    def predict(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> None:
        """IMU-driven prediction step.

        TODO: implement process_model(), compute F, propagate P = F P Fᵀ + Q.
        """
        if not self._initialized:
            return
        # Placeholder: simple Euler integration of velocity/position
        accel_corrected = accel - self.x[12:15]   # subtract accel bias
        gyro_corrected  = gyro  - self.x[9:12]    # subtract gyro bias
        self.x[3:6] += accel_corrected * dt        # v += a * dt
        self.x[0:3] += self.x[3:6] * dt           # p += v * dt
        self.x[6:9] += gyro_corrected * dt         # attitude += omega * dt

    def update_odometry(self, odom_vel: np.ndarray, R_odom: np.ndarray) -> None:
        """EKF measurement update from wheel odometry velocity.

        Args:
            odom_vel: Measured velocity [vx, vy] in robot frame (m/s).
            R_odom:   2×2 measurement noise covariance.

        TODO: implement full linearised measurement update.
        """
        # Stub: copy velocity directly (no covariance propagation yet)
        self.x[3:5] = odom_vel

    def update_slam_pose(self, pose: np.ndarray, R_slam: np.ndarray) -> None:
        """EKF measurement update from SLAM pose correction.

        Args:
            pose:   [px, py, yaw] observation from SLAM.
            R_slam: 3×3 measurement noise covariance.

        TODO: implement linearised measurement update with outlier rejection.
        """
        self.x[0:2] = pose[0:2]
        self.x[8]   = pose[2]


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

if _ROS2_AVAILABLE:

    class ImuFusionNode(Node):
        """ROS2 node that fuses IMU and wheel odometry into a smoothed pose.

        TODO:
            - Wire up actual ROS2 message callbacks.
            - Serialise fused pose to nav_msgs/Odometry and publish TF.
            - Replace ComplementaryFilter with NavigationEKF for full state.
        """

        def __init__(self) -> None:
            super().__init__("imu_fusion")
            self._comp_filter = ComplementaryFilter(alpha=0.02)
            self._ekf = NavigationEKF()
            self._last_imu_time: Optional[float] = None

            self._sub_imu = self.create_subscription(
                Imu, "imu/raw", self._imu_callback, 200
            )
            self._sub_odom = self.create_subscription(
                Odometry, "odom/wheel", self._odom_callback, 50
            )
            self._pub_odom = self.create_publisher(Odometry, "odom/fused", 50)
            self._pub_imu  = self.create_publisher(Imu, "imu/filtered", 200)

            self.get_logger().info("ImuFusionNode initialised.")

        def _imu_callback(self, msg: Imu) -> None:
            """Process one IMU sample.

            TODO:
                - Extract gyro / accel from msg.
                - Compute dt from header stamp.
                - Update complementary filter.
                - Run EKF prediction step.
                - Publish filtered IMU.
            """
            pass

        def _odom_callback(self, msg: Odometry) -> None:
            """Process one wheel odometry sample.

            TODO:
                - Extract wheel velocity from msg.
                - Run EKF odometry update step.
                - Publish fused Odometry with updated covariance.
            """
            pass


def main() -> None:
    if not _ROS2_AVAILABLE:
        print("rclpy not available — cannot run as a ROS2 node.")
        return
    rclpy.init()
    node = ImuFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
