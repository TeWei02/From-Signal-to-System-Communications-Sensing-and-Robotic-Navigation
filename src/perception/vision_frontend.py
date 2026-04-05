"""
vision_frontend.py
------------------
Camera perception front-end: undistortion, feature extraction, and a stub for
monocular visual odometry and semantic segmentation.

Subscribes:
    /robot/camera/raw          sensor_msgs/Image   30 Hz

Publishes:
    /robot/camera/undistorted  sensor_msgs/Image   30 Hz
    /robot/camera/features     custom/FeatureArray 30 Hz  (stub message type)

TODO:
    - Replace stub FeatureArray with a proper custom message.
    - Add temporal feature matching for visual odometry fallback.
    - Integrate a lightweight MobileNetV3 semantic segmentation model.
    - Add exposure / white-balance control via camera driver service calls.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2
    _CV2_AVAILABLE = True
except ImportError:
    _CV2_AVAILABLE = False

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False


@dataclass
class CameraIntrinsics:
    """Pinhole camera intrinsics + Brown-Conrady distortion model."""
    fx: float = 525.0
    fy: float = 525.0
    cx: float = 319.5
    cy: float = 239.5
    distortion_coeffs: tuple = (-0.28, 0.07, 0.0, 0.0, 0.0)
    width: int = 640
    height: int = 480

    @property
    def K(self) -> np.ndarray:
        return np.array([
            [self.fx,     0.0, self.cx],
            [    0.0, self.fy, self.cy],
            [    0.0,     0.0,     1.0],
        ], dtype=np.float64)

    @property
    def D(self) -> np.ndarray:
        return np.array(self.distortion_coeffs, dtype=np.float64)


def undistort_image(image: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray:
    """Apply lens distortion correction to an image.

    Args:
        image: Input BGR image as (H, W, 3) uint8 array.
        intrinsics: Camera intrinsics and distortion coefficients.

    Returns:
        Undistorted image.

    TODO:
        - Cache the undistortion map (cv2.initUndistortRectifyMap) for speed.
        - Support fisheye model for wide-angle cameras.
    """
    if not _CV2_AVAILABLE:
        return image  # passthrough if OpenCV unavailable
    return cv2.undistort(image, intrinsics.K, intrinsics.D)


def extract_orb_features(image: np.ndarray,
                          n_features: int = 500
                          ) -> tuple[list, Optional[np.ndarray]]:
    """Detect ORB keypoints and compute descriptors.

    Args:
        image: Greyscale or BGR image.
        n_features: Maximum number of features to retain.

    Returns:
        Tuple of (keypoints, descriptors).  descriptors is None if no features found.

    TODO:
        - Implement uniform spatial distribution using a grid-based suppression.
        - Add subpixel refinement with cv2.cornerSubPix.
    """
    if not _CV2_AVAILABLE:
        return [], None

    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    orb = cv2.ORB_create(nfeatures=n_features)
    keypoints, descriptors = orb.detectAndCompute(grey, None)
    return keypoints, descriptors


def match_features(desc_prev: np.ndarray,
                   desc_curr: np.ndarray,
                   ratio_thresh: float = 0.75
                   ) -> list:
    """Brute-force descriptor matching with Lowe ratio test.

    Args:
        desc_prev: Descriptors from the previous frame (N, 32) uint8.
        desc_curr: Descriptors from the current frame (M, 32) uint8.
        ratio_thresh: Ratio test threshold (Lowe 2004).

    Returns:
        List of cv2.DMatch objects passing the ratio test.

    TODO:
        - Switch to FLANN-based matcher for larger descriptor sets.
        - Add geometric verification (fundamental matrix RANSAC).
    """
    if not _CV2_AVAILABLE or desc_prev is None or desc_curr is None:
        return []

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw_matches = bf.knnMatch(desc_prev, desc_curr, k=2)
    good_matches = []
    for m_pair in raw_matches:
        if len(m_pair) == 2 and m_pair[0].distance < ratio_thresh * m_pair[1].distance:
            good_matches.append(m_pair[0])
    return good_matches


if _ROS2_AVAILABLE:

    class VisionFrontendNode(Node):
        """ROS2 node wrapping the camera preprocessing and feature extraction pipeline."""

        def __init__(self, intrinsics: Optional[CameraIntrinsics] = None) -> None:
            super().__init__("vision_frontend")
            self._intrinsics = intrinsics or CameraIntrinsics()
            self._bridge = CvBridge()
            self._prev_descriptors: Optional[np.ndarray] = None

            self._sub = self.create_subscription(
                Image, "camera/raw", self._image_callback, 30
            )
            self._pub_undistorted = self.create_publisher(Image, "camera/undistorted", 30)
            # TODO: publish FeatureArray once custom message is defined
            self.get_logger().info("VisionFrontendNode initialised.")

        def _image_callback(self, msg: Image) -> None:
            """Process one camera frame.

            TODO:
                - Convert msg to numpy via CvBridge.
                - Call undistort_image().
                - Call extract_orb_features().
                - Match against previous frame for visual odometry.
                - Publish undistorted image and feature array.
            """
            pass


def main() -> None:
    if not _ROS2_AVAILABLE:
        print("rclpy not available.")
        return
    rclpy.init()
    node = VisionFrontendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
