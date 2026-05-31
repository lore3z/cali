#!/usr/bin/env python3
"""
Quick test script to verify which extrinsic matrix direction is correct.

Usage:
  python3 verify_extrinsic_direction.py

This script:
1. Loads the calibration result extrinsic.txt
2. Listens to real sensor data
3. Projects a point and checks if the pixel is in-bounds
4. Tells you which direction works
"""

import sys
import time
import numpy as np
import cv2
import rclpy
from pathlib import Path
from typing import Optional
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge


def load_extrinsic_matrix(path: Path) -> np.ndarray:
    """Load extrinsic matrix from CSV format."""
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        values = [float(v.strip()) for v in line.split(",") if v.strip()]
        rows.append(values)
    return np.asarray(rows, dtype=np.float64)


class ExtrinsicTester(Node):
    def __init__(self):
        super().__init__("extrinsic_tester")
        self.bridge = CvBridge()
        
        # Load extrinsic
        self.extrinsic = load_extrinsic_matrix(
            Path("/home/lzt/m20pro/calib_data/live_once/extrinsic.txt")
        )
        self.extrinsic_inv = np.eye(4, dtype=np.float64)
        self.extrinsic_inv[:3, :3] = self.extrinsic[:3, :3].T
        self.extrinsic_inv[:3, 3] = -self.extrinsic[:3, :3].T @ self.extrinsic[:3, 3]
        
        # Hard-coded RealSense intrinsics
        fx, fy = 918.3, 917.6
        cx, cy = 640.0, 359.6
        self.K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)
        
        self.latest_image: Optional[np.ndarray] = None
        self.latest_points: Optional[np.ndarray] = None
        
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            "/livox/lidar",
            self.cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        
        self.get_logger().info("Listening to sensor data... (max 10 seconds)")
        self.start_time = time.time()
    
    def image_callback(self, msg: Image) -> None:
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
    
    def cloud_callback(self, msg: PointCloud2) -> None:
        points = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ))
        if points:
            self.latest_points = np.array(points[:100], dtype=np.float64)  # Take first 100 points

    def test(self):
        """Run the test."""
        if self.latest_image is None:
            self.get_logger().error("No image received!")
            return False
        
        if self.latest_points is None or len(self.latest_points) == 0:
            self.get_logger().error("No points received!")
            return False
        
        self.get_logger().info(f"\nTesting with {len(self.latest_points)} points")
        self.get_logger().info(f"Image shape: {self.latest_image.shape}")
        
        results = {}
        
        for name, M in [("Original", self.extrinsic), ("Inverted", self.extrinsic_inv)]:
            R = M[:3, :3]
            T = M[:3, 3:].reshape(3, 1)
            
            # Project points
            ones = np.ones((self.latest_points.shape[0], 1))
            points_h = np.hstack([self.latest_points, ones])
            points_cam = (M @ points_h.T).T[:, :3]
            
            # Filter points in front of camera
            valid_mask = points_cam[:, 2] > 0.05
            points_cam_valid = points_cam[valid_mask]
            
            if len(points_cam_valid) == 0:
                self.get_logger().warn(f"{name}: All points behind camera!")
                results[name] = {"in_view": 0, "projected": 0, "success": False}
                continue
            
            # Project using cv2
            rvec, _ = cv2.Rodrigues(R)
            tvec = T.reshape(3, 1)
            
            pixels, _ = cv2.projectPoints(
                self.latest_points.reshape(-1, 1, 3).astype(np.float64),
                rvec,
                tvec,
                self.K,
                self.dist_coeffs,
            )
            pixels = pixels.reshape(-1, 2)
            
            # Check bounds
            h, w = self.latest_image.shape[:2]
            in_view = (pixels[:, 0] >= 0) & (pixels[:, 0] < w) & \
                      (pixels[:, 1] >= 0) & (pixels[:, 1] < h)
            
            num_in_view = np.sum(in_view)
            num_in_front = np.sum(valid_mask)
            
            success = num_in_view > len(self.latest_points) * 0.1  # At least 10% in view
            results[name] = {
                "in_view": num_in_view,
                "projected": num_in_front,
                "success": success
            }
            
            self.get_logger().info(f"\n{name}:")
            self.get_logger().info(f"  Points in front of camera: {num_in_front}")
            self.get_logger().info(f"  Points projected in image: {num_in_view}")
            self.get_logger().info(f"  Result: {'✓ VALID' if success else '✗ INVALID'}")
        
        # Determine winner
        original_ok = results["Original"]["success"]
        inverted_ok = results["Inverted"]["success"]
        
        self.get_logger().info("\n" + "="*60)
        if original_ok and not inverted_ok:
            self.get_logger().info("✓ Use ORIGINAL extrinsic matrix")
            self.get_logger().info("  Command: python3 pick_lidar_xy.py")
        elif inverted_ok and not original_ok:
            self.get_logger().info("✓ Use INVERTED extrinsic matrix")
            self.get_logger().info("  Command: python3 pick_lidar_xy_debug.py --invert-extrinsic")
        elif original_ok and inverted_ok:
            self.get_logger().warn("⚠️ Both directions work! Visual inspection needed.")
        else:
            self.get_logger().error("✗ Neither matrix direction works!")
            self.get_logger().error("  Likely causes:")
            self.get_logger().error("    1. Camera intrinsics are wrong")
            self.get_logger().error("    2. Extrinsic calibration failed (try re-calibration)")
        self.get_logger().info("="*60)
        
        return original_ok or inverted_ok


def main():
    rclpy.init()
    node = ExtrinsicTester()
    
    start = time.time()
    while time.time() - start < 12 and not node.latest_image and not node.latest_points:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    time.sleep(0.5)  # Get a bit more data
    node.test()
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
