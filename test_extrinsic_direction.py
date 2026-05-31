#!/usr/bin/env python3
"""Comprehensive extrinsic matrix direction test."""

import sys
import numpy as np
import cv2
from pathlib import Path

# Load extrinsic
extrinsic_path = Path("/home/lzt/m20pro/calib_data/live_once/extrinsic.txt")
rows = []
for line in extrinsic_path.read_text().splitlines():
    line = line.strip()
    if not line:
        continue
    values = [float(v.strip()) for v in line.split(",") if v.strip()]
    rows.append(values)

M = np.asarray(rows, dtype=np.float64)
M_inv = np.eye(4, dtype=np.float64)
M_inv[:3, :3] = M[:3, :3].T
M_inv[:3, 3] = -M[:3, :3].T @ M[:3, 3]

# RealSense D435i intrinsics (from calibration result or manual)
# Common values for 1280x720 resolution
fx = 640.7
fy = 640.1
cx = 640.0
cy = 360.0

K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
dist_coeffs = np.zeros((1, 5), dtype=np.float64)

print("=" * 80)
print("Testing 4 scenarios to find the correct extrinsic direction")
print("=" * 80)

# Create a simple test point in LiDAR frame
# This point should appear somewhere reasonable in the image
test_point_lidar = np.array([[0.5, 0.0, 1.0]], dtype=np.float64)  # 50cm forward, 1m above

scenarios = [
    ("M (direct) + K + standard", M, K, "standard"),
    ("M_inv (inverted) + K + standard", M_inv, K, "standard"),
]

# Also try to get camera intrinsics from a potential camera_info topic
# or from ROS parameters

for name, M_test, K_test, note in scenarios:
    print(f"\n{name}")
    print("-" * 80)
    
    R = M_test[:3, :3]
    T = M_test[:3, 3:].reshape(3, 1)
    
    # Transform point to camera frame
    ones = np.ones((test_point_lidar.shape[0], 1))
    points_h = np.hstack([test_point_lidar, ones])
    points_cam = (M_test @ points_h.T).T[:, :3]
    
    print(f"LiDAR point: {test_point_lidar[0]}")
    print(f"Camera point: {points_cam[0]}")
    print(f"Camera Z (should be > 0): {points_cam[0, 2]:.4f}")
    
    if points_cam[0, 2] <= 0:
        print("⚠️  Point is behind camera (z <= 0), this matrix direction is WRONG")
        continue
    
    # Project using cv2.projectPoints
    rvec, _ = cv2.Rodrigues(R)
    tvec = T.reshape(3, 1)
    
    try:
        pixels, _ = cv2.projectPoints(
            test_point_lidar.reshape(-1, 1, 3).astype(np.float64),
            rvec,
            tvec,
            K_test,
            dist_coeffs,
        )
        pixel = pixels.reshape(-1, 2)[0]
        
        print(f"Projected pixel (cv2): {pixel}")
        
        # Check if pixel is within image bounds (1280x720)
        in_bounds = 0 <= pixel[0] < 1280 and 0 <= pixel[1] < 720
        print(f"In image bounds [0-1280, 0-720]: {in_bounds}")
        
        if in_bounds:
            print("✓ This seems like a valid configuration!")
        else:
            print("✗ Pixel out of bounds, probably wrong direction")
            
    except Exception as e:
        print(f"Error during projection: {e}")

print("\n" + "=" * 80)
print("Next step: Run pick_lidar_xy.py and click on actual points to verify")
print("=" * 80)
