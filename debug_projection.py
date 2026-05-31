#!/usr/bin/env python3
"""Debug script to verify projection math and coordinate transformations."""

import numpy as np
import cv2
from pathlib import Path

# Load extrinsic matrix
extrinsic_path = Path("/home/lzt/m20pro/calib_data/live_once/extrinsic.txt")
rows = []
for line in extrinsic_path.read_text().splitlines():
    line = line.strip()
    if not line:
        continue
    values = [float(v.strip()) for v in line.split(",") if v.strip()]
    rows.append(values)

extrinsic = np.asarray(rows, dtype=np.float64)
print("Extrinsic Matrix (4x4):")
print(extrinsic)
print()

# Extract rotation and translation
R = extrinsic[:3, :3]
T = extrinsic[:3, 3].reshape(3, 1)

print("Rotation Matrix R:")
print(R)
print()

print("Translation Vector T:")
print(T.flatten())
print()

# Verify it's a valid rotation (det(R) ≈ 1, R^T @ R ≈ I)
det_R = np.linalg.det(R)
print(f"det(R) = {det_R:.6f} (should be ≈ 1.0 for valid rotation)")
print()

identity_check = R.T @ R
print("R^T @ R (should be ≈ I):")
print(identity_check)
print()

# Test with a sample point
test_point_lidar = np.array([[0.5, 0.3, 1.0]], dtype=np.float64)
print(f"Test LiDAR point: {test_point_lidar}")

# Method 1: Direct matrix multiplication (as in pick_lidar_xy.py)
ones = np.ones((test_point_lidar.shape[0], 1), dtype=np.float64)
points_h = np.hstack([test_point_lidar, ones])
points_camera_method1 = (extrinsic @ points_h.T).T[:, :3]
print(f"Camera point (Method 1 - full 4x4): {points_camera_method1}")

# Method 2: Decomposed (R @ P + T)
points_camera_method2 = (R @ test_point_lidar.T + T).T
print(f"Camera point (Method 2 - decomposed): {points_camera_method2}")

# Method 3: cv2.projectPoints style
# cv2.projectPoints expects rotation vector (from cv2.Rodrigues) and translation vector
rvec, _ = cv2.Rodrigues(R)
tvec = T.reshape(3, 1)

print(f"\nrvec (rotation vector): {rvec.flatten()}")
print(f"tvec (translation vector): {tvec.flatten()}")

# Verify Rodrigues reconstruction
R_recon, _ = cv2.Rodrigues(rvec)
print(f"\nRecovered R from Rodrigues (should match original R):")
print(np.allclose(R, R_recon))
print()

# Now test with some real camera intrinsics
# RealSense D435i typical values
fx = 640.7
fy = 640.1
cx = 640.0
cy = 360.0

K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
dist_coeffs = np.zeros((1, 5), dtype=np.float64)

print(f"Camera intrinsics K:")
print(K)
print()

# Project using cv2.projectPoints
pixels, _ = cv2.projectPoints(
    test_point_lidar.reshape(-1, 1, 3).astype(np.float64),
    rvec,
    tvec,
    K,
    dist_coeffs,
)
print(f"Projected pixel: {pixels.reshape(-1, 2)}")

# Also project manually
pixel_manual = (K @ points_camera_method1.T).T
pixel_manual = pixel_manual / pixel_manual[:, 2:3]
print(f"Projected pixel (manual): {pixel_manual[:, :2]}")
print()

# Check if points are in front of camera
print(f"Camera Z coordinate (should be > 0): {points_camera_method1[0, 2]:.6f}")
