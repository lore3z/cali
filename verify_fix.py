#!/usr/bin/env python3
"""验证投影修复 - 测试坐标变换是否正确。"""

import numpy as np
import cv2
from pathlib import Path

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


# 加载外参
extrinsic = load_extrinsic_matrix(Path("/home/lzt/m20pro/calib_data/live_once/extrinsic.txt"))
print("外参矩阵 M:")
print(extrinsic)
print()

# 相机内参（RealSense D435i）
fx, fy = 918.3, 917.6
cx, cy = 640.0, 359.6
K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
print("相机矩阵 K:")
print(K)
print()

# 创建测试点
test_points_lidar = np.array([
    [0.5, 0.0, 1.0],    # 50cm前，1m上
    [0.3, 0.2, 0.8],    # 30cm前，20cm左，80cm上
    [0.7, -0.1, 1.2],   # 70cm前，10cm右，1.2m上
], dtype=np.float64)

print(f"测试 LiDAR 点 ({len(test_points_lidar)} 个):")
for i, pt in enumerate(test_points_lidar):
    print(f"  点 {i}: {pt}")
print()

# === 旧方法（错误的）===
print("=" * 60)
print("❌ 旧方法（使用 cv2.projectPoints 会双重变换）")
print("=" * 60)

rvec, _ = cv2.Rodrigues(extrinsic[:3, :3])
tvec = extrinsic[:3, 3].reshape(3, 1)
dist_coeffs = np.zeros((1, 5), dtype=np.float64)

pixels_old, _ = cv2.projectPoints(
    test_points_lidar.reshape(-1, 1, 3).astype(np.float64),
    rvec,
    tvec,
    K,
    dist_coeffs,
)
pixels_old = pixels_old.reshape(-1, 2)

for i, (pt_lidar, px) in enumerate(zip(test_points_lidar, pixels_old)):
    print(f"点 {i} LiDAR {pt_lidar} → 像素 {px}")

print()

# === 新方法（正确的）===
print("=" * 60)
print("✓ 新方法（直接用相机坐标投影）")
print("=" * 60)

# 第1步：变换到相机坐标系
ones = np.ones((test_points_lidar.shape[0], 1), dtype=np.float64)
points_h = np.hstack([test_points_lidar, ones])
points_camera = (extrinsic @ points_h.T).T[:, :3]

print("相机坐标系中的点:")
for i, pt_cam in enumerate(points_camera):
    print(f"  点 {i}: {pt_cam}")
print()

# 第2步：过滤在相机前方的点
valid_mask = points_camera[:, 2] > 0.05
print(f"在相机前方的点: {np.sum(valid_mask)} / {len(points_camera)}")
print()

points_camera_valid = points_camera[valid_mask]

# 第3步：直接投影（不使用 cv2.projectPoints 的 R、T）
pixels_homogeneous = (K @ points_camera_valid.T).T
pixels_new = (pixels_homogeneous[:, :2] / pixels_homogeneous[:, 2:3]).astype(np.float64)

print("投影后的像素:")
for i, (pt_cam, px) in enumerate(zip(points_camera_valid, pixels_new)):
    in_bounds = 0 <= px[0] < 1280 and 0 <= px[1] < 720
    status = "✓ 在图像内" if in_bounds else "✗ 超出范围"
    print(f"点 {i} 相机 {pt_cam} → 像素 {px} {status}")

print()

# === 比较 ===
print("=" * 60)
print("比较结果")
print("=" * 60)

# 检查新方法是否产生合理的像素坐标
in_bounds_count = 0
for px in pixels_new:
    if 0 <= px[0] < 1280 and 0 <= px[1] < 720:
        in_bounds_count += 1

print(f"✓ 新方法: {in_bounds_count}/{len(pixels_new)} 点在图像内")
print()

# 检查旧方法
old_in_bounds = 0
for px in pixels_old:
    if 0 <= px[0] < 1280 and 0 <= px[1] < 720:
        old_in_bounds += 1
print(f"❌ 旧方法: {old_in_bounds}/{len(pixels_old)} 点在图像内")
print()

if in_bounds_count > old_in_bounds:
    print("✓ 新方法投影更正确！修复成功！")
elif in_bounds_count < old_in_bounds:
    print("⚠️  新方法投影的点更少。可能需要检查相机内参或外参方向。")
else:
    print("⚠️  两个方法结果相同。需要检查其他问题。")
