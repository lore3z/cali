#!/usr/bin/env python3
"""分析点云不对应的根本原因。"""

import numpy as np
from pathlib import Path

def load_extrinsic_matrix(path: Path) -> np.ndarray:
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        values = [float(v.strip()) for v in line.split(",") if v.strip()]
        rows.append(values)
    return np.asarray(rows, dtype=np.float64)

extrinsic = load_extrinsic_matrix(Path("/home/lzt/m20pro/calib_data/live_once/extrinsic.txt"))

print("=" * 70)
print("点云投影不对应的原因分析")
print("=" * 70)
print()

# 测试点
pt_lidar = np.array([[0.5, 0.0, 1.0, 1.0]])  # LiDAR 坐标系中的点

# 变换
pt_camera_h = (extrinsic @ pt_lidar.T).T
pt_camera = pt_camera_h[0, :3]

print(f"输入 LiDAR 点: X={pt_lidar[0,0]:.3f}, Y={pt_lidar[0,1]:.3f}, Z={pt_lidar[0,2]:.3f}")
print(f"输出相机点:  X={pt_camera[0]:.4f}, Y={pt_camera[1]:.4f}, Z={pt_camera[2]:.4f}")
print()

# 分析
print("问题分析:")
print("-" * 70)
print()

if pt_camera[1] < -0.5:
    print("⚠️  相机坐标的 Y 值非常负数！")
    print()
    print("这表明 LiDAR 的点被投影到了相机坐标系的")
    print("远下方（离相机光轴很远的地方）。")
    print()
    print("可能的原因:")
    print()
    print("1. 外参矩阵是反向的")
    print("   → 矩阵代表的是 Camera → LiDAR 而非 LiDAR → Camera")
    print("   → 需要反演: M_inv = [[R.T | -R.T @ T], [0 0 0 | 1]]")
    print()
    print("2. 或者相机坐标系的定义与标准不同")
    print("   → 例如 Y 轴指向不同的方向")
    print()
else:
    print(f"✓ 相机坐标的 Y 值合理: {pt_camera[1]:.4f}")
print()

# 尝试反演
print("=" * 70)
print("尝试反演外参矩阵...")
print("=" * 70)
print()

M_inv = np.eye(4, dtype=np.float64)
M_inv[:3, :3] = extrinsic[:3, :3].T
M_inv[:3, 3] = -extrinsic[:3, :3].T @ extrinsic[:3, 3]

pt_camera_inv_h = (M_inv @ pt_lidar.T).T
pt_camera_inv = pt_camera_inv_h[0, :3]

print(f"使用反演矩阵后: X={pt_camera_inv[0]:.4f}, Y={pt_camera_inv[1]:.4f}, Z={pt_camera_inv[2]:.4f}")
print()

if abs(pt_camera_inv[1]) < abs(pt_camera[1]):
    print("✓ 反演后 Y 值更合理！")
    print()
    print("**建议: 使用 pick_lidar_xy_debug.py --invert-extrinsic**")
else:
    print("✗ 反演后没有改善...")
    print()
    print("**建议: 检查相机内参和外参矩阵本身的准确性**")
