# LiDAR 点云投影问题诊断与修复

## 问题总结

`pick_lidar_xy.py` 点云投影工具出现以下问题：
1. ✗ 点云点无法显示或不更新  
2. ✗ 显示的点与实际图像不对应

## 根本原因分析

### 问题 1: 相机参数缺失（已修复）

**症状**: 程序启动后卡住，等待 `/camera/camera/color/camera_info` 话题
**原因**: RealSense 驱动未发布相机参数
**解决**: 修改了 `pick_lidar_xy.py` 增加 5 秒超时，若无法获取则使用 RealSense D435i 默认参数
  - fx = 918.3, fy = 917.6
  - cx = 640.0, cy = 359.6

### 问题 2: 外参矩阵方向可能错误（待测试）

**症状**: 投影的像素坐标在图像范围外（如 Y = -942）
**原因**: 标定算法输出的外参矩阵可能采用了不同的坐标系约定  
**可能原因**:
- 矩阵代表 Camera → LiDAR 转换（而非 LiDAR → Camera）
- 或相机帧的坐标系定义不匹配

## 文件修改

### 1. 已修复: `pick_lidar_xy.py`

主要改进：
```python
# 增加 5 秒超时机制
self.fallback_timer = self.create_timer(5.0, self._use_fallback_intrinsics)

# 超时后使用默认参数
def _use_fallback_intrinsics(self) -> None:
    """若相机信息没有到达，使用 RealSense D435i 默认参数"""
    fx, fy = 918.3, 917.6
    cx, cy = 640.0, 359.6
    # ...
```

### 2. 新文件: `pick_lidar_xy_debug.py`

支持外参矩阵反演的版本（用于测试）

```bash
# 原始方向
python3 pick_lidar_xy_debug.py

# 反演方向（若原始不工作）
python3 pick_lidar_xy_debug.py --invert-extrinsic
```

### 3. 新文件: `verify_extrinsic_direction.py`

自动测试哪个外参矩阵方向正确

```bash
python3 verify_extrinsic_direction.py
```

输出示例:
```
✓ Use ORIGINAL extrinsic matrix
  Command: python3 pick_lidar_xy.py
```

或
```
✓ Use INVERTED extrinsic matrix  
  Command: python3 pick_lidar_xy_debug.py --invert-extrinsic
```

## 使用步骤

### Step 1: 测试外参矩阵方向（必需）

```bash
cd /home/lzt/m20pro
python3 verify_extrinsic_direction.py
```

等待输出，它会告诉你哪个版本是正确的。

### Step 2: 根据测试结果选择工具

#### Case A: 原始矩阵正确 ✓
```bash
python3 pick_lidar_xy.py
```

#### Case B: 需要反演矩阵
```bash
python3 pick_lidar_xy_debug.py --invert-extrinsic
```

然后将 `pick_lidar_xy_debug.py` 复制为 `pick_lidar_xy_correct.py` 以备后用。

### Step 3: 若仍不工作

#### 原因 3.1: 相机内参错误
如果仍无点显示（或显示位置完全错误），尝试手动指定相机参数：

```bash
# 假设已知 fx=640.7, fy=640.1, cx=640, cy=360
python3 pick_lidar_xy.py --fx 640.7 --fy 640.1 --cx 640 --cy 360
```

#### 原因 3.2: 标定结果本身错误
若所有尝试都不工作，需要重新运行标定算法：

```bash
cd /home/lzt/m20pro

# 确保有富有 3D 几何特征的场景（不要平面）
python3 live_calibrate_once.py \
  --cloud-topic /livox/lidar \
  --image-topic /camera/camera/color/image_raw \
  --accumulate-frames 50 \
  --output-dir ./calib_data/live_once_new

# 新的 extrinsic.txt 会保存到 calib_data/live_once_new/
```

## 技术细节

### 坐标系约定

LiDAR 坐标系 (Livox MID360):
- X: 前向
- Y: 左向  
- Z: 上向

相机坐标系 (RealSense D435i):
- X: 右向
- Y: 下向
- Z: 前向（光轴方向）

外参矩阵 **M** 应该满足：
```
P_camera = M @ P_lidar_homogeneous
```

其中 P_camera.z > 0 才能投影到图像上。

### 投影公式

```python
# 1. 变换到相机坐标系
P_cam = M[0:3, 0:3] @ P_lidar + M[0:3, 3]

# 2. 检查是否在相机前方
if P_cam[2] <= 0:
    point_is_behind_camera()

# 3. 利用相机内参投影
P_pixel = K @ P_cam / P_cam[2]
```

## 常见错误

| 现象 | 原因 | 解决方案 |
|------|------|---------|
| 无点显示 | 相机参数缺失或矩阵方向错 | 运行 `verify_extrinsic_direction.py` |
| 点完全错位（Y 为负） | 外参矩阵方向错 | 试试 `--invert-extrinsic` 标志 |
| 部分点显示但位置不对 | 相机内参不准 | 检查/重新标定相机 |
| 所有点都在图像边缘 | 相机内参的主点 (cx, cy) 错误 | 手动指定 `--cx --cy` |

## 下一步

1. **立即**: 运行 `verify_extrinsic_direction.py` 确定正确的矩阵方向
2. **验证**: 用 `pick_lidar_xy.py` 或 `pick_lidar_xy_debug.py --invert-extrinsic` 测试  
3. **若仍不工作**: 收集更多诊断信息（见下方）

## 诊断信息收集

若点云仍不显示，请收集以下信息：

```bash
# 1. 检查 LiDAR 点云质量
ros2 topic hz /livox/lidar

# 2. 检查相机图像质量  
ros2 topic hz /camera/camera/color/image_raw

# 3. 检查外参矩阵是否被正确加载
cd /home/lzt/m20pro
python3 -c "
from pathlib import Path
import numpy as np

M = np.loadtxt(Path('calib_data/live_once/extrinsic.txt'), delimiter=',')
print('Extrinsic Matrix:')
print(M)
print()
print('det(R) =', np.linalg.det(M[:3, :3]))  # 应该 ≈ 1.0
print('Translation:', M[:3, 3])
"
```

## 参考

- 标定配置: `src/lidar_camera_calib/config/config_indoor.yaml`
- 标定结果: `calib_data/live_once/extrinsic.txt`
- 标定可视化: `calib_data/live_once/result.png`
