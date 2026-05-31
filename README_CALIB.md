# 🎯 lidar_camera_calib 设置完成总结

## ✅ 已完成

### 1. 项目部署
- ✓ 克隆 GitHub 仓库到 `src/lidar_camera_calib`
- ✓ 成功编译项目（解决了 backward_ros 依赖）
- ✓ 所有二进制文件已构建

### 2. 辅助工具创建
以下文件已在 `~/m20pro/` 中创建：

| 文件 | 功能 |
|------|------|
| `QUICKSTART.md` | 📖 快速开始指南 |
| `calib_setup_guide.md` | 📚 详细设置文档 |
| `calib_workflow.sh` | 🤖 完整自动化工作流 |
| `extract_calib_image.py` | 🖼️ 从ROS2 bag提取图像 |
| `generate_calib_config.py` | ⚙️ 生成配置文件助手 |

### 3. 你的硬件配置

**运行中的话题：**
```
激光雷达:
  /livox/lidar          - 点云数据
  /livox/imu            - IMU数据
  /Laser_map            - 地图数据
  /Odometry             - 里程计

相机:
  /camera/camera/color/image_raw              - RGB图像
  /camera/camera/color/camera_info            - 相机内参
  /camera/camera/depth/image_rect_raw         - 深度图像
  /camera/camera/depth/camera_info            - 深度相机内参
```

## 🚀 接下来的步骤

### 最快开始（推荐）

```bash
cd ~/m20pro

# 1. 记录数据
./calib_workflow.sh record

# 2. 提取图像
./calib_workflow.sh extract

# 3. 生成配置
./calib_workflow.sh config

# 4. 查询相机参数（在另一个终端）
source install/setup.bash
ros2 topic echo /camera/camera/color/camera_info

# 5. 编辑配置文件，填入相机参数
nano calib_data/calib_config.yaml

# 6. 运行标定
./calib_workflow.sh calibrate
```

### 需要注意的事项

1. **重要！** 在运行标定前，你需要：
   - 获取相机的**内参矩阵** (K matrix)
   - 获取相机的**畸变系数** (distortion coefficients)
   
   这些参数在 `/camera/camera/color/camera_info` 话题中：
   ```bash
   ros2 topic echo /camera/camera/color/camera_info
   ```

2. **获取PCD文件** - 有两种方式：
   
   **方式A: 使用FAST-LIO（你已经安装）**
   ```bash
   # 终端1: 启动FAST-LIO
   source ~/m20pro/install/setup.bash
   ros2 launch fast_lio mapping.launch.py
   
   # 终端2: 回放bag
   ros2 bag play ~/m20pro/calib_data/calib_bag
   
   # 等待完成，FAST-LIO会输出 map.pcd
   ```
   
   **方式B: 从FAST-LIO现有输出**
   - 如果FAST-LIO已经生成过地图，check: `build/fast_lio/`

3. **标定配置文件位置**：
   ```bash
   ~/m20pro/calib_data/calib_config.yaml
   ```
   需要修改的字段：
   - `image_file` - 提取的图像路径
   - `pcd_file` - 点云地图路径
   - `camera_matrix` - 相机内参
   - `dist_coeffs` - 畸变系数

## 📊 工作流概览

```
数据录制
  ↓ (ROS2 bag)
图像提取 + PCD文件
  ↓
配置标定参数
  ↓
运行标定程序 (Ceres优化)
  ↓
输出外参矩阵 (4x4) 
  ↓
可视化结果验证
```

## 📁 文件结构

```
~/m20pro/
├── README.md (本文件)
├── QUICKSTART.md                  ← 快速开始
├── calib_setup_guide.md          ← 详细指南
├── calib_workflow.sh             ← 自动脚本
├── extract_calib_image.py        ← 辅助工具
├── generate_calib_config.py      ← 辅助工具
├── src/
│   └── lidar_camera_calib/      ← 标定工具源码
├── install/setup.bash           ← ROS2环境
└── calib_data/                  ← 你的数据目录（需要创建）
    ├── calib_bag/               ← ROS2 bag文件
    ├── map.pcd                  ← 点云地图
    ├── calib_config.yaml        ← 标定配置
    └── results/
        ├── image.png            ← 提取的图像
        └── extrinsic.txt        ← 标定结果
```

## 🔍 验证安装

```bash
# 检查编译是否成功
cd ~/m20pro
source install/setup.bash
ros2 pkg list | grep lidar_camera_calib
# 应该显示: lidar_camera_calib

# 检查可执行文件
which lidar_camera_calib
# 应该显示: ~/m20pro/install/lidar_camera_calib/bin/lidar_camera_calib

# 列出所有辅助脚本
ls -lh ~/m20pro/*.{sh,py,md}
```

## 💡 关键参数说明

### 相机矩阵 (Camera Matrix)
```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]

fx, fy: 焦距（像素为单位）
cx, cy: 主点（图像中心）
```

### 畸变系数 (Distortion Coefficients)
```
[k1, k2, p1, p2, k3]

k1, k2: 径向畸变系数
p1, p2: 切向畸变系数
k3: 第三径向畸变系数
```

### 初始外参 (Initial Extrinsic)
```yaml
ExtrinsicMat: !!opencv-matrix
  rows: 4
  cols: 4
  dt: d
  data: [R00, R01, R02, Tx,
         R10, R11, R12, Ty,
         R20, R21, R22, Tz,
         0,   0,   0,   1]

R: 3x3旋转矩阵
T: 3x1平移向量
```

## 🎓 算法流程

标定工具使用的是基于Ceres求解器的非线性优化：

1. **边缘提取** - 从图像和点云中提取边缘
2. **特征匹配** - 匹配图像边缘和点云投影
3. **初始化** - 粗标定求得初始外参
4. **优化** - Ceres求解器精细化外参

## ⚠️ 常见问题

**Q: 需要多长的数据？**
A: 最少10秒，推荐20-30秒。数据越多，结果越稳定。

**Q: 什么样的运动最好？**
A: 侧向平移 + 俯仰运动混合。避免纯旋转。

**Q: 标定需要多久？**
A: 通常5-10分钟，取决于数据量和参数。

**Q: 标定精度如何评估？**
A: 查看可视化结果，点云投影到相机图像上是否对齐。

## 📞 获取帮助

- 查看详细指南：`calib_setup_guide.md`
- 查看快速开始：`QUICKSTART.md`
- 原项目仓库：https://github.com/simi-asher/lidar_camera_calib
- 相关工具：https://github.com/rsasaki0109/lidarslam_ros2

## ✨ 下一步建议

1. **第一次运行**
   - 使用 `./calib_workflow.sh record` 采集10秒数据
   - 用FAST-LIO生成PCD
   - 按照快速开始指南完成标定

2. **调试优化**
   - 如果结果不好，增加数据量再试一次
   - 调整 `config_indoor.yaml` 中的参数
   - 尝试不同的运动模式

3. **生产使用**
   - 一旦标定成功，保存外参矩阵
   - 在其他应用中使用这个外参
   - 定期重新标定以维持精度

---

**祝你标定成功！** 🎉

如有任何问题，请参考详细文档或查看项目GitHub页面。
