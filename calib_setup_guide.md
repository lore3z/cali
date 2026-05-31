# LiDAR-Camera 标定完整指南

## 环境
- **项目位置**: `src/lidar_camera_calib`
- **已编译**: ✅
- **ROS2 版本**: Humble
- **运行中的话题**:
  - `/livox/lidar` - 激光雷达点云
  - `/camera/camera/color/image_raw` - 相机RGB图像

## 整个流程概览

这是一个离线标定工具，需要以下步骤：

```
1. 记录ROS2 bag (同时包含雷达+相机数据)
   ↓
2. 从点云生成PCD地图文件 (FAST-LIO已安装可用)
   ↓
3. 从bag中提取第一张相机图像
   ↓
4. 获取相机的内参和畸变系数
   ↓
5. 修改 calib.yaml 配置文件
   ↓
6. 运行标定程序
   ↓
7. 获得最终的外参矩阵
```

## 详细步骤

### 步骤 1: 记录ROS2 Bag

打开一个终端，运行：

```bash
cd ~/m20pro
mkdir -p calib_data
cd calib_data

# 记录数据（假设10秒，可按需调整）
ros2 bag record -o calib_bag /livox/lidar /camera/camera/color/image_raw /livox/imu -d 10
```

或者使用鼠标运动方式记录（推荐）：
```bash
ros2 bag record -o calib_bag /livox/lidar /camera/camera/color/image_raw /livox/imu
# 移动设备大约10-20秒，然后 Ctrl+C 停止
```

**推荐的运动方式**:
- 侧向平移运动（lateral translation）
- 或俯仰运动（pitching motion）

### 步骤 2: 从FAST-LIO生成PCD地图

FAST-LIO已经编译，可以直接使用。创建 `run_mapping.launch.py`：

```bash
# 在 calib_data 目录下运行
cd ~/m20pro/calib_data

# 回放bag文件并用FAST-LIO生成地图
ros2 bag play calib_bag

# 在另一个终端启动FAST-LIO
source ~/m20pro/install/setup.bash
ros2 launch fast_lio mapping.launch.py

# 等待FAST-LIO完成处理...
```

FAST-LIO会生成 `/Odometry` 话题和地图。最后会输出PCD文件。

### 步骤 3: 提取相机图像

使用提供的脚本提取第一张图像：

```bash
cd ~/m20pro
source install/setup.bash
python3 src/lidar_camera_calib/scripts/extract_image_from_bag.py \
  --bag-path calib_data/calib_bag \
  --image-topic /camera/camera/color/image_raw \
  --output-path calib_data/image.png
```

或使用简单的方法（如果上面的脚本不工作）：

```bash
# 使用ROS2工具从bag中提取
cd calib_data
ros2 bag play calib_bag --start-paused
# 然后手动订阅话题并保存图像
```

### 步骤 4: 获取相机内参

你的相机参数应该已经在ROS2驱动中。查询：

```bash
ros2 topic echo /camera/camera/color/camera_info
```

会输出类似：
```
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: "plumb_bob"
d: [k1, k2, p1, p2, k3]
```

记下这些参数！

### 步骤 5: 修改标定配置文件

编辑 `src/lidar_camera_calib/config/calib.yaml`：

```yaml
lidar_camera_calib:
    ros__parameters:
        # 修改这些路径为你的数据路径
        image_file: "/home/lzt/m20pro/calib_data/image.png"
        pcd_file: "/home/lzt/m20pro/calib_data/map.pcd"
        result_file: "/home/lzt/m20pro/calib_data/extrinsic.txt"

        # 从 /camera/camera/color/camera_info 获取的参数
        camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        dist_coeffs: [k1, k2, p1, p2, k3]

        # 标定参数
        calib_config_file: "/home/lzt/m20pro/src/lidar_camera_calib/config/config_indoor.yaml"
        use_rough_calib: true  # 如果初始外参很差，设置为true
        save_img: true
        folder: "/home/lzt/m20pro/calib_data/results"
```

### 步骤 6: 运行标定程序

```bash
cd ~/m20pro
source install/setup.bash

# 创建输出目录
mkdir -p calib_data/results

# 运行标定
ros2 launch lidar_camera_calib calib.launch.py
```

### 步骤 7: 获取结果

标定完成后，结果会保存在：
- `calib_data/extrinsic.txt` - 外参矩阵 (4x4)
- `calib_data/results/` - 可视化结果

## 可选配置调整

编辑 `config/config_indoor.yaml` 或 `config/config_outdoor.yaml`：

```yaml
# 根据你的场景调整这些参数
Canny.gray_threshold: 10        # Canny边缘检测阈值
Voxel.size: 1.0                 # 体素大小
Plane.min_points_size: 60       # 最小平面点数
Ransac.dis_threshold: 0.02      # RANSAC距离阈值
```

## 常见问题

### Q: 怎样判断标定结果好不好？

A: 查看 `calib_data/results/` 中的可视化图像，看点云投影到相机图像上是否对齐。

### Q: use_rough_calib 什么时候设置为 true？

A: 当你的初始外参（config_indoor.yaml中的ExtrinsicMat）与实际相差很大时。

### Q: 需要多少数据才能进行标定？

A: 最少10秒左右的运动数据。更多数据会得到更好的标定结果。

### Q: 可以用实时数据标定吗？

A: 不行。这个工具是离线的，需要完整的PCD文件和静态图像。

## 下一步

标定成功后，获得的外参矩阵可以用于：
1. 更新 `config/config_indoor.yaml` 中的 `ExtrinsicMat`
2. 在RVIZ中显示对齐的点云
3. 用于其他需要雷达-相机标定的应用
