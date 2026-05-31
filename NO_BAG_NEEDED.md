# 🎯 正确的流程：直接用 /Laser_map（不需要 bag）

你说得完全对！`/Laser_map` 就是 FAST-LIO 实时发布的完整全局地图。

## ✨ 新的超简单流程

```bash
cd ~/m20pro
mkdir -p calib_data/results
source install/setup.bash

# ========== 终端1：启动FAST-LIO ==========
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

# ========== 终端2：保存地图和图像 ==========
# 等待FAST-LIO 初始化完成（~5-10秒）
python3 save_calib_data.py calib_data/results

# 这会：
# 1. 订阅 /Laser_map → 保存为 map.pcd ✓
# 2. 订阅 /camera/camera/color/image_raw → 保存为 image.png ✓
# 3. 自动完成退出 

# ========== 终端3：运行标定 ==========
cat > calib_data/calib_config.yaml << 'EOF'
lidar_camera_calib:
    ros__parameters:
        image_file: "/home/lzt/m20pro/calib_data/results/image.png"
        pcd_file: "/home/lzt/m20pro/calib_data/results/map.pcd"
        result_file: "/home/lzt/m20pro/calib_data/results/extrinsic.txt"
        camera_matrix: [915.6424560546875, 0.0, 636.0186157226562, 0.0, 915.1904907226562, 370.6102600097656, 0.0, 0.0, 1.0]
        dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
        calib_config_file: "/home/lzt/m20pro/src/lidar_camera_calib/config/config_indoor.yaml"
        use_rough_calib: true
        save_img: true
        folder: "/home/lzt/m20pro/calib_data/results"
EOF

ros2 launch lidar_camera_calib calib.launch.py \
  lidarcameracalib_param_dir:=$(pwd)/calib_data/calib_config.yaml
```

## 📊 流程对比

| 方式 | 步骤 | 优点 | 缺点 |
|------|------|------|------|
| 旧方法（bag） | 录制 → 回放 → 提取 → 标定 | 数据持久化 | 麻烦，要多个终端 |
| **新方法** | **启动FAST-LIO → 直接保存 → 标定** | **简单！实时！** | **依赖FAST-LIO已运行** |

## 🎯 关键点

1. **`/Laser_map` 就是完整地图** - FAST-LIO 在实时累积
2. **不需要录制 bag** - 直接从 topic 保存
3. **自动时间对齐** - 同一个 ROS2 时刻的数据
4. **超快速** - 不用等待 bag 回放

## ✅ 验证已就绪

```bash
# 检查 /Laser_map 是否在发布
ros2 topic list | grep -i map
# 应该显示: /Laser_map

# 检查相机
ros2 topic list | grep -i camera
# 应该显示: /camera/camera/color/image_raw
```

## 🚀 完整命令（复制粘贴）

```bash
cd ~/m20pro && \
mkdir -p calib_data/results && \
source install/setup.bash && \
echo "在终端1运行: ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml" && \
echo "然后在终端2运行: python3 save_calib_data.py calib_data/results"
```

就这么简单！🎉
