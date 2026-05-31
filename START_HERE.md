# 🎉 你已经有相机参数了，简化！

## 不需要额外查询参数 ✓

你已经有了：
```yaml
相机内参 (k):
  fx: 915.6424560546875
  fy: 915.1904907226562
  cx: 636.0186157226562
  cy: 370.6102600097656

畸变系数 (d): [0, 0, 0, 0, 0]
```

## 现在只需要 3 样东西：

1. **map.pcd** - 点云地图 (FAST-LIO 生成)
2. **image.png** - 一张相机图像 (从 bag 提取)
3. **calib_config.yaml** - 配置文件 (直接填你已有的参数)

## 🚀 一键标定流程

```bash
cd ~/m20pro
mkdir -p calib_data/results

# ===== 第1步：记录激光+相机数据 =====
# 终端1
source install/setup.bash
ros2 launch fast_lio mapping.launch.py

# 终端2
source install/setup.bash
ros2 bag record -o calib_data/calib_bag /livox/lidar /camera/camera/color/image_raw -d 20

# ===== 第2步：FAST-LIO处理（等待完成） =====
# FAST-LIO 会输出 map.pcd
# 复制到: calib_data/map.pcd

# ===== 第3步：提取图像 =====
python3 extract_calib_image.py \
  calib_data/calib_bag \
  /camera/camera/color/image_raw \
  calib_data/results/image.png

# ===== 第4步：创建配置 (一条命令) =====
cat > calib_data/calib_config.yaml << 'EOF'
lidar_camera_calib:
    ros__parameters:
        image_file: "/home/lzt/m20pro/calib_data/results/image.png"
        pcd_file: "/home/lzt/m20pro/calib_data/map.pcd"
        result_file: "/home/lzt/m20pro/calib_data/results/extrinsic.txt"
        camera_matrix: [915.6424560546875, 0.0, 636.0186157226562, 0.0, 915.1904907226562, 370.6102600097656, 0.0, 0.0, 1.0]
        dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
        calib_config_file: "/home/lzt/m20pro/src/lidar_camera_calib/config/config_indoor.yaml"
        use_rough_calib: true
        save_img: true
        folder: "/home/lzt/m20pro/calib_data/results"
EOF

# ===== 第5步：运行标定 =====
source install/setup.bash
ros2 launch lidar_camera_calib calib.launch.py \
  lidarcameracalib_param_dir:=$(pwd)/calib_data/calib_config.yaml

# ===== 第6步：查看结果 =====
cat calib_data/results/extrinsic.txt
```

## ✨ 就这么简单

相机参数已有 ✓
→ 记录激光+相机数据
→ 用FAST-LIO生成PCD
→ 提取一张图像
→ 运行标定
→ 完成！

不需要任何额外的查询和转换。

