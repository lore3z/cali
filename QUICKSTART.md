# 📋 快速开始指南 (真正简化版)

## ✅ 当前状态
- ✓ 项目已克隆到 `src/lidar_camera_calib`
- ✓ 项目已编译成功
- ✓ **相机参数已获取** ✓
- ✓ 你的LiDAR和相机驱动正在运行

## ✨ 核心问题解答

**Q: 为什么一定要录制 bag？直接用 topic 信息不行吗？**

A: 可以！你已经有了相机参数，不需要从 bag 再次提取。

录制 bag 的**真正目的**只有两个：
1. 用 FAST-LIO 生成 **PCD 点云地图**（这个不能从单个 topic 获得）
2. 提取**一张代表性的相机图像**

## 🚀 实际流程（简化到最少）

### 第1步：记录20秒激光数据
```bash
cd ~/m20pro
mkdir -p calib_data/results

# 终端1: 启动FAST-LIO
source install/setup.bash
ros2 launch fast_lio mapping.launch.py

# 终端2: 记录数据（20秒）
source install/setup.bash
ros2 bag record -o calib_data/calib_bag \
  /livox/lidar \
  /camera/camera/color/image_raw \
  -d 20
```

### 第2步：等待FAST-LIO生成map.pcd
```bash
# FAST-LIO会处理点云数据
# 完成后会输出 map.pcd
# 将其复制到 calib_data/map.pcd
```

### 第3步：提取一张图像
```bash
python3 extract_calib_image.py \
  calib_data/calib_bag \
  /camera/camera/color/image_raw \
  calib_data/results/image.png
```

### 第4步：用你已获取的相机参数创建配置
```bash
cat > calib_data/calib_config.yaml << 'EOF'
lidar_camera_calib:
    ros__parameters:
        image_file: "/home/lzt/m20pro/calib_data/results/image.png"
        pcd_file: "/home/lzt/m20pro/calib_data/map.pcd"
        result_file: "/home/lzt/m20pro/calib_data/results/extrinsic.txt"
        
        # ✓ 直接用你从 topic echo 获取的参数
        camera_matrix: [915.6424560546875, 0.0, 636.0186157226562,
                        0.0, 915.1904907226562, 370.6102600097656,
                        0.0, 0.0, 1.0]
        dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
        
        calib_config_file: "/home/lzt/m20pro/src/lidar_camera_calib/config/config_indoor.yaml"
        use_rough_calib: true
        save_img: true
        folder: "/home/lzt/m20pro/calib_data/results"
EOF
```

### 第5步：运行标定
```bash
source install/setup.bash
ros2 launch lidar_camera_calib calib.launch.py \
  lidarcameracalib_param_dir:=$(pwd)/calib_data/calib_config.yaml
```

## 📊 就这么简单

```
你已有: 相机参数 ✓
需要生成:
  1. map.pcd (用FAST-LIO从bag生成)
  2. image.png (从bag提取)
  ↓
直接运行标定
  ↓
得到 extrinsic.txt
```

## 🔧 你的相机参数

从 `ros2 topic echo /camera/camera/color/camera_info` 获取：

```
Camera Matrix (K):
  fx: 915.6424560546875
  fy: 915.1904907226562
  cx: 636.0186157226562
  cy: 370.6102600097656

Distortion (d): [0, 0, 0, 0, 0]
```

这就是标定工具需要的全部。✓

## ⚠️ 常见问题

**Q: 每次都要查询相机参数吗？**
A: 不用。参数不变的话，直接用上面的值。

**Q: PCD文件怎么获得？**
A: 用FAST-LIO处理你录制的bag，会自动输出map.pcd

**Q: 图像怎么提取？**
A: 用 `extract_calib_image.py` 脚本从bag中提取第一张

---

**关键要点**: 相机参数 ✓，PCD 和图像用脚本从 bag 生成。完了。

## 📊 文件位置参考

```
~/m20pro/
├── calib_data/                    # 你的数据目录
│   ├── calib_bag/                 # 记录的ROS2 bag
│   ├── map.pcd                    # FAST-LIO生成的点云
│   ├── calib_config.yaml          # 标定配置
│   └── results/
│       ├── image.png              # 提取的图像
│       ├── extrinsic.txt          # 标定结果
│       └── ...                    # 可视化结果
├── src/
│   └── lidar_camera_calib/        # 标定工具源码
├── install/setup.bash             # ROS2环境
├── calib_workflow.sh              # 自动化脚本
├── extract_calib_image.py         # 提取图像脚本
└── generate_calib_config.py       # 生成配置脚本
```

## ⚙️ 常见参数调整

编辑 `src/lidar_camera_calib/config/config_indoor.yaml`：

```yaml
# 边缘检测参数
Canny.gray_threshold: 10      # 越小越敏感，推荐 5-20
Canny.len_threshold: 200      # 最小边缘长度

# 点云处理参数
Voxel.size: 1.0               # 体素网格大小
Voxel.down_sample_size: 0.02  # 下采样大小

# 平面检测参数
Plane.min_points_size: 60     # 最小平面点数
Plane.normal_theta_min: 45    # 法向量最小角度
Plane.normal_theta_max: 135   # 法向量最大角度

# RANSAC参数
Ransac.dis_threshold: 0.02    # 距离阈值

# 边缘检测参数
Edge.min_dis_threshold: 0.03
Edge.max_dis_threshold: 0.06
```

## 🐛 故障排除

### 问题：找不到相机参数话题
```bash
# 确认相机驱动在运行
ros2 topic list | grep camera

# 应该看到:
# /camera/camera/color/camera_info
# /camera/camera/color/image_raw
```

### 问题：FAST-LIO没有输出map.pcd
```bash
# 检查FAST-LIO是否正在发布话题
ros2 topic echo /Odometry

# 检查日志
ros2 run fast_lio mapping.launch.py
```

### 问题：标定结果不好
1. 检查数据质量 - 运动必须包含平移和旋转
2. 增加数据量 - 记录20-30秒而不是10秒
3. 调整参数 - 编辑config_indoor.yaml中的阈值
4. 检查初始外参 - 设置use_rough_calib: true

### 问题：提取图像失败
```bash
# 检查bag中有哪些话题
ros2 bag info calib_bag

# 查看图像话题名称是否正确
```

## 📚 更多信息

- 详细指南：见 `calib_setup_guide.md`
- 原项目：https://github.com/simi-asher/lidar_camera_calib
- 相关工具：https://github.com/rsasaki0109/lidarslam_ros2

## 💡 提示

1. **最佳运动方式**：侧向平移 + 俯仰运动混合
2. **数据质量**：场景需要有清晰的边缘（墙、物体边界）
3. **时间成本**：标定通常需要5-10分钟
4. **重复标定**：建议多次验证，取最好结果

---

有问题？查看详细指南：`calib_setup_guide.md`
