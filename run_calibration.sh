#!/bin/bash
# 完整的标定流程：一键从标定到结果

set -e

WORK_DIR="/home/lzt/m20pro"
DATA_DIR="$WORK_DIR/calib_data"

echo "========================================="
echo "激光雷达-相机 自动标定流程"
echo "========================================="
echo ""

# 第1步：确保目录存在
mkdir -p "$DATA_DIR/results"
cd "$WORK_DIR"
source install/setup.bash

# 第2步：检查是否需要保存数据
if [ ! -f "$DATA_DIR/results/map.pcd" ] || [ ! -f "$DATA_DIR/results/image.png" ]; then
    echo "[步骤1] 保存标定数据..."
    echo "前提：FAST-LIO 应该已在运行"
    echo ""
    python3 save_calib_data.py "$DATA_DIR/results"
    echo ""
else
    echo "[跳过] 标定数据已存在"
    echo "  - map.pcd: $(wc -c < $DATA_DIR/results/map.pcd | numfmt --to=iec 2>/dev/null || echo 'N/A')"
    echo "  - image.png: 存在"
    echo ""
fi

# 第3步：创建标定配置文件
echo "[步骤2] 创建标定配置..."
cat > "$DATA_DIR/calib_config.yaml" << 'EOF'
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
echo "✓ 配置文件已创建"
echo ""

# 第4步：运行标定
echo "[步骤3] 运行标定程序..."
echo "配置文件: $DATA_DIR/calib_config.yaml"
echo ""

ros2 launch lidar_camera_calib calib.launch.py \
  lidarcameracalib_param_dir:="$DATA_DIR/calib_config.yaml"

# 第5步：检查结果
echo ""
echo "========================================="
echo "✅ 标定完成！"
echo "========================================="
echo ""
echo "结果文件位置: $DATA_DIR/results/"
echo ""

if [ -f "$DATA_DIR/results/extrinsic.txt" ]; then
    echo "外参矩阵:"
    cat "$DATA_DIR/results/extrinsic.txt"
else
    echo "⚠️  extrinsic.txt 未找到"
fi

echo ""
echo "其他输出文件:"
ls -lh "$DATA_DIR/results/" | grep -v "^total" | awk '{print "  " $9 " (" $5 ")"}'
echo ""
