#!/bin/bash
# 完整的标定工作流脚本

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE=$SCRIPT_DIR
DATA_DIR=$WORKSPACE/calib_data

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}LiDAR-Camera 标定工作流${NC}"
echo -e "${YELLOW}========================================${NC}"

# 检查是否提供了操作参数
if [ $# -eq 0 ]; then
    echo -e "${YELLOW}可用命令:${NC}"
    echo "  ./calib_workflow.sh record      - 记录ROS2 bag数据"
    echo "  ./calib_workflow.sh extract     - 从bag中提取图像"
    echo "  ./calib_workflow.sh config      - 生成配置文件"
    echo "  ./calib_workflow.sh calibrate   - 运行标定程序"
    echo "  ./calib_workflow.sh all         - 执行全部流程"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  ./calib_workflow.sh record      # 记录10秒数据"
    echo "  ./calib_workflow.sh extract     # 提取第一张图像"
    echo "  ./calib_workflow.sh config      # 生成配置"
    echo "  ./calib_workflow.sh calibrate   # 运行标定"
    exit 1
fi

# 获取操作
OPERATION=$1

# 创建数据目录
mkdir -p $DATA_DIR/results

echo "工作目录: $DATA_DIR"

# ============================================================================
# 第一步: 记录数据
# ============================================================================

record_data() {
    echo -e "${GREEN}[1/4] 记录ROS2 Bag 数据${NC}"
    echo "将在 10 秒内记录:"
    echo "  - /livox/lidar (激光雷达点云)"
    echo "  - /camera/camera/color/image_raw (相机图像)"
    echo "  - /livox/imu (IMU数据)"
    echo ""
    echo -e "${YELLOW}提示: 你也可以按 Ctrl+C 手动停止${NC}"
    echo ""
    
    cd $DATA_DIR
    source $WORKSPACE/install/setup.bash
    ros2 bag record -o calib_bag \
        /livox/lidar \
        /camera/camera/color/image_raw \
        /livox/imu \
        -d 10
    
    if [ -d "calib_bag" ]; then
        echo -e "${GREEN}✓ Bag数据记录成功${NC}"
    else
        echo -e "${RED}✗ 记录失败${NC}"
        exit 1
    fi
}

# ============================================================================
# 第二步: 提取图像
# ============================================================================

extract_image() {
    echo -e "${GREEN}[2/4] 从Bag中提取图像${NC}"
    
    cd $DATA_DIR
    source $WORKSPACE/install/setup.bash
    
    python3 $WORKSPACE/extract_calib_image.py \
        calib_bag \
        /camera/camera/color/image_raw \
        results/image.png
    
    if [ -f "results/image.png" ]; then
        echo -e "${GREEN}✓ 图像提取成功${NC}"
    else
        echo -e "${RED}✗ 图像提取失败${NC}"
        exit 1
    fi
}

# ============================================================================
# 第三步: 生成配置
# ============================================================================

generate_config() {
    echo -e "${GREEN}[3/4] 生成标定配置文件${NC}"
    
    # 用户已经从 topic 获取相机参数，直接使用
    # 从这里手动填入你的相机参数
    echo -e "${YELLOW}使用从 /camera/camera/color/camera_info 获取的参数${NC}"
    echo ""
    
    cat > $DATA_DIR/calib_config.yaml << 'EOF'
lidar_camera_calib:
    ros__parameters:
        image_file: "/home/lzt/m20pro/calib_data/results/image.png"
        pcd_file: "/home/lzt/m20pro/calib_data/map.pcd"
        result_file: "/home/lzt/m20pro/calib_data/results/extrinsic.txt"
        
        # ✓ 直接用 ros2 topic echo 获取的参数
        camera_matrix: [915.6424560546875, 0.0, 636.0186157226562,
                        0.0, 915.1904907226562, 370.6102600097656,
                        0.0, 0.0, 1.0]
        dist_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
        
        calib_config_file: "/home/lzt/m20pro/src/lidar_camera_calib/config/config_indoor.yaml"
        use_rough_calib: true
        save_img: true
        folder: "/home/lzt/m20pro/calib_data/results"
EOF
    
    echo -e "${GREEN}✓ 配置文件已生成: $DATA_DIR/calib_config.yaml${NC}"
    echo ""
    echo -e "${YELLOW}注意: 如果相机参数不同，请编辑此文件:${NC}"
    echo "  nano $DATA_DIR/calib_config.yaml"
    echo ""
}

# ============================================================================
# 第四步: 运行标定
# ============================================================================

run_calibration() {
    echo -e "${GREEN}[4/4] 运行标定程序${NC}"
    
    cd $WORKSPACE
    source $WORKSPACE/install/setup.bash
    
    if [ ! -f "$DATA_DIR/calib_config.yaml" ]; then
        echo -e "${RED}✗ 找不到配置文件: $DATA_DIR/calib_config.yaml${NC}"
        exit 1
    fi
    
    if [ ! -f "$DATA_DIR/results/image.png" ]; then
        echo -e "${RED}✗ 找不到图像文件: $DATA_DIR/results/image.png${NC}"
        exit 1
    fi
    
    echo "启动标定程序..."
    ros2 launch lidar_camera_calib calib.launch.py \
        lidarcameracalib_param_dir:=$DATA_DIR/calib_config.yaml
    
    if [ -f "$DATA_DIR/results/extrinsic.txt" ]; then
        echo -e "${GREEN}✓ 标定完成!${NC}"
        echo "结果文件:"
        echo "  - 外参矩阵: $DATA_DIR/results/extrinsic.txt"
        echo "  - 可视化图像: $DATA_DIR/results/"
    fi
}

# ============================================================================
# 执行选择的操作
# ============================================================================

case $OPERATION in
    record)
        record_data
        ;;
    extract)
        extract_image
        ;;
    config)
        generate_config
        ;;
    calibrate)
        run_calibration
        ;;
    all)
        record_data
        extract_image
        generate_config
        run_calibration
        ;;
    *)
        echo -e "${RED}未知命令: $OPERATION${NC}"
        exit 1
        ;;
esac

echo -e "${GREEN}✓ 完成!${NC}"
