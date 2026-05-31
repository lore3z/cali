#!/usr/bin/env python3
"""
辅助脚本：查询相机参数并生成标定配置文件模板
使用方法: python3 generate_calib_config.py --camera-info-topic <topic> --output <yaml_file>
"""

import sys
import argparse
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoListener(Node):
    def __init__(self, camera_info_topic):
        super().__init__('camera_info_listener')
        self.subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            1)
        self.camera_info = None
        
    def camera_info_callback(self, msg: CameraInfo):
        """接收相机信息"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info('Received camera info!')


def get_camera_info(camera_info_topic, timeout=5):
    """从ROS2话题获取相机参数"""
    
    rclpy.init()
    listener = CameraInfoListener(camera_info_topic)
    
    # 等待接收消息
    start_time = listener.get_clock().now()
    while listener.camera_info is None:
        rclpy.spin_once(listener, timeout_sec=0.1)
        elapsed = (listener.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout:
            print(f"ERROR: Timeout waiting for camera info on '{camera_info_topic}'")
            rclpy.shutdown()
            return None
    
    camera_info = listener.camera_info
    rclpy.shutdown()
    
    return {
        'frame_id': camera_info.header.frame_id,
        'width': camera_info.width,
        'height': camera_info.height,
        'camera_matrix': list(camera_info.k),
        'dist_coeffs': list(camera_info.d),
    }


def generate_config(camera_info, bag_path, pcd_path, result_dir, output_file):
    """生成标定配置文件"""
    
    config = {
        'lidar_camera_calib': {
            'ros__parameters': {
                'image_file': f'{result_dir}/image.png',
                'pcd_file': pcd_path,
                'result_file': f'{result_dir}/extrinsic.txt',
                'camera_matrix': camera_info['camera_matrix'],
                'dist_coeffs': camera_info['dist_coeffs'],
                'calib_config_file': 'config/config_indoor.yaml',
                'use_rough_calib': True,
                'save_img': True,
                'folder': result_dir,
            }
        }
    }
    
    with open(output_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print(f"\n✓ 配置文件已生成: {output_file}\n")
    print("相机参数:")
    print(f"  分辨率: {camera_info['width']}x{camera_info['height']}")
    print(f"  焦距 fx: {camera_info['camera_matrix'][0]:.2f}")
    print(f"  焦距 fy: {camera_info['camera_matrix'][4]:.2f}")
    print(f"  主点 cx: {camera_info['camera_matrix'][2]:.2f}")
    print(f"  主点 cy: {camera_info['camera_matrix'][5]:.2f}")
    print(f"  畸变系数: {[f'{d:.6f}' for d in camera_info['dist_coeffs']]}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='从ROS2话题生成标定配置文件')
    parser.add_argument('--camera-info-topic', type=str, 
                       default='/camera/camera/color/camera_info',
                       help='相机信息话题 (default: /camera/camera/color/camera_info)')
    parser.add_argument('--bag-path', type=str, default='./calib_bag',
                       help='ROS2 bag文件路径')
    parser.add_argument('--pcd-path', type=str, default='./map.pcd',
                       help='PCD文件路径')
    parser.add_argument('--result-dir', type=str, default='./calib_results',
                       help='结果输出目录')
    parser.add_argument('--output', type=str, default='calib_config.yaml',
                       help='输出配置文件名')
    
    args = parser.parse_args()
    
    print(f"正在查询相机参数: {args.camera_info_topic}")
    print("(确保相机驱动正在运行...)\n")
    
    camera_info = get_camera_info(args.camera_info_topic)
    
    if camera_info:
        generate_config(camera_info, args.bag_path, args.pcd_path, 
                       args.result_dir, args.output)
    else:
        print("ERROR: 无法获取相机参数")
        sys.exit(1)
