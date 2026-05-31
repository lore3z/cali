#!/usr/bin/env python3
"""
直接从运行中的 FAST-LIO 保存地图和图像
不需要 ROS2 bag 录制
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv2
import sys
import time
from pathlib import Path


class CalibDataCollector(Node):
    def __init__(self, output_dir):
        super().__init__('calib_data_collector')
        self.bridge = CvBridge()
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.map_received = False
        self.image_received = False
        
        # 订阅 /Laser_map
        self.map_sub = self.create_subscription(
            PointCloud2,
            '/Laser_map',
            self.map_callback,
            1)
        
        # 订阅相机图像
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            1)
        
        self.get_logger().info('等待 /Laser_map 和 /camera/camera/color/image_raw...')
    
    def map_callback(self, msg):
        """保存点云地图"""
        if self.map_received:
            return
            
        try:
            # 方法1: 尝试使用 pcl 库
            try:
                import pcl
                from sensor_msgs_py import point_cloud2
                
                # 从 PointCloud2 消息读取点
                points_list = []
                for point in point_cloud2.read_points(msg, skip_nans=True):
                    points_list.append([point[0], point[1], point[2]])
                
                if len(points_list) == 0:
                    self.get_logger().warn('收到空的点云')
                    return
                
                # 创建 PCL 点云
                pc = pcl.PointCloud(points_list)
                
                # 保存为 PCD
                output_path = self.output_dir / 'map.pcd'
                pcl.save(pc, str(output_path))
                
                self.get_logger().info(f'✓ 地图已保存: {output_path}')
                self.get_logger().info(f'  点数: {len(points_list)}')
                self.map_received = True
                
            except ImportError:
                # 方法2: 如果没有 pcl，使用 ASCII PCD 格式
                self.get_logger().warn('pcl 库未找到，使用 ASCII PCD 格式保存')
                from sensor_msgs_py import point_cloud2
                
                # 读取点云数据
                points = list(point_cloud2.read_points(msg, skip_nans=True))
                
                if len(points) == 0:
                    self.get_logger().warn('收到空的点云')
                    return
                
                # 写入 PCD 文件
                output_path = self.output_dir / 'map.pcd'
                with open(output_path, 'w') as f:
                    # PCD 头
                    f.write("# .PCD v.7 - Point Cloud Data file format\n")
                    f.write("VERSION .7\n")
                    f.write("FIELDS x y z\n")
                    f.write("SIZE 4 4 4\n")
                    f.write("TYPE f f f\n")
                    f.write("COUNT 1 1 1\n")
                    f.write(f"WIDTH {len(points)}\n")
                    f.write("HEIGHT 1\n")
                    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                    f.write(f"POINTS {len(points)}\n")
                    f.write("DATA ascii\n")
                    
                    # 数据
                    for point in points:
                        f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
                
                self.get_logger().info(f'✓ 地图已保存: {output_path}')
                self.get_logger().info(f'  点数: {len(points)}')
                self.map_received = True
                
        except Exception as e:
            self.get_logger().error(f'保存地图失败: {e}')
            import traceback
            traceback.print_exc()
    
    def image_callback(self, msg):
        """保存相机图像"""
        if self.image_received:
            return
        
        try:
            # 转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 保存图像
            output_path = self.output_dir / 'image.png'
            cv2.imwrite(str(output_path), cv_image)
            
            self.get_logger().info(f'✓ 图像已保存: {output_path}')
            self.get_logger().info(f'  分辨率: {cv_image.shape[1]}x{cv_image.shape[0]}')
            self.image_received = True
            
        except Exception as e:
            self.get_logger().error(f'保存图像失败: {e}')
    
    def is_ready(self):
        return self.map_received and self.image_received


def main():
    if len(sys.argv) < 2:
        print("用法: python3 save_calib_data.py <output_dir>")
        print("例如: python3 save_calib_data.py calib_data/results")
        sys.exit(1)
    
    output_dir = sys.argv[1]
    
    rclpy.init()
    node = CalibDataCollector(output_dir)
    
    try:
        # 等待接收数据（超时60秒）
        timeout = 60
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            
            if node.is_ready():
                node.get_logger().info('✓ 所有数据已收集完成！')
                print("\n[标定数据已准备好]")
                print(f"  地图: {output_dir}/map.pcd")
                print(f"  图像: {output_dir}/image.png")
                return 0
        
        # 超时
        node.get_logger().error(f'超时！未能接收到所有数据')
        if not node.map_received:
            node.get_logger().error('  ✗ /Laser_map 未收到')
        if not node.image_received:
            node.get_logger().error('  ✗ 相机图像未收到')
        return 1
        
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
        return 0
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
