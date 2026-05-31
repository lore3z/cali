#!/usr/bin/env python3
"""从 ROS2 topic 抓取一组数据并执行一次标定。

流程：
1. 订阅相机图像、相机参数和雷达点云
2. 收到一组数据后保存为 image.png / map.pcd
3. 生成 lidar_camera_calib 需要的参数 YAML
4. 调用现有离线标定程序完成一次校准
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


def write_ascii_pcd(path: Path, points: np.ndarray) -> None:
    """保存 xyz 点云到 ASCII PCD。"""
    with path.open("w", encoding="utf-8") as file_handle:
        file_handle.write("# .PCD v.7 - Point Cloud Data file format\n")
        file_handle.write("VERSION .7\n")
        file_handle.write("FIELDS x y z\n")
        file_handle.write("SIZE 4 4 4\n")
        file_handle.write("TYPE f f f\n")
        file_handle.write("COUNT 1 1 1\n")
        file_handle.write(f"WIDTH {len(points)}\n")
        file_handle.write("HEIGHT 1\n")
        file_handle.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        file_handle.write(f"POINTS {len(points)}\n")
        file_handle.write("DATA ascii\n")
        for x, y, z in points:
            file_handle.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def build_param_yaml(
    output_file: Path,
    image_file: Path,
    pcd_file: Path,
    result_file: Path,
    camera_matrix: list[float],
    dist_coeffs: list[float],
    calib_config_file: Path,
    folder: Path,
) -> None:
    content = f"""lidar_camera_calib:
  ros__parameters:
    image_file: "{image_file}"
    pcd_file: "{pcd_file}"
    result_file: "{result_file}"
    camera_matrix: {camera_matrix}
    dist_coeffs: {dist_coeffs}
    calib_config_file: "{calib_config_file}"
    use_rough_calib: true
    save_img: true
    folder: "{folder}"
"""
    output_file.write_text(content, encoding="utf-8")


class LiveCalibCollector(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("live_calib_collector")
        self.args = args
        self.bridge = CvBridge()

        self.image: Optional[np.ndarray] = None
        self.camera_info: Optional[CameraInfo] = None
        self.lidar_points: Optional[np.ndarray] = None
        self.got_image = False
        self.got_camera_info = False
        self.got_lidar = False
        self.collected_frames = 0
        self.point_buffer = []

        self.image_sub = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            10,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            args.camera_info_topic,
            self.camera_info_callback,
            10,
        )
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            args.cloud_topic,
            self.lidar_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.get_logger().info(f"等待图像: {args.image_topic}")
        self.get_logger().info(f"等待相机参数: {args.camera_info_topic}")
        self.get_logger().info(f"等待点云: {args.cloud_topic}")

    def image_callback(self, msg: Image) -> None:
        if self.got_image:
            return
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.got_image = True
            self.get_logger().info("已收到图像")
        except Exception as exc:
            self.get_logger().error(f"图像转换失败: {exc}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.got_camera_info:
            return
        self.camera_info = msg
        self.got_camera_info = True
        self.get_logger().info("已收到相机参数")

    def lidar_callback(self, msg: PointCloud2) -> None:
        if self.got_lidar:
            return

        points = []
        stride = max(1, int(self.args.point_stride))
        for index, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % stride != 0:
                continue
            points.append([point[0], point[1], point[2]])

        if not points:
            self.get_logger().warn("点云为空，继续等待下一帧")
            return

        self.point_buffer.append(np.asarray(points, dtype=np.float64))
        self.collected_frames += 1
        
        # 如果是 Laser_map 等已经稠密的话题，1帧即可；否则累积多帧
        target_frames = 1 if "map" in self.args.cloud_topic.lower() else self.args.accumulate_frames
        
        self.get_logger().info(f"收集雷达帧: {self.collected_frames} / {target_frames}")
        
        if self.collected_frames >= target_frames:
            self.lidar_points = np.vstack(self.point_buffer)
            self.got_lidar = True
            self.get_logger().info(f"已完毕，共累积点云: {len(self.lidar_points)} 个点")

    def ready(self) -> bool:
        return self.got_image and self.got_camera_info and self.got_lidar


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="从 ROS2 topic 抓取一帧数据并自动执行一次标定")
    parser.add_argument("--image-topic", default="/camera/camera/color/image_raw", help="相机图像话题")
    parser.add_argument("--camera-info-topic", default="/camera/camera/color/camera_info", help="相机参数话题")
    parser.add_argument("--cloud-topic", default="/livox/lidar", help="点云话题，默认使用原始雷达话题 /livox/lidar")
    parser.add_argument("--output-dir", default="/home/lzt/m20pro/calib_data/live_once", help="输出目录")
    parser.add_argument("--point-stride", type=int, default=1, help="点云采样步长，默认 1")
    parser.add_argument("--accumulate-frames", type=int, default=50, help="自动累积多帧")
    parser.add_argument("--timeout", type=float, default=60.0, help="等待 topic 的超时时间")
    parser.add_argument("--calib-config-file", default="", help="标定算法配置文件，默认使用 package 中的 config_indoor.yaml")
    parser.add_argument("--launch-command", default="ros2 launch lidar_camera_calib calib.launch.py", help="执行标定的命令前缀")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    package_share = Path(get_package_share_directory("lidar_camera_calib"))
    calib_config_file = Path(args.calib_config_file) if args.calib_config_file else package_share / "config" / "config_indoor.yaml"
    if not calib_config_file.exists():
        print(f"ERROR: 标定配置文件不存在: {calib_config_file}")
        return 1

    rclpy.init()
    node = LiveCalibCollector(args)
    start_time = time.time()

    try:
        while rclpy.ok() and time.time() - start_time < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.ready():
                break
    finally:
        rclpy.shutdown()

    if not node.ready():
        print("ERROR: 超时，未同时收到图像、相机参数和点云")
        return 1

    image_file = output_dir / "image.png"
    pcd_file = output_dir / "map.pcd"
    result_file = output_dir / "extrinsic.txt"
    param_file = output_dir / "calib_config.yaml"

    cv2.imwrite(str(image_file), node.image)

    if node.lidar_points is None:
        print("ERROR: 点云未准备好")
        return 1
    write_ascii_pcd(pcd_file, node.lidar_points)

    if node.camera_info is None:
        print("ERROR: 相机参数未准备好")
        return 1

    build_param_yaml(
        output_file=param_file,
        image_file=image_file,
        pcd_file=pcd_file,
        result_file=result_file,
        camera_matrix=list(node.camera_info.k),
        dist_coeffs=(list(node.camera_info.d) + [0.0] * 5)[:5],
        calib_config_file=calib_config_file,
        folder=output_dir,
    )

    print(f"\n已保存: {image_file}")
    print(f"已保存: {pcd_file}")
    print(f"已生成: {param_file}")
    print("开始执行一次标定...\n")

    launch_cmd = args.launch_command.split() + [f"lidarcameracalib_param_dir:={param_file}"]
    completed = subprocess.run(launch_cmd, check=False)
    if completed.returncode != 0:
        print(f"ERROR: 标定命令退出码 {completed.returncode}")
        return completed.returncode

    print(f"标定完成，结果文件: {result_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())