#!/usr/bin/env python3
"""Modified picker with hard-coded intrinsics and extrinsic inversion test."""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2


def load_extrinsic_matrix(path: Path) -> np.ndarray:
    """读取标定输出的 4x4 外参矩阵。"""
    rows = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line:
            continue
        values = [value.strip() for value in line.split(",") if value.strip()]
        rows.append([float(value) for value in values])

    matrix = np.asarray(rows, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError(f"外参矩阵格式错误: {path}, 当前形状: {matrix.shape}")
    return matrix


class PixelToLidarPickerDebug(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("pixel_to_lidar_picker_debug")
        self.bridge = CvBridge()
        self.args = args

        self.window_name = args.window_name
        self.image: Optional[np.ndarray] = None
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.points_buffer = deque(maxlen=args.accumulate_frames)
        self.latest_lidar_points: Optional[np.ndarray] = None
        self.projected_pixels: Optional[np.ndarray] = None
        self.projected_lidar_points: Optional[np.ndarray] = None
        self.projected_camera_points: Optional[np.ndarray] = None
        self.dense_label_map: Optional[np.ndarray] = None
        self.label_to_index: np.ndarray = np.array([])
        self.selected_pixel: Optional[Tuple[int, int]] = None
        self.selected_message: str = "Waiting for image and point clouds..."
        self.need_reproject = False

        # Load extrinsic
        self.extrinsic = load_extrinsic_matrix(Path(args.extrinsic_file))
        self.get_logger().info(f"原始外参矩阵:\n{self.extrinsic}")
        
        # If args.invert_extrinsic, invert it
        if args.invert_extrinsic:
            M_inv = np.eye(4, dtype=np.float64)
            M_inv[:3, :3] = self.extrinsic[:3, :3].T
            M_inv[:3, 3] = -self.extrinsic[:3, :3].T @ self.extrinsic[:3, 3]
            self.extrinsic = M_inv
            self.get_logger().info(f"已反演外参矩阵:\n{self.extrinsic}")
        
        self.rvec, _ = cv2.Rodrigues(self.extrinsic[:3, :3])
        self.tvec = self.extrinsic[:3, 3].reshape(3, 1)

        # Use hard-coded intrinsics for RealSense D435i
        # These are typical values; actual values should come from calibration
        fx_default = 918.3
        fy_default = 917.6
        cx_default = 640.0
        cy_default = 359.6
        
        if args.fx > 0 and args.fy > 0 and args.cx >= 0 and args.cy >= 0:
            fx, fy, cx, cy = args.fx, args.fy, args.cx, args.cy
            self.get_logger().info("使用命令行提供的相机内参")
        else:
            fx, fy, cx, cy = fx_default, fy_default, cx_default, cy_default
            self.get_logger().info(f"使用硬编码的RealSense D435i内参: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

        self.camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)
        self.get_logger().info(f"相机矩阵 K:\n{self.camera_matrix}")

        self.image_sub = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            10,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            args.cloud_topic,
            self.cloud_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.timer = self.create_timer(1.0 / args.fps, self.render)

        self.get_logger().info(f"图像话题: {args.image_topic}")
        self.get_logger().info(f"点云话题: {args.cloud_topic}")
        self.get_logger().info(f"外参文件: {args.extrinsic_file}")
        self.get_logger().info("左键点击图像像素，终端会输出最近雷达点的坐标")

    def image_callback(self, msg: Image) -> None:
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.need_reproject = True
        except Exception as exc:
            self.get_logger().error(f"图像转换失败: {exc}")

    def cloud_callback(self, msg: PointCloud2) -> None:
        points = []
        stride = max(1, int(self.args.point_stride))
        for index, point in enumerate(
            point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % stride != 0:
                continue
            points.append([point[0], point[1], point[2]])

        if not points:
            if not self.points_buffer:
                self.latest_lidar_points = None
                self.projected_pixels = None
                self.projected_lidar_points = None
                self.projected_camera_points = None
            return

        self.points_buffer.append(np.asarray(points, dtype=np.float64))
        self.latest_lidar_points = np.vstack(self.points_buffer)
        self.need_reproject = True

    def reproject_points(self) -> None:
        if self.image is None:
            return
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.selected_message = "Waiting for camera params..."
            return
        if self.latest_lidar_points is None or self.latest_lidar_points.size == 0:
            self.selected_message = "Waiting for LiDAR clouds..."
            return

        points_lidar = self.latest_lidar_points
        ones = np.ones((points_lidar.shape[0], 1), dtype=np.float64)
        points_h = np.hstack([points_lidar, ones])
        points_camera = (self.extrinsic @ points_h.T).T[:, :3]

        valid_mask = points_camera[:, 2] > 0.05
        points_lidar = points_lidar[valid_mask]
        points_camera = points_camera[valid_mask]

        if points_lidar.size == 0:
            self.projected_pixels = None
            self.projected_lidar_points = None
            self.projected_camera_points = None
            self.selected_message = "No points projected to camera front"
            return

        pixels, _ = cv2.projectPoints(
            points_lidar.reshape(-1, 1, 3).astype(np.float64),
            self.rvec,
            self.tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        pixels = pixels.reshape(-1, 2)

        xs = np.rint(pixels[:, 0]).astype(np.int32)
        ys = np.rint(pixels[:, 1]).astype(np.int32)
        height, width = self.image.shape[:2]
        
        in_view = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)

        self.projected_pixels = pixels[in_view]
        self.projected_lidar_points = points_lidar[in_view]
        self.projected_camera_points = points_camera[in_view]
        xs = xs[in_view]
        ys = ys[in_view]

        if self.projected_pixels.size == 0:
            self.dense_label_map = None
            self.label_to_index = np.array([])
            self.selected_message = f"No visible projected points (checked {len(pixels)} total)"
            return

        depths = self.projected_camera_points[:, 2]
        # Sort by descending depth (far -> close), so closest points overwrite farther ones at identical pixels
        sort_idx = np.argsort(depths)[::-1]
        xs_s = xs[sort_idx]
        ys_s = ys[sort_idx]

        seed_mask = np.full((height, width), 255, dtype=np.uint8)
        seed_mask[ys_s, xs_s] = 0

        _, labels = cv2.distanceTransformWithLabels(
            seed_mask,
            cv2.DIST_L2,
            5,
            labelType=getattr(cv2, "DIST_LABEL_PIXEL", 1),
        )
        self.dense_label_map = labels

        # Map labels to the point indices via array indexing
        max_label = np.max(labels)
        self.label_to_index = np.full(max_label + 1, -1, dtype=np.int32)
        label_ids = labels[ys_s, xs_s]
        self.label_to_index[label_ids] = sort_idx

        self.need_reproject = False

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.dense_label_map is None or self.projected_lidar_points is None:
            self.selected_message = "No projected points available"
            self.get_logger().info(self.selected_message)
            return

        label = int(self.dense_label_map[y, x])
        if label < 0 or label >= len(self.label_to_index):
            self.selected_message = f"Pixel({x},{y}) no nearest point found"
            self.get_logger().info(self.selected_message)
            return

        index = self.label_to_index[label]
        if index == -1:
            self.selected_message = f"Pixel({x},{y}) no nearest point found"
            self.get_logger().info(self.selected_message)
            return

        nearest_pixel = self.projected_pixels[index]
        min_distance = float(np.linalg.norm(nearest_pixel - np.array([x, y], dtype=np.float64)))

        # Check pickup radius to avoid wild depth jumping
        if min_distance > self.args.pick_radius:
            self.selected_message = f"Pixel({x},{y}) -> nearest point too far: {min_distance:.1f}px > {self.args.pick_radius:.1f}px"
            self.get_logger().info(self.selected_message)
            return

        lidar_point = self.projected_lidar_points[index]
        camera_point = self.projected_camera_points[index]
        self.selected_pixel = (x, y)
        self.selected_message = (
            f"Pixel({x},{y}) -> LiDAR X={lidar_point[0]:.3f} Y={lidar_point[1]:.3f} Z={lidar_point[2]:.3f}, "
            f"Cam Z={camera_point[2]:.3f}, dist={min_distance:.2f}px"
        )
        self.get_logger().info(self.selected_message)

    def render(self) -> None:
        if self.image is None:
            canvas = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                canvas,
                "Waiting for camera image...",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.window_name, canvas)
            cv2.waitKey(1)
            return

        if self.need_reproject:
            self.reproject_points()

        display = self.image.copy()
        height, width = display.shape[:2]

        if self.projected_pixels is not None and self.projected_pixels.size > 0:
            xs = np.rint(self.projected_pixels[:, 0]).astype(np.int32)
            ys = np.rint(self.projected_pixels[:, 1]).astype(np.int32)
            valid = (xs >= 0) & (xs < width) & (ys >= 0) & (ys < height)
            xs = xs[valid]
            ys = ys[valid]
            display[ys, xs] = (0, 255, 0)

        if self.selected_pixel is not None:
            cv2.circle(display, self.selected_pixel, 6, (0, 0, 255), 2)
            cv2.drawMarker(
                display,
                self.selected_pixel,
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=14,
                thickness=2,
            )

        lines = [
            "Left click a pixel to query the nearest LiDAR point",
            self.selected_message,
        ]
        y = 28
        for line in lines:
            cv2.putText(
                display,
                line,
                (20, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            y += 28

        cv2.imshow(self.window_name, display)
        cv2.waitKey(1)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Debug version with inverted extrinsic option")
    parser.add_argument(
        "--image-topic",
        default="/camera/camera/color/image_raw",
        help="相机图像话题",
    )
    parser.add_argument(
        "--cloud-topic",
        default="/livox/lidar",
        help="雷达点云话题，默认使用原始雷达话题",
    )
    parser.add_argument(
        "--extrinsic-file",
        default="/home/lzt/m20pro/calib_data/live_once/extrinsic.txt",
        help="标定输出的 4x4 外参矩阵文件",
    )
    parser.add_argument(
        "--invert-extrinsic",
        action="store_true",
        help="是否反演外参矩阵（如果标定输出是Camera->LiDAR格式）",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="显示刷新频率",
    )
    parser.add_argument(
        "--point-stride",
        type=int,
        default=1,
        help="点云降采样步长，越小越密集但越耗时，默认 1",
    )
    parser.add_argument(
        "--accumulate-frames",
        type=int,
        default=20,
        help="累积多少帧点云以增加全局密度（10帧约等于1秒），这对Livox非重复扫描很重要",
    )
    parser.add_argument(
        "--pick-radius",
        type=float,
        default=30.0,
        help="点击时最大允许的吸附像素距离，超过该距离不受理，防止深度跳变到远处的背景",
    )
    parser.add_argument("--fx", type=float, default=0.0, help="相机 fx")
    parser.add_argument("--fy", type=float, default=0.0, help="相机 fy")
    parser.add_argument("--cx", type=float, default=-1.0, help="相机 cx")
    parser.add_argument("--cy", type=float, default=-1.0, help="相机 cy")
    parser.add_argument(
        "--window-name",
        default="LiDAR Image Picker (Debug)",
        help="OpenCV 窗口名称",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    extrinsic_path = Path(args.extrinsic_file)
    if not extrinsic_path.exists():
        print(f"ERROR: 外参文件不存在: {extrinsic_path}")
        return 1

    rclpy.init()
    node = PixelToLidarPickerDebug(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
