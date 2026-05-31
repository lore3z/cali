import sys

with open("live_calibrate_once.py", "r", encoding="utf-8") as f:
    text = f.read()

# Add accumulate param
old_parser = """    parser.add_argument("--point-stride", type=int, default=1, help="点云采样步 长，默认 1")"""
new_parser = """    parser.add_argument("--point-stride", type=int, default=1, help="点云采样步长，默认 1")
    parser.add_argument("--accumulate-frames", type=int, default=50, help="如果使用原始雷达数据，自动累积多帧（默认50帧/约5秒）以生成稠密点云，这对边缘标定非常重要")"""
if old_parser in text:
    text = text.replace(old_parser, new_parser)
else:
    # try matching roughly
    import re
    text = re.sub(r'(parser\.add_argument\("--point-stride"[^\n]+)', r'\1\n    parser.add_argument("--accumulate-frames", type=int, default=50, help="自动累积多帧")', text)

# Init
old_init = """        self.got_lidar = False

        self.image_sub = self.create_subscription("""
new_init = """        self.got_lidar = False
        self.collected_frames = 0
        self.point_buffer = []

        self.image_sub = self.create_subscription("""
if old_init in text:
    text = text.replace(old_init, new_init)

# Callback
old_cb = """        if not points:
            self.get_logger().warn("点云为空，继续等待下一帧")
            return

        self.lidar_points = np.asarray(points, dtype=np.float64)
        self.got_lidar = True
        self.get_logger().info(f"已收到点云: {len(points)} 个点")"""
new_cb = """        if not points:
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
            self.get_logger().info(f"已完毕，共累积点云: {len(self.lidar_points)} 个点")"""
if old_cb in text:
    text = text.replace(old_cb, new_cb)


with open("live_calibrate_once.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Patch calib applied.")
