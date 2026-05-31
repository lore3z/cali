import sys

with open("pick_lidar_xy.py", "r", encoding="utf-8") as f:
    text = f.read()

# 1. Revert X>0 cutoff that caused half screen to be empty
old_mask = 'valid_mask = (points_camera[:, 2] > 0.05) & (points_lidar[:, 0] > 0.0)'
new_mask = 'valid_mask = points_camera[:, 2] > 0.05'
if old_mask in text:
    text = text.replace(old_mask, new_mask)
else:
    print("WARNING: valid_mask condition not found!")

# 2. Add max distance filter in mouse_callback
old_mouse = """        nearest_pixel = self.projected_pixels[index]
        min_distance = float(np.linalg.norm(nearest_pixel - np.array([x, y], dtype=np.float64)))

        lidar_point = self.projected_lidar_points[index]"""

new_mouse = """        nearest_pixel = self.projected_pixels[index]
        min_distance = float(np.linalg.norm(nearest_pixel - np.array([x, y], dtype=np.float64)))

        # Check pickup radius to avoid wild depth jumping
        if min_distance > self.args.pick_radius:
            self.selected_message = f"Pixel({x},{y}) -> nearest point too far: {min_distance:.1f}px > {self.args.pick_radius:.1f}px"
            self.get_logger().info(self.selected_message)
            return

        lidar_point = self.projected_lidar_points[index]"""
if old_mouse in text:
    text = text.replace(old_mouse, new_mouse)
else:
    print("WARNING: mouse distance block not found!")

# 3. Update the argparser help text and default value
old_arg = """    parser.add_argument(
        "--pick-radius",
        type=float,
        default=5.0,
        help="保留兼容参数；当前使用全图最近邻映射，不再做硬阈值过滤",
    )"""

new_arg = """    parser.add_argument(
        "--pick-radius",
        type=float,
        default=30.0,
        help="点击时最大允许的吸附像素距离，超过该距离不受理，防止深度跳变到远处的背景",
    )"""
if old_arg in text:
    text = text.replace(old_arg, new_arg)

with open("pick_lidar_xy.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Patch applied successfully.")
