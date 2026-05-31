import sys

with open("pick_lidar_xy.py", "r", encoding="utf-8") as f:
    text = f.read()

# 1. Update basic strings and type hints
text = text.replace('self.label_to_index: dict[int, int] = {}', 'self.label_to_index: np.ndarray = np.array([])')
text = text.replace('self.selected_message: str = "等待图像和点云..."', 'self.selected_message: str = "Waiting for image and point clouds..."')
text = text.replace('self.selected_message = "等待相机参数..."', 'self.selected_message = "Waiting for camera params..."')
text = text.replace('self.selected_message = "等待雷达点云..."', 'self.selected_message = "Waiting for LiDAR clouds..."')
text = text.replace('self.selected_message = "当前点云没有投影到相机前方"', 'self.selected_message = "No points projected to camera front"')

# 2. Add X > 0 condition
old_valid = 'valid_mask = points_camera[:, 2] > 0.05'
new_valid = 'valid_mask = (points_camera[:, 2] > 0.05) & (points_lidar[:, 0] > 0.0)'
if old_valid not in text: print("WARNING: valid_mask not found"); sys.exit(1)
text = text.replace(old_valid, new_valid)

# 3. Vectorize reproject_points block
old_block1 = """        height, width = self.image.shape[:2]
        in_view = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < width)
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < height)
        )

        self.projected_pixels = pixels[in_view]
        self.projected_lidar_points = points_lidar[in_view]
        self.projected_camera_points = points_camera[in_view]

        if self.projected_pixels.size == 0:
            self.dense_label_map = None
            self.label_to_index = {}
            self.selected_message = "当前没有可见的投影点"
            return

        height, width = self.image.shape[:2]
        seed_mask = np.full((height, width), 255, dtype=np.uint8)
        seed_best: dict[tuple[int, int], int] = {}

        for index, pixel in enumerate(self.projected_pixels):
            x = int(round(float(pixel[0])))
            y = int(round(float(pixel[1])))
            if x < 0 or x >= width or y < 0 or y >= height:
                continue
            key = (x, y)
            previous_index = seed_best.get(key)
            if previous_index is None:
                seed_best[key] = index
                continue
            if self.projected_camera_points[index, 2] < self.projected_camera_points[previous_index, 2]:
                seed_best[key] = index

        self.label_to_index = {}
        for (x, y), index in seed_best.items():
            seed_mask[y, x] = 0

        if len(seed_best) == 0:
            self.dense_label_map = None
            self.selected_message = "投影点全部落在图像外"
            return

        _, labels = cv2.distanceTransformWithLabels(
            seed_mask,
            cv2.DIST_L2,
            5,
            labelType=getattr(cv2, "DIST_LABEL_PIXEL", 1),
        )
        self.dense_label_map = labels

        for (x, y), index in seed_best.items():
            label = int(labels[y, x])
            self.label_to_index[label] = index

        self.need_reproject = False"""

new_block1 = """        xs = np.rint(pixels[:, 0]).astype(np.int32)
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
            self.selected_message = "No visible projected points"
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

        self.need_reproject = False"""

if old_block1 not in text: print("WARNING: old_block1 not found"); sys.exit(1)
text = text.replace(old_block1, new_block1)

# 4. Fix mouse callback
old_mouse = """    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.dense_label_map is None or self.projected_lidar_points is None:
            self.selected_message = "当前还没有可用的投影点"
            return

        label = int(self.dense_label_map[y, x])
        index = self.label_to_index.get(label)
        if index is None:
            self.selected_message = f"像素({x},{y}) 没有找到对应的最近点"
            self.get_logger().info(self.selected_message)
            return

        nearest_pixel = self.projected_pixels[index]
        min_distance = float(np.linalg.norm(nearest_pixel - np.array([x, y], dtype=np.float64)))

        lidar_point = self.projected_lidar_points[index]
        camera_point = self.projected_camera_points[index]
        self.selected_pixel = (x, y)
        self.selected_message = (
            f"像素({x},{y}) -> 雷达坐标 x={lidar_point[0]:.3f}, y={lidar_point[1]:.3f}, z={lidar_point[2]:.3f}, "
            f"相机坐标 z={camera_point[2]:.3f}, 像素距离={min_distance:.2f}px"
        )
        self.get_logger().info(self.selected_message)"""

new_mouse = """    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
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

        lidar_point = self.projected_lidar_points[index]
        camera_point = self.projected_camera_points[index]
        self.selected_pixel = (x, y)
        self.selected_message = (
            f"Pixel({x},{y}) -> LiDAR X={lidar_point[0]:.3f} Y={lidar_point[1]:.3f} Z={lidar_point[2]:.3f}, "
            f"Cam Z={camera_point[2]:.3f}, dist={min_distance:.2f}px"
        )
        self.get_logger().info(self.selected_message)"""

if old_mouse not in text: print("WARNING: old_mouse not found"); sys.exit(1)
text = text.replace(old_mouse, new_mouse)

with open("pick_lidar_xy.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Patch applied successfully.")
