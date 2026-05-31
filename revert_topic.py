import sys

with open("pick_lidar_xy.py", "r", encoding="utf-8") as f:
    text = f.read()

# Revert topic
text = text.replace('default="/Laser_map",', 'default="/livox/lidar",')

# Revert accumulation
text = text.replace('default=1,  # 使用全球 Laser_map 时不需要累积，1 帧即包含所有点', 'default=20,')

with open("pick_lidar_xy.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Reverted to /livox/lidar.")
