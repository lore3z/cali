import sys

with open("pick_lidar_xy.py", "r", encoding="utf-8") as f:
    text = f.read()

# Change accumulate frames to 1 if using Laser_map, to avoid memory explosion
old_accum = '        default=20,'
new_accum = '        default=1,  # 使用全球 Laser_map 时不需要累积，1 帧即包含所有点'
if old_accum in text:
    text = text.replace(old_accum, new_accum)

with open("pick_lidar_xy.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Patch accum applied.")
