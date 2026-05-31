import sys

with open("pick_lidar_xy.py", "r", encoding="utf-8") as f:
    text = f.read()

old_path = '        default="/home/lzt/m20pro/calib_data/results/extrinsic.txt",'
new_path = '        default="/home/lzt/m20pro/calib_data/live_once/extrinsic.txt",'

if old_path in text:
    text = text.replace(old_path, new_path)
    with open("pick_lidar_xy.py", "w", encoding="utf-8") as f:
        f.write(text)
    print("Path updated.")
else:
    print("Path not found.")
