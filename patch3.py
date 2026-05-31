import sys

with open("pick_lidar_xy.py", "r", encoding="utf-8") as f:
    text = f.read()

# Replace the default cloud topic
old_topic = 'default="/livox/lidar",'
new_topic = 'default="/Laser_map",'
if old_topic in text:
    text = text.replace(old_topic, new_topic)
else:
    print("WARNING: default topic not found")

with open("pick_lidar_xy.py", "w", encoding="utf-8") as f:
    f.write(text)

print("Patch 3 applied.")
