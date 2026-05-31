import sys

yaml_path = "install/lidar_camera_calib/share/lidar_camera_calib/config/config_indoor.yaml"
with open(yaml_path, 'r', encoding='utf-8') as f:
    text = f.read()

# Replace the ExtrinsicMat data
old_data = """  data: [0.0,   -1.0,   0.0,    0.0,
         0.0,  0.0,  -1.0,    0.0,
         1.0,   0.0,    0.0,    0.0,
         0.0,   0.0,    0.0,    1.0]"""

# T_z = -0.15 means Camera Pos X = +0.15 (15cm in front of LiDAR)
new_data = """  data: [0.0,   -1.0,   0.0,    0.0,
         0.0,  0.0,  -1.0,    0.0,
         1.0,   0.0,    0.0,   -0.15,
         0.0,   0.0,    0.0,    1.0]"""

if old_data in text:
    text = text.replace(old_data, new_data)
else:
    print("WARNING: Exact match failed")

with open(yaml_path, 'w', encoding='utf-8') as f:
    f.write(text)

print("Patch yaml applied.")

# Also update the source tree version so it persists on rebuild
src_yaml_path = "src/lidar_camera_calib/config/config_indoor.yaml"
try:
    with open(src_yaml_path, 'w', encoding='utf-8') as f:
        f.write(text)
    print("Source yaml updated.")
except Exception as e:
    pass

