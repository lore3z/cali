import re

paths = [
    "install/lidar_camera_calib/share/lidar_camera_calib/config/config_indoor.yaml",
    "src/lidar_camera_calib/config/config_indoor.yaml"
]

for p in paths:
    try:
        with open(p, 'r', encoding='utf-8') as f:
            t = f.read()
        
        # We want to replace the ExtrinsicMat data block securely.
        # Find: ExtrinsicMat: !!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: d\n  data: [...]
        t = re.sub(
            r"(ExtrinsicMat:[^\]]+data:\s*\[[^\]]+\])",
            "ExtrinsicMat: !!opencv-matrix\\n  rows: 4\\n  cols: 4\\n  dt: d\\n  data: [0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, -0.15, 0.0, 0.0, 0.0, 1.0]",
            t
        )
        
        with open(p, 'w', encoding='utf-8') as f:
            f.write(t)
        print(f"Fixed {p}")
    except Exception as e:
        print(f"Failed {p}: {e}")

