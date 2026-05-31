import yaml
import numpy as np

extrinsic = np.array([
    [0.0, -1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0, 0.0],
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])

camera_matrix = np.array([915.6424560546875, 0.0, 636.0186157226562, 0.0, 915.1904907226562, 370.6102600097656, 0.0, 0.0, 1.0]).reshape(3, 3)

pts_lidar = np.array([
    [2.0, 0.0, 0.0, 1.0],  # Center forward
    [2.0, 1.0, 0.0, 1.0],  # Forward left
    [2.0, -1.0,0.0, 1.0],  # Forward right
    [2.0, 0.0, 1.0, 1.0],  # Forward up
    [2.0, 0.0,-1.0, 1.0]   # Forward down
])

pts_cam = (extrinsic @ pts_lidar.T).T[:, :3]
print("Pts cam:\\n", pts_cam)

pts_2d = []
for p in pts_cam:
    x = p[0] / p[2]
    y = p[1] / p[2]
    u = camera_matrix[0,0] * x + camera_matrix[0,2]
    v = camera_matrix[1,1] * y + camera_matrix[1,2]
    pts_2d.append((u, v))
print("Pts 2d:\\n", pts_2d)

