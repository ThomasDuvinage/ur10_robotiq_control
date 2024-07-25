import yaml
import numpy as np

data = {}

with open("output/calibration_matrix.yaml", "r") as f:
    data = yaml.safe_load(f)

camera_matrix = np.array(data["camera_matrix"])

# Example pixel coordinates of the object in the image
u = 328  # example x pixel coordinates
v = 238  # example y pixel coordinates

# Depth of the object in meters (example value)
depth = 0.8

# Convert pixel coordinates to normalized image coordinates
x_norm = (u - camera_matrix[0][2]) / camera_matrix[0][0]
y_norm = (v - camera_matrix[1][2]) / camera_matrix[1][1]

# Direction vector in camera coordinates
direction_vector = np.array([x_norm, y_norm, 1])

# Convert direction vector to 3D position in camera coordinates
position_camera = direction_vector * depth

# Example camera pose (rotation matrix and translation vector)
R = np.eye(3)  # identity rotation (no rotation)
t = np.array([0, 0, 0])  # zero translation (camera at origin)

# Transform to world coordinates
position_world = np.dot(R, position_camera) + t

print("Object position in camera coordinates:", position_camera)
print("Object position in world coordinates:", position_world)

