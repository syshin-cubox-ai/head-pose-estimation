import math

import numpy as np
import open3d as o3d


def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


filename = 'assets/model.txt'

raw_value = []
with open(filename) as file:
    for line in file:
        raw_value.append(line)
model_points = np.array(raw_value, dtype=np.float32)
model_points = np.reshape(model_points, (3, -1)).T
model_points[:, 2] *= -1
R = eulerAnglesToRotationMatrix([0, 0, np.pi])
model_points = np.matmul(model_points, R)
print(model_points[36:42].mean(0))
print(model_points[42:48].mean(0))
print(model_points[30])
print(model_points[48])
print(model_points[54])

axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=25, origin=[0, 0, 0])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(model_points[:42])
o3d.visualization.draw_geometries([pcd, axis])
