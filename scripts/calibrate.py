import numpy as np
import scipy.optimize as opt
import pytransform3d as p3d
from pytransform3d import transformations
import matplotlib.pyplot as plt
from pytransform3d.rotations import (quaternion_integrate, matrix_from_quaternion, plot_basis)
import cv2

data = np.load("scripts/ee_calib_data.npz")
base_to_end_effector = data['base_to_end_effector']
world_to_rigid_body = data['world_to_rigid_body']

world_to_rigid_body_inv = np.linalg.inv(world_to_rigid_body)

R, t = cv2.calibrateHandEye(base_to_end_effector[:, :3, :3], base_to_end_effector[:, :3, 3],
                            world_to_rigid_body_inv[:, :3, :3], world_to_rigid_body_inv[:, :3, 3])
X = np.concatenate((np.concatenate((R, t), axis=1), np.array([[0, 0, 0, 1]])))
X = np.linalg.inv(X)
print(X)

ax = None
A = np.zeros((4,4))
for t in range(len(world_to_rigid_body)):
    # T = world_to_rigid_body[t] @ X
    # ax = plot_basis(ax=ax, s=0.15, R=T[:3, :3], p=T[:3, 3])

    T = world_to_rigid_body[t] @ X @ np.linalg.inv(base_to_end_effector[t])
    print("world_to_robotbase",T)
    ax = plot_basis(ax=ax, s=0.15, R=T[:3, :3], p=T[:3, 3])
    A = A + T
    # ax = plot_basis(ax=ax, s=0.15, R=world_to_rigid_body[t][:3, :3], p=world_to_rigid_body[t][:3, 3])
A = A/t
print("avg world_to_base",A)
np.save("RT_world2franka.npy", A)
plt.show()