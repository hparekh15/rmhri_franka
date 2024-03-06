import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.rotations import (plot_basis)

# Load the data
ee_file = 'eval\data\T0_mb\ee.csv'
ml_file = 'eval\data\T0_mb\ml.csv'
mb_file = 'eval\data\T0_mb\mb.csv'
world2franka = np.load('eval\data\RT_world2franka.npy')
# franka2world = np.linalg.inv(world2franka)
# cam2world = np.load('eval\data\RT_camera2world.npy')
# world2cam = np.linalg.inv(cam2world)

# print("w2f",world2franka)
# print("det w2f",np.linalg.det(world2franka))
# print("f2w",franka2world)
# print("det f2w",np.linalg.det(franka2world))
# print("w2f x f2w",np.dot(world2franka,franka2world))
# print("c2w",cam2world)
# print("det c2w",np.linalg.det(cam2world))
# print("w2c",world2cam)
# print("det w2c",np.linalg.det(world2cam))
# print("c2w x w2c",np.dot(cam2world,world2cam))

def read_csv(file):
    df = pd.read_csv(file)
    return df['t'], df['X'], df['Y'], df['Z']

def match_timestamps(main_t, other_t, other_x, other_y, other_z):
    closest_indices = main_t.apply(lambda x: np.abs(other_t - x).idxmin())
    return other_t.iloc[closest_indices].reset_index(drop=True), other_x.iloc[closest_indices].reset_index(drop=True), other_y.iloc[closest_indices].reset_index(drop=True), other_z.iloc[closest_indices].reset_index(drop=True)

def transform_frame(T, x, y, z):
    homo_coords = np.vstack([x,y,z,np.ones(len(x))])
    world_coords = np.dot(T, homo_coords)
    return world_coords[0,:], world_coords[1,:], world_coords[2,:]

def plot_frame(matrix, ax, label):
    # Origin of the frame
    origin = np.dot(matrix, np.array([0, 0, 0, 1]))

    # Directions of the X, Y, Z axes
    scale = 0.3
    x_dir = np.dot(matrix, np.array([scale, 0, 0, 1]))
    y_dir = np.dot(matrix, np.array([0, scale, 0, 1]))
    z_dir = np.dot(matrix, np.array([0, 0, scale, 1]))

    # Plotting the axes
    ax.quiver(*origin[:3], *(x_dir[:3] - origin[:3]), color='r', arrow_length_ratio=0.2, label=f'X_{label}')
    ax.quiver(*origin[:3], *(y_dir[:3] - origin[:3]), color='g', arrow_length_ratio=0.2, label=f'Y_{label}')
    ax.quiver(*origin[:3], *(z_dir[:3] - origin[:3]), color='b', arrow_length_ratio=0.2, label=f'Z_{label}')

# Read Data
t1, x1, y1, z1 = read_csv(ee_file)
t2, x2, y2, z2 = read_csv(ml_file)
t3, x3, y3, z3 = read_csv(mb_file)

# Match Timestamps
min_len = min(len(t1), len(t2), len(t3))
if len(t1) == min_len:
    t2, x2, y2, z2 = match_timestamps(t1, t2, x2, y2, z2)
    t3, x3, y3, z3 = match_timestamps(t1, t3, x3, y3, z3)
elif len(t2) == min_len:
    t1, x1, y1, z1 = match_timestamps(t2, t1, x1, y1, z1)
    t3, x3, y3, z3 = match_timestamps(t2, t3, x3, y3, z3)
else:
    t1, x1, y1, z1 = match_timestamps(t3, t1, x1, y1, z1)
    t2, x2, y2, z2 = match_timestamps(t3, t2, x2, y2, z2)

# Transform to World Frame
x_ee, y_ee, z_ee = transform_frame(world2franka, x1, y1, z1)

# Left to Right
# t1, x_ee, y_ee, z_ee = t1[:100], x_ee[:100], y_ee[:100], z_ee[:100]
# t2, x2, y2, z2 = t2[:100], x2[:100], y2[:100], z2[:100]
# t3, x3, y3, z3 = t3[:100], x3[:100], y3[:100], z3[:100]

# Right to Left
# t1, x_ee, y_ee, z_ee = t1[100:], x_ee[100:], y_ee[100:], z_ee[100:]
# t2, x2, y2, z2 = t2[100:], x2[100:], y2[100:], z2[100:]
# t3, x3, y3, z3 = t3[100:], x3[100:], y3[100:], z3[100:]


# Plot the data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.title('Task 0 - Marker Based')
ax.plot(x_ee, y_ee, z_ee, label='ee')
ax.plot(x2, y2, z2, label='ml')
ax.plot(x3, y3, z3, label='mb')

# Plot Coordinate frames
# plot_frame(np.eye(4), ax, 'World')
# plot_frame(cam2world, ax, 'Cam')
# plot_frame(world2franka, ax, 'Franka')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.savefig('eval\data\T0_mb\T0_mb.png')
# plt.show()

ml2ee_dist = np.sqrt((x2 - x_ee)**2 + (y2 - y_ee)**2 + (z2 - z_ee)**2)
mb2ee_dist = np.sqrt((x3 - x_ee)**2 + (y3 - y_ee)**2 + (z3 - z_ee)**2)
ml2mb_dist = np.sqrt((x2 - x3)**2 + (y2 - y3)**2 + (z2 - z3)**2)

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
ax1.plot(t1, ml2ee_dist, 'r',label='ml')
ax1.plot(t1, mb2ee_dist,'g' ,label='mb')
ax1.set_ylabel('Distance (m)')
ax1.legend()
ax1.grid(True)
ax1.set_title('Safety Distance from EE over Time')

ax2.plot(t1, ml2mb_dist, label='diff from gt')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Distance (m)')
ax2.legend()
ax2.grid(True)
ax2.set_title('Tracking Error over Time')
plt.savefig('eval\data\T0_mb\safety_tracking.png')
# plt.show()

velocity = np.gradient([x_ee, y_ee, z_ee], t1 ,axis=1)
# print(velocity.shape)
acceleration = np.gradient(velocity, t1, axis=1) #3,308
# print(acceleration.shape)
jerk = np.gradient(acceleration, t1, axis=1) 
jerk = np.linalg.norm(jerk, axis=0)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
velocity = np.linalg.norm(velocity, axis=0)
ax1.plot(t1, velocity)

ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('EE Velocity over Time')
ax1.grid(True)
acceleration = np.linalg.norm(acceleration, axis=0)
ax2.plot(t1, acceleration)

ax2.set_ylabel('Acceleration (m/s^2)')
ax2.set_title('EE Acceleration over Time')
ax2.grid(True)
ax3.plot(t1, jerk)
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Jerk (m/s^3)')
ax3.set_title('EE Jerk over Time')
ax3.grid(True)
plt.savefig('eval\data\T0_mb\jerk.png')
plt.show()