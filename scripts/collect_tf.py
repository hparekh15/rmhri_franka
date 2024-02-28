#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import pytransform3d as p3d
from pytransform3d import rotations
import cv2
import numpy as np
import sys

cv2.namedWindow("TF Collector")
rospy.init_node('tf_collector', anonymous=True)
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
data_optitrack = []
data_ee = []
index = 0

def tf2_list(tf2):
    return [tf2.transform.translation.x,
            tf2.transform.translation.y,
            tf2.transform.translation.z,
            tf2.transform.rotation.w,
            tf2.transform.rotation.x,
            tf2.transform.rotation.y,
            tf2.transform.rotation.z]

print('loop started')
try:
    while not rospy.is_shutdown():
        cv2.imshow("TF Collector", np.zeros((100,100), np.uint8))
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        elif key == ord("s"):
            try:
                base_to_end_effector = tf_buffer.lookup_transform("panda_link0", "panda_EE", rospy.Time(0))
                T = np.array(tf2_list(base_to_end_effector))
                R = p3d.rotations.matrix_from_quaternion(T[3:])
                base_to_end_effector = np.concatenate((np.concatenate((R, T[:3][:, np.newaxis]), axis=1), np.array([[0, 0, 0, 1]])))
                # print("base_to_end_effector in loop", base_to_end_effector)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to capture ee TFs.")

            try:
                world_to_rigid_body = tf_buffer.lookup_transform("world", "franka_panda_ee", rospy.Time(0))
                T = np.array(tf2_list(world_to_rigid_body))
                R = p3d.rotations.matrix_from_quaternion(T[3:])
                world_to_rigid_body = np.concatenate((np.concatenate((R, T[:3][:, np.newaxis]), axis=1), np.array([[0, 0, 0, 1]])))
                # print("world_to_rigid_body in loop",world_to_rigid_body)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("Failed to capture optitrack TFs.")

            index += 1
            print("saved {0:d} transformation matrix!".format(index))
            print("base_to_end_effector", base_to_end_effector)
            print("world_to_rigid_body",world_to_rigid_body)
            data_optitrack.append(world_to_rigid_body)
            data_ee.append(base_to_end_effector)
finally:
    np.savez("ee_calib_data.npz",
             base_to_end_effector=np.stack(data_ee),
             world_to_rigid_body=np.stack(data_optitrack))


  
    