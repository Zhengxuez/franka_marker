#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tft
import os
import numpy as np
from geometry_msgs.msg import PoseStamped
import cv2
from scipy.spatial.transform import Rotation
import math
import frankx

global_pose = None

def pose_callback(pose_msg):
    global global_pose
    global_pose = pose_msg

if __name__ == '__main__':
    rospy.init_node('collector_node')  # Initialize the ROS node first

    sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, pose_callback)
    robot = frankx.Robot("10.8.11.204")
    state = robot.read_once()
    ee_pose = robot.current_pose()
    translation_ee = ee_pose.translation()
    rotation = ee_pose.rotation()
    translation = np.array([[0], [0], [0]])

    # Construct 4x4 matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[0:3, 0:3] = rotation
    transformation_matrix[0:3, 3] = translation[:, 0]
    quaternion = tft.quaternion_from_matrix(transformation_matrix)
    # quaterion = tft.quaternion_from_euler(roll, pitch, yaw)
    # rotation_matrix = np.array(ee_pose)

    joint_state = state.q
    print('js', joint_state)
    # print('ee', ee_pose)
    print('ee_pose', translation_ee[0],translation_ee[1],translation_ee[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3])
    # print('rotation', rotation)
    while not rospy.is_shutdown():
        if global_pose is not None:
            print('Aruco pose',global_pose.pose.position.x, global_pose.pose.position.y,global_pose.pose.position.z, global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w)
            break
        rospy.sleep(0.1)
