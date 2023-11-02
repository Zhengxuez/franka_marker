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
# import frankx

global_pose = None

def pose_callback(pose_msg):
    global global_pose
    global_pose = pose_msg

if __name__ == '__main__':
    rospy.init_node('collector_node')  # Initialize the ROS node first
    sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, pose_callback)
    while not rospy.is_shutdown():
        if global_pose is not None:
            print(f"Aruco pose:\n Position: {global_pose.pose.position}\n Orientation: {global_pose.pose.orientation}")
            break
        rospy.sleep(0.1)
    # robot = frankx.Robot("10.8.11.204")
    # state = robot.read_once()
    # ee_pose = robot.current_pose()
    # joint_state = state.q
    # print('js', joint_state)
    # print('ee', ee_pose)
    # print(f"Aruco pose:\n Position: {global_pose}\n")
