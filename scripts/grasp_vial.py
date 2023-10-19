#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tft
import os
import numpy as np
from franka_gripper.msg import MoveActionGoal, GraspActionGoal, GraspEpsilon
from geometry_msgs.msg import PoseStamped
import cv2
from scipy.spatial.transform import Rotation
import math
from math import pi
import copy
import geometry_msgs.msg
import quaternion
import moveit_commander
import sys

from frankx import Robot, Gripper, Affine
import frankx

class VialGrasper:
    def __init__(self):
        
        rospy.init_node('vial_grasper')
        self.p = PandaArm()
        self.inter = ArmInterface()
        self.g = self.p.get_gripper()
        self.pose_subscriber = rospy.Subscriber(
            '/aruco_single/pose', PoseStamped, self.pose_callback)

    def move_to_neutral(self):
       #self.p.wait_for_motion_completion()
       self.p.move_to_neutral()

    def move_to_check(self):
        #self.p.wait_for_motion_completion()
        self.p.move_to_joint_position([0.19723233, -0.5032747, 0.39010309, -2.23760213, 0.12057556, 1.74502069, 1.25511978], timeout=10.0)


    def B2E(self):
      pos, ori = self.p.ee_pose()
      #print('pos and ori', pos, ori)
      ori_array = [ori.x, ori.y, ori.z, ori.w]

      # Convert the quaternion to a rotation matrix
      rotation_matrix = tft.quaternion_matrix(ori_array)

      # Create a 4x4 transformation matrix
      transformation_matrix = np.eye(4)
      transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
      transformation_matrix[:3, 3] = pos

      return transformation_matrix
    

    def E2Q(self, pose_msg):
        # Calculate the transformation matrix from the calibration board to the end effector
        t_e2c = np.array([0.06010367, -0.03320061, -0.09002263])
        r_e2c = np.array([[-0.02028174,  -0.99758689,  -0.06640065],
                          [0.99979007,  -0.02004369,  -0.00424929],
                          [0.00290812, -0.06647289, 0.99778399]])
        T_e2c = tft.compose_matrix(translate=t_e2c, angles=tft.euler_from_matrix(r_e2c))
        T_e2q = tft.concatenate_matrices(T_e2c, self.C2Q(pose_msg))
        #T_q2e_1 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[1])
        #T_q2e_2 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[2])
        #T_q2e_3 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[3])
        #T_q2e_4 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[4])
        #T_q2e_5 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[5])
        #T_q2e_6 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[6])
        #T_q2e_7 = tft.concatenate_matrices(T_c2e, self.Q2C(pose_msg)[7])
        print('T_e2q',T_e2q)
        return T_e2q #, T_q2e_1 #, T_q2e_2, T_q2e_3, T_q2e_4, T_q2e_5, T_q2e_6, T_q2e_7
    
    def C2Q(self,pose_msg):
        # to calculate the transformation matrix from calibration to camera
        pos = pose_msg.pose.position
        ori = pose_msg.pose.orientation 
        translation = [pos.x, pos.y, pos.z]  # translation in the x, y and z direction
        rotation = [ori.x, ori.y, ori.z, ori.w]
        record = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]

        #print('RECORD qr translation + rotation of aruco', record)

        ori_euler = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w], axes='sxyz') #output euler z y x
        #print('ori_euler',ori_euler)

        T_now = tft.compose_matrix(translate=translation, angles=(ori_euler[0], ori_euler[1], ori_euler[2]))
        T_c2q = tft.concatenate_matrices(T_now, self.E2Q_desired())
        print('T_c2q',T_c2q)
        #T_c2q_1 = tft.concatenate_matrices(T_now, self.Q2E_desired()[1])
        #T_c2q_2 =tft.concatenate_matrices(T_now, self.Q2E_desired()[2])
        #T_c2q_3 = tft.concatenate_matrices(T_now, self.Q2E_desired()[3])
        #T_c2q_4 = tft.concatenate_matrices(T_now, self.Q2E_desired()[4])
        #T_c2q_5 =tft.concatenate_matrices(T_now, self.Q2E_desired()[5])
        #T_c2q_6 =tft.concatenate_matrices(T_now, self.Q2E_desired()[6])
        #T_c2q_7 =tft.concatenate_matrices(T_now, self.Q2E_desired()[7])
        return T_c2q #, #T_c2q_1 #, T_c2q_2, T_c2q_3, T_c2q_4, T_c2q_5, T_c2q_6, T_c2q_7
    
    def E2Q_desired(self):
        # 7 poses of the end effector in terms of the calibration board
        t_0 = np.array([0, 0, 0.45])
      #   t_1 = np.array([0, 0, 0.3])
      #   t_2 = np.array([-0.1128, -0.007, 0.30]) # pre pick up position
      #   t_3 = np.array([-0.1128, -0.007, 0.205]) # pick up position
      #   t_4 = np.array([-0.113, -0.007, 0.30]) # rise up
      #   t_5 = np.array([-0.154, 0.157, 0.30]) # drop-down top
      #   t_6 = np.array([-0.154, 0.157, 0.23]) # drop down position
      #   t_7 = np.array([-0.154, 0.157, 0.30]) # up
        R_d = tft.euler_matrix(-pi, 0, pi/2)[:3, :3] # the new pose under current marker's frame
        T_d_0 = tft.compose_matrix(translate=t_0, angles=tft.euler_from_matrix(R_d))
        print('T_d_0',T_d_0)
      #   T_d_1 = tft.compose_matrix(translate=t_1, angles=tft.euler_from_matrix(R_d))
      #   T_d_2 = tft.compose_matrix(translate=t_2, angles=tft.euler_from_matrix(R_d))
      #   T_d_3 = tft.compose_matrix(translate=t_3, angles=tft.euler_from_matrix(R_d))
      #   T_d_4 = tft.compose_matrix(translate=t_4, angles=tft.euler_from_matrix(R_d))
      #   T_d_5 = tft.compose_matrix(translate=t_5, angles=tft.euler_from_matrix(R_d))
      #   T_d_6 = tft.compose_matrix(translate=t_6, angles=tft.euler_from_matrix(R_d))
      #   T_d_7 = tft.compose_matrix(translate=t_7, angles=tft.euler_from_matrix(R_d))

        return T_d_0 #, T_d_1, T_d_2, T_d_3, T_d_4, T_d_5, T_d_6, T_d_7
    
    def execute(self, pose_matrix):
        print('pose_matrix \n',pose_matrix)
        target_pose = geometry_msgs.msg.PoseStamped()
      #   target_pose.header.frame_id = "kmriiwa_link_0"
        #pose = Pose()
        target_pose.header.frame_id = "panda_hand"
        target_pose.pose.position.x = pose_matrix[0][0][3]
        target_pose.pose.position.y = pose_matrix[0][1][3]
        target_pose.pose.position.z = pose_matrix[0][2][3]
        quat = tft.quaternion_from_matrix(pose_matrix[0])
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]
        return target_pose
    


    def pose_callback(self, pose_msg):
        print('pose_msg', pose_msg)
        #print('length of Q2E',len(self.Q2E(pose_msg)))
        pose_matrices = [tft.concatenate_matrices(self.B2E(),self.E2Q(pose_msg))]
        print('self.B2E()',self.B2E())
        print('self.p.ee_pose()', self.p.ee_pose())
        print('self.E2Q(pose_msg)', self.E2Q(pose_msg))
        target_pose_0 = self.execute(pose_matrices)

        ori = quaternion.quaternion(target_pose_0.pose.orientation.x, target_pose_0.pose.orientation.y, target_pose_0.pose.orientation.z, target_pose_0.pose.orientation.w)
        pos = np.array([target_pose_0.pose.position.x, target_pose_0.pose.position.y, target_pose_0.pose.position.z])
        self.p.move_to_cartesian_pose(pos,ori)
        #print(self.p.angles())

      #   print('pose matrix', pose_matrices)
      #   target_poses = [self.execute(pose_matrix) for pose_matrix in pose_matrices]

      #   waypoints = [copy.deepcopy(target_pose) for target_pose in target_poses]

      #   for i, waypoint in enumerate(waypoints):
      #       print('i',i)
      #       pos,ori = waypoint
      #       self.p.move_to_cartesian_pose(pos,ori)

            # self.move_group.set_pose_target(waypoint, self.end_effector_link)
            # plan = self.move_group.go(wait=True)
            # self.move_group.clear_pose_targets()
            # if i == 3:
            #     #gripper_pub(3, 0)
            #     self.gripper_publisher.publish(self.gripper_pub(3, 0.2))
            #     rospy.sleep(1.5)
            # if i == 6:
            #     #gripper_pub(1, 0.5)
            #     self.gripper_publisher.publish(self.framegripper_pub(2, 0))
            #     rospy.sleep(1.5)
        self.pose_subscriber.unregister()
        pass

    def run(self):
        #self.p.move_to_neutral(timeout=10.0)
        
        #self.move_to_check()
        
        #rospy.sleep(3.0)
        self.pose_subscriber
        print('robot has moved to marker')
        #rospy.spin()

    def run1(self):
        self.move_to_neutral()



def move_to_check():
    joint_position = [0.02898749, -0.22848525, 0.33815208, -1.72260216, 0.06149956, 1.51219192, 1.12661461]
    robotx.move_to_joint_position(joint_position)


def move_to_check2():

       robotx.move_to_joint_position([-0.43906129, -1.42307817, 1.18843465, -2.62009176, -0.18600077, 2.93034442, 2.11520688], timeout=10.0)



def B2E(self):
    # Get current pose from the robot
    current_pose = self.robotx.current_pose()

    # Extract position and orientation
    position = current_pose.translation()
    rotation = current_pose.rotation()

    # Convert rotation matrix to quaternion
    quat = frankx.Rotation(rotation).to_quaternion()

    # Convert the quaternion to a rotation matrix using tf
    rotation_matrix = tft.quaternion_matrix(quat)

    # Create a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = position

    return transformation_matrix




def T2Q_desired():
    
    delate_x = 0.048
    delata_y = 0.0645
    delta_z = 0.015
    h_chemspeed = 0.017
    t_0 = np.array([0, 0, 0.30])
    t_1 = np.array([0, 0, 0.15])
    t_2 = np.array([-0.1131-delate_x, -0.007-delata_y, 0.15]) # pre pick up position
    t_3 = np.array([-0.1131-delate_x, -0.007-delata_y, 0.05 + delta_z - h_chemspeed]) # pick up position 1
    t_4 = np.array([-0.1131-delate_x, -0.007-delata_y, 0.15]) # rise up
    t_5 = np.array([-0.1517-delate_x, 0.157-delata_y, 0.15]) # drop-down top
    t_6 = np.array([-0.1517-delate_x, 0.157-delata_y, 0.07 + delta_z - h_chemspeed]) # drop down position 1
    t_7 = np.array([-0.1517-delate_x, 0.157-delata_y, 0.15]) # up
    t_8 = np.array([-0.1517-delate_x, 0.157-delata_y, 0.05 + delta_z - h_chemspeed]) # pick up position 2
    t_9 = np.array([-0.1517-delate_x, 0.157-delata_y, 0.15]) # up
    t_10 = np.array([-0.1135-delate_x, -0.008-delata_y, 0.15]) # rise up
    t_11 = np.array([-0.1135-delate_x, -0.008-delata_y, 0.07 + delta_z - h_chemspeed]) # drop down position 2
    t_12 = np.array([-0.1135-delate_x, -0.008-delata_y, 0.15]) # rise up

    R_d = np.array([pi, 0, -pi/2]) #rxyz here

    T_d_0 = tft.compose_matrix(translate=t_0, angles=R_d)
    # print('T_d_0',T_d_0)
    T_d_1 = tft.compose_matrix(translate=t_1, angles=R_d)
    T_d_2 = tft.compose_matrix(translate=t_2, angles=R_d)
    T_d_3 = tft.compose_matrix(translate=t_3, angles=R_d)
    T_d_4 = tft.compose_matrix(translate=t_4, angles=R_d)
    T_d_5 = tft.compose_matrix(translate=t_5, angles=R_d)
    T_d_6 = tft.compose_matrix(translate=t_6, angles=R_d)
    T_d_7 = tft.compose_matrix(translate=t_7, angles=R_d)
    T_d_8 = tft.compose_matrix(translate=t_8, angles=R_d)
    T_d_9 = tft.compose_matrix(translate=t_9, angles=R_d)
    T_d_10 = tft.compose_matrix(translate=t_10, angles=R_d)
    T_d_11 = tft.compose_matrix(translate=t_11, angles=R_d)
    T_d_12 = tft.compose_matrix(translate=t_12, angles=R_d)

    return T_d_0, T_d_1, T_d_2, T_d_3, T_d_4, T_d_5, T_d_6, T_d_7, T_d_8, T_d_9, T_d_10, T_d_11, T_d_12

def T2Q_desired2():
    
    delate_x = 0.032
    delata_y = 0.0085
    delta_z = 0.015
    h_pick = -0.005
    h_drop = 0

    t_0 = np.array([0, 0, 0.30])
    t_1 = np.array([0, 0, 0.15])
    t_2 = np.array([-0.016+delate_x, -0.094+delata_y, 0.15]) # pre pick up position
    t_3 = np.array([-0.016+delate_x, -0.094+delata_y, h_pick-0.002]) # pick up position 1
    t_4 = np.array([-0.016+delate_x, -0.094+delata_y, 0.15]) # rise up
    t_5 = np.array([-0.0545+delate_x, 0.071+delata_y, 0.15]) # drop-down top
    t_6 = np.array([-0.0545+delate_x, 0.071+delata_y, h_drop]) # drop down position 1
    t_7 = np.array([-0.0545+delate_x, 0.071+delata_y, 0.15]) # up
    t_8 = np.array([-0.0545+delate_x, 0.071+delata_y, h_pick]) # pick up position 2
    t_9 = np.array([-0.0545+delate_x, 0.071+delata_y, 0.15]) # up
    t_10 = np.array([-0.016+delate_x, -0.095+delata_y, 0.15]) # rise up
    t_11 = np.array([-0.016+delate_x, -0.095+delata_y, h_drop]) # drop down position 2
    t_12 = np.array([-0.016+delate_x, -0.095+delata_y, 0.15]) # rise up

    R_d = np.array([pi, 0, -pi/2]) #rxyz here

    T_d_0 = tft.compose_matrix(translate=t_0, angles=R_d)
    # print('T_d_0',T_d_0)
    T_d_1 = tft.compose_matrix(translate=t_1, angles=R_d)
    T_d_2 = tft.compose_matrix(translate=t_2, angles=R_d)
    T_d_3 = tft.compose_matrix(translate=t_3, angles=R_d)
    T_d_4 = tft.compose_matrix(translate=t_4, angles=R_d)
    T_d_5 = tft.compose_matrix(translate=t_5, angles=R_d)
    T_d_6 = tft.compose_matrix(translate=t_6, angles=R_d)
    T_d_7 = tft.compose_matrix(translate=t_7, angles=R_d)
    T_d_8 = tft.compose_matrix(translate=t_8, angles=R_d)
    T_d_9 = tft.compose_matrix(translate=t_9, angles=R_d)
    T_d_10 = tft.compose_matrix(translate=t_10, angles=R_d)
    T_d_11 = tft.compose_matrix(translate=t_11, angles=R_d)
    T_d_12 = tft.compose_matrix(translate=t_12, angles=R_d)

    return T_d_0, T_d_1, T_d_2, T_d_3, T_d_4, T_d_5, T_d_6, T_d_7, T_d_8, T_d_9, T_d_10, T_d_11, T_d_12


def C2Q(pose_msg):
            # to calculate the transformation matrix from marker to camera
    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation 
    translation = [pos.x, pos.y, pos.z]  # translation in the x, y and z direction
    rotation = [ori.x, ori.y, ori.z, ori.w]
    record = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]

    #print('RECORD qr translation + rotation of aruco', record)

    ori_euler = tft.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w], axes='sxyz') #output euler x y z
    #print('ori_euler',ori_euler)

    T_now = tft.compose_matrix(translate=translation, angles=(ori_euler[0], ori_euler[1], ori_euler[2]))
    # T_now is T_camera_marker
    T_c2q = tft.concatenate_matrices(T_now, T2Q_desired()[0])
    #print('T_c2q',T_c2q)
    T_c2q_1 = tft.concatenate_matrices(T_now, T2Q_desired()[1])
    T_c2q_2 = tft.concatenate_matrices(T_now, T2Q_desired()[2])
    T_c2q_3 = tft.concatenate_matrices(T_now, T2Q_desired()[3])
    T_c2q_4 = tft.concatenate_matrices(T_now, T2Q_desired()[4])
    T_c2q_5 = tft.concatenate_matrices(T_now, T2Q_desired()[5])
    T_c2q_6 = tft.concatenate_matrices(T_now, T2Q_desired()[6])
    T_c2q_7 = tft.concatenate_matrices(T_now, T2Q_desired()[7])
    T_c2q_8 = tft.concatenate_matrices(T_now, T2Q_desired()[8])
    T_c2q_9 = tft.concatenate_matrices(T_now, T2Q_desired()[9])
    T_c2q_10 = tft.concatenate_matrices(T_now, T2Q_desired()[10])
    T_c2q_11 = tft.concatenate_matrices(T_now, T2Q_desired()[11])
    T_c2q_12 = tft.concatenate_matrices(T_now, T2Q_desired()[12])
    return T_c2q, T_c2q_1, T_c2q_2, T_c2q_3, T_c2q_4, T_c2q_5, T_c2q_6, T_c2q_7, T_c2q_8, T_c2q_9, T_c2q_10, T_c2q_11, T_c2q_12

def E2Q(pose_msg):
    # Calculate the transformation matrix from the calibration board to the end effector
    t_e2c = np.array([0.06010367, -0.03320061, -0.09002263])
    r_e2c = np.array([[-0.02028174,  -0.99758689,  -0.06640065],
                        [0.99979007,  -0.020014369,  -0.00424929],
                        [0.00290812, -0.06647289, 0.99778399]])
    T_e2c = tft.compose_matrix(translate=t_e2c, angles=tft.euler_from_matrix(r_e2c))
    T_e2q = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[0])
    T_q2e_1 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[1])
    T_q2e_2 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[2])
    T_q2e_3 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[3])
    T_q2e_4 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[4])
    T_q2e_5 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[5])
    T_q2e_6 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[6])
    T_q2e_7 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[7])
    T_q2e_8 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[8])
    T_q2e_9 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[9])
    T_q2e_10 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[10])
    T_q2e_11 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[11])
    T_q2e_12 = tft.concatenate_matrices(T_e2c, C2Q(pose_msg)[12])
    #print('T_e2q',T_e2q)
    return T_e2q, T_q2e_1, T_q2e_2, T_q2e_3, T_q2e_4, T_q2e_5, T_q2e_6, T_q2e_7, T_q2e_8, T_q2e_9, T_q2e_10, T_q2e_11, T_q2e_12
    
def execute(pose_matrix):
    #print('pose_matrix \n',pose_matrix)
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = pose_matrix[0][3]
    target_pose.position.y = pose_matrix[1][3]
    target_pose.position.z = pose_matrix[2][3]
    quat = tft.quaternion_from_matrix(pose_matrix)
    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]
    return target_pose

def gripper_init():
    goal_msg.goal.width = 0.05
    goal_msg.goal.speed = 0.1
    goal_msg.goal.force = 0.0
    pub.publish(goal_msg)
    rospy.sleep(1.0)

def callback(pose_msg):
    pose_matrix_0 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[0])
    pose_matrix_1 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[1])
    pose_matrix_2 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[2])
    pose_matrix_3 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[3])
    pose_matrix_4 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[4])
    pose_matrix_5 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[5])
    pose_matrix_6 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[6])
    pose_matrix_7 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[7])
    pose_matrix_8 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[8])
    pose_matrix_9 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[9])
    pose_matrix_10 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[10])
    pose_matrix_11= tft.concatenate_matrices(B2E(), E2Q(pose_msg)[11])
    pose_matrix_12 = tft.concatenate_matrices(B2E(), E2Q(pose_msg)[12])
    target_pose_0 = execute(pose_matrix_0)
    target_pose_1 = execute(pose_matrix_1)
    target_pose_2 = execute(pose_matrix_2)
    target_pose_3 = execute(pose_matrix_3)
    target_pose_4 = execute(pose_matrix_4)
    target_pose_5 = execute(pose_matrix_5)
    target_pose_6 = execute(pose_matrix_6)
    target_pose_7 = execute(pose_matrix_7)
    target_pose_8 = execute(pose_matrix_8)
    target_pose_9 = execute(pose_matrix_9)
    target_pose_10 = execute(pose_matrix_10)
    target_pose_11 = execute(pose_matrix_11)
    target_pose_12 = execute(pose_matrix_12)

    waypoints = []
    wpose = target_pose_0
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_1
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_2
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_3
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_4
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_5
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_6
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_7
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_8
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_9
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_10
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_11
    waypoints.append(copy.deepcopy(wpose))
    wpose = target_pose_12
    waypoints.append(copy.deepcopy(wpose))


    for i, waypoint in enumerate(waypoints):
        #print('target_pose', waypoint)
        # if i==0:
        #     gripper_init()

        if i == 4:
            #g.close()
            #rospy.sleep(5.0)
            goal_msg.goal.width = 0.017 # for chemspeed vial
            goal_msg.goal.speed = 0.05
            goal_msg.goal.force = 5
            goal_msg.goal.epsilon.inner = 0.001
            goal_msg.goal.epsilon.outer = 0.005
            pub.publish(goal_msg)
            rospy.sleep(2.0)
        if i == 7:
            #g.open()
            goal_msg.goal.width = 0.05
            goal_msg.goal.speed = 0.1
            goal_msg.goal.force = 0.0
            pub.publish(goal_msg)
            rospy.sleep(2.0)
        if i == 9:
            #g.close()pp
            #rospy.sleep(5.0)
            goal_msg.goal.width = 0.017 # for chemspeed vial
            goal_msg.goal.speed = 0.05
            goal_msg.goal.force = 5
            goal_msg.goal.epsilon.inner = 0.001
            goal_msg.goal.epsilon.outer = 0.005
            pub.publish(goal_msg)
            rospy.sleep(2.0)
        if i == 12:
            #g.open()
            goal_msg.goal.width = 0.05
            goal_msg.goal.speed = 0.1
            goal_msg.goal.force = 0.0
            pub.publish(goal_msg)
            rospy.sleep(2.0)

        #gripper_init()
        move_group.set_pose_target(waypoint, "panda_hand_tcp")
        plan = move_group.go(wait=True)
        move_group.clear_pose_targets() 
    sub.unregister()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('grasp_vial')
    robot = moveit_commander.RobotCommander(robot_description='robot_description') #node name space
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name,robot_description='/robot_description')
    
    robotx = Robot("172.16.0.2")  # Change IP address 
    gripper = Gripper(robot)

    # Initialization with frankx
    robotx.recover()
    robotx.set_default_behavior()

    # p = PandaArm()
    # g = p.get_gripper()
    # print(p.angles())
    move_to_check()
    pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=10)
    goal_msg = GraspActionGoal()
    sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, callback)
    move_to_check()
    rospy.spin()


    
    

