#!/usr/bin/env python

import rospy
import tf
import tf.transformations as tft
import os
import numpy as np
from franka_interface import ArmInterface
from franka_gripper.msg import MoveActionGoal
from panda_robot import PandaArm
from geometry_msgs.msg import PoseStamped
import cv2
from scipy.spatial.transform import Rotation
import math
from math import pi
import copy
import geometry_msgs.msg
import quaternion


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
        
if __name__ == '__main__':
    
    grasper = VialGrasper()


    print("Running run1")
    #grasper.run1()
    print("Finished run1")

    print("Running run")
    grasper.run()
    


