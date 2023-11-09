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

def get_tf(from_frame, to_frame):
    tf_listener = tf.TransformListener()
    
    # Allow tf to catch up
    rospy.sleep(1.0)
    
    try:
        # The transform function returns a tuple with translation and rotation
        (trans,rot) = tf_listener.lookupTransform(from_frame, to_frame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo(e)
        return None
    
    return trans, rot

def tf_e2g():
    t_adapter = [0, 0, 0.05] #thickness of the camera adapter
    t_e2g_0 = [0.0, 0.0, 0.1034] #initial translation tf from link8 to tcp 'https://download.franka.de/documents/220010_Product%20Manual_Franka%20Hand_1.2_EN.pdf'
    t_e2g = t_e2g_0 + t_adapter
    #r_c2e = [0.0, 0.0, -0.38268343236508984, 0.9238795325112867]
    r_e2g = np.array([[0.707, 0.707, 0],
                      [-0.707, 0.707, 0],
                      [0, 0, 0]])

    T_e2g = tft.compose_matrix(translate = t_e2g, angles=tft.euler_from_matrix(r_e2g))
    return T_e2g

ee_poses = []
aruco_poses = []

def move_2_20_config(joint_positions, path):

    global ee_poses, aruco_poses
    robot = frankx.Robot("10.8.11.204")
    # robot = 

    if not isinstance(joint_positions, list) or not all(isinstance(pos, list) for pos in joint_positions):
        print("Invalid input: joint_positions should be a list of lists")
        return

    for i, pos in enumerate(joint_positions):
        if not all(isinstance(angle, (int, float)) for angle in pos):
            print(f"Invalid joint positions at index {i}. All positions should be numbers.")
            return

        try:
            joint_motion = frankx.JointMotion(pos)
            robot.move(joint_motion)
            # p.move_to_joint_position(pos)
            rospy.sleep(1) # you might need to adjust this sleep duration depending on your setup
            print(f"After movement {i+1}, the end-effector pose is {robot.current_pose()}")

            # Save ee_pose ####notice####
            # p.ee_pose here is to acquire the pose of gripper_center_point!!!!!!!
            # not the pose of 'panda_link8'
            ee_pose = robot.current_pose()
            # ee_pose_list = list(ee_pose[0]) + [ee_pose[1].x, ee_pose[1].y, ee_pose[1].z, ee_pose[1].w]
            ee_pose_list = [ee_pose.x(), ee_pose.y(), ee_pose.z(), ee_pose.qw(), ee_pose.qx(), ee_pose.qy(), ee_pose.qz()]
            ee_poses.append(ee_pose_list)

            if global_pose is not None:
                print(f"Aruco pose after move #{i+1}:\n Position: {global_pose.pose.position}\n Orientation: {global_pose.pose.orientation}")

                # Save Aruco pose
                aruco_pose = [global_pose.pose.position.x, global_pose.pose.position.y, global_pose.pose.position.z, global_pose.pose.orientation.x, global_pose.pose.orientation.y, global_pose.pose.orientation.z, global_pose.pose.orientation.w]
                aruco_poses.append(aruco_pose)
                
            else:
                print(f"No Aruco pose received after move #{i+1}")

        except Exception as e:
            print(f"An error occurred when moving to joint position at index {i}: {e}")
            return

    # Save poses to txt files
    np.savetxt(os.path.join(path, 'ee_pose.txt'), ee_poses)
    np.savetxt(os.path.join(path, 'aruco_pose.txt'), aruco_poses)
        
def Eye_2_Hand():
    num = 18
    end_effector_poses = np.loadtxt(os.path.join(path, 'ee_pose.txt'))
    aruco_poses = np.loadtxt(os.path.join(path, 'aruco_pose.txt'))
    R_all_end_to_base = []
    T_all_end_to_base = []
    R_all_chess_to_cam = []
    T_all_chess_to_cam = []

    # Convert your end effector poses and aruco poses to the required format
    for ee_pose, aruco_pose in zip(end_effector_poses, aruco_poses):
        # End Effector Poses
        T_end_to_base = ee_pose[0:3]
        q_end_to_base = ee_pose[3:7]
        R_end_to_base = Rotation.from_quat(q_end_to_base).as_matrix()

        # Aruco Poses
        T_chess_to_cam = aruco_pose[0:3]
        q_chess_to_cam = aruco_pose[3:7]
        R_chess_to_cam = Rotation.from_quat(q_chess_to_cam).as_matrix()

        # Append to lists
        R_all_end_to_base.append(R_end_to_base)
        T_all_end_to_base.append(T_end_to_base.reshape(-1, 1))
        R_all_chess_to_cam.append(R_chess_to_cam)
        T_all_chess_to_cam.append(T_chess_to_cam.reshape(-1, 1))

    # Perform Hand-Eye Calibration
    R, T = cv2.calibrateHandEye(R_all_end_to_base, T_all_end_to_base, R_all_chess_to_cam, T_all_chess_to_cam, method=cv2.CALIB_HAND_EYE_TSAI)

    # Construct the transformation matrix
    RT=np.column_stack((R,T))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))#cam to end

    print('E_2_H matrix： \n', RT)


    third_cols_x = []
    third_cols_y = []
    third_cols_z = []
    #result validation，theoritically，tf from the bass to chess should be more still
    for i in range(num):

        # tf from hand to base
        RT_end_to_base=np.column_stack((R_all_end_to_base[i],T_all_end_to_base[i]))
        RT_end_to_base=np.row_stack((RT_end_to_base,np.array([0,0,0,1])))
        # print(RT_end_to_base)

        # tf from world to cam
        RT_chess_to_cam=np.column_stack((R_all_chess_to_cam[i],T_all_chess_to_cam[i]))
        RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
        # print(RT_chess_to_cam)

        # tf of E 2 H
        RT_cam_to_end=np.column_stack((R,T))
        RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))
        # print(RT_cam_to_end)

        # tf from world to cam
        RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam
        RT_chess_to_base=np.linalg.inv(RT_chess_to_base)
        
        third_cols_x.append([RT_chess_to_base[0, 3]])
        third_cols_y.append([RT_chess_to_base[1, 3]])
        third_cols_z.append([RT_chess_to_base[2, 3]])


    averages_x = np.mean(third_cols_x, axis=0)
    averages_y = np.mean(third_cols_y, axis=0)
    averages_z = np.mean(third_cols_z, axis=0)
    print('averages_x',averages_x)
    print('averages_y',averages_y)
    print('averages_z',averages_z)
    #dis = math.sqrt(averages_x**2 + averages_y**2 + averages_z**2)
    #print('averages_dis',dis)
    dis_sum=[]
    for i in range(num):

        # tf from hand to base
        RT_end_to_base=np.column_stack((R_all_end_to_base[i],T_all_end_to_base[i]))
        RT_end_to_base=np.row_stack((RT_end_to_base,np.array([0,0,0,1])))
        # print(RT_end_to_base)

        # tf from world to cam
        RT_chess_to_cam=np.column_stack((R_all_chess_to_cam[i],T_all_chess_to_cam[i]))
        RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
        # print(RT_chess_to_cam)

        # tf of E 2 H
        RT_cam_to_end=np.column_stack((R,T))
        RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))
        # print(RT_cam_to_end)

        # tf from world to cam
        RT_chess_to_base=RT_end_to_base@RT_cam_to_end@RT_chess_to_cam
        RT_chess_to_base=np.linalg.inv(RT_chess_to_base)
        
        third_cols_x.append([RT_chess_to_base[0, 3]])
        third_cols_y.append([RT_chess_to_base[1, 3]])
        third_cols_z.append([RT_chess_to_base[2, 3]])

        delta_x = RT_chess_to_base[0, 3] - averages_x
        delta_y = RT_chess_to_base[1, 3] - averages_y
        delta_z = RT_chess_to_base[2, 3] - averages_z
        dis = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        dis_sum.append(dis)
        
        #sqrt(delta_x^2 + delta_y^2 + delta_z^2)
        print('time',i)
        print(RT_chess_to_base[:3,:])
        print('')
        print('delta x y z', delta_x, delta_y, delta_z, dis)

    average = sum(dis_sum)/len(dis_sum)
    print('average dis', average)

def acquire_data():
    # r = ArmInterface()
    # p = PandaArm()
    joint_positions = np.loadtxt(os.path.join(path, 'joint_positions_improved.txt')).tolist()
    sub = rospy.Subscriber('/aruco_single/pose', PoseStamped, pose_callback)
    move_2_20_config(joint_positions, path)
    rospy.spin()

    
if __name__ == '__main__':
    
    rospy.init_node('calib_scripts')

    path = '/home/panda2/franka_cam_ws/src/franka_marker/scripts/config'
    # acquire_data()
    Eye_2_Hand()

    '''
    test on 08/11/23
    E_2_H matrix： 
    [[-4.95490712e-02 -9.98541322e-01 -2.14503745e-02  4.70602376e-02]
    [ 9.98771237e-01 -4.95171519e-02 -2.01697481e-03 -3.16416358e-02]
    [ 9.51871243e-04 -2.15239563e-02  9.99767880e-01  1.35281097e-01]
    [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

    '''


