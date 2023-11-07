import argparse
import json
import socket
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import time
import math
import select
import numpy as np
# from aruco_detector.msg import ArucoPose
import tf.transformations as tft
import threading
import frankx
from frankx import Robot, Affine, LinearRelativeMotion, JointMotion, LinearMotion, PositionHold, Kinematics
from pynput import keyboard

def parse_arguments():
    parser = argparse.ArgumentParser(description="marker-based manipulation.")
    parser.add_argument('--mode', choices=['go_to_check', 'record_waypoint', 'calculate_Tmarker', 'execute_saved_waypoints', 'save_eye_hand', 'save_Tbase_to_ee', 'save_current_Tcamera_to_marker', 'save_Tcamera_to_marker_1'], required=True,
                        help="Mode in which to run the server.")
    return parser.parse_args()

class franka_marker:
    def __init__(self):
        self.saved_poses = {"matrix": []}
        rospy.init_node('franka_listener')
        self.marker_pose = None
        self.marker_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.marker_callback)
        self.robot = frankx.Robot("10.8.11.204")
        self.robot.set_dynamic_rel(0.06)
        # self.state = self.robot.read_once()
        self.recorded_poses = []
        self.collecting = True

    def keyboard_listener(self):
        # Listener for keyboard events
        with keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release) as listener:
            listener.join()

    
    def invert_transform(self, matrix):
        """Inverts a 4x4 transformation matrix."""
        return np.linalg.inv(matrix)

    def multiply_transforms(self, *matrices):
        """Multiplies multiple 4x4 transformation matrices."""
        result = np.eye(4)
        for matrix in matrices:
            result = np.dot(result, matrix)
        return result
    def quaternion_to_matrix(self, quaternion):

        return tft.quaternion_matrix(quaternion)[:3, :3]

    def marker_callback(self, msg):
        """Callback for the aruco pose subscriber."""
        # Extract translation (position) and quaternion (orientation) from the ArucoPose
        t = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        # print(q)

        rotation_matrix = self.quaternion_to_matrix(q)  

        # Create the 4x4 transformation matrix
        self.marker_pose = [
            [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], t[0]],
            [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], t[1]],
            [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], t[2]],
            [0, 0, 0, 1]
        ]

    def save_Tcamera_to_marker_1(self):
        # Wait until we receive a marker pose
        while self.marker_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

            data = {
                "matrix": self.marker_pose
            }

        with open("Tcamera_to_marker_1.json", "w") as file:
            json.dump(data, file, indent = 4)

        print("Tcamera_to_marker_1 saved successfully!")

    def save_Tcamera_to_marker_c(self):
        # Wait until we receive a marker pose
        while self.marker_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

            data = {
                "matrix": self.marker_pose
            }

        with open("Tcamera_to_marker_c.json", "w") as file:
            json.dump(data, file, indent = 4)

        print("Fetching current Tcamera_to_marker !")

    def save_poses_to_json(self):
        with open("ee_poses.json", "w") as file:
            json.dump(self.saved_poses, file, indent = 4)
        print("Poses saved to 'ee_poses.json'")
    
    def load_transformation_from_json(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
        return np.array(data["matrix"])

    def save_Tee_to_camera_to_json(self):
        # calibration result
        matrix = [
            [-0.72764462, -0.68585163, -0.01186786, 0.05192262],
            [0.68595005, -0.72747031, -0.01610833, 0.00849046],
            [0.00241441, -0.01986189, 0.99979982, 0.0033307],
            [0, 0, 0, 1]
        ]
        

        data = {    
            "matrix": matrix
        }

        with open("Tee_to_camera.json", "w") as json_file:
            json.dump(data, json_file, indent=4)

    def on_press(self, key):
        if not hasattr(self, 'instructions_printed'):
            print("Press 'Enter' to record the current pose. Press 'Esc' to stop recording and exit.")
            self.instructions_printed = True
        # If Enter is pressed, record current pose
        if key == keyboard.Key.enter:
            ee_pose = self.robot.current_pose()
            
            # Extract the translation component from the Affine object
            translation = ee_pose.translation()

            # Extract the rotation component as a 4x4 matrix from the Affine object
            rotation_matrix = ee_pose.rotation()
            print('rotation_matrix',rotation_matrix)
            transformation_matrix = np.identity(4)
            transformation_matrix[:3, :3] = rotation_matrix
            transformation_matrix[:3, 3] = translation
            if not hasattr(self, 'saved_poses'):
                self.saved_poses = {"matrix": []}
            self.saved_poses["matrix"].append(transformation_matrix.tolist())
            
            # Convert the rotation matrix to a quaternion
            print(f"Recorded Pose #{len(self.saved_poses)}")
            print(np.array(transformation_matrix)) 


        # Stop listener
        if key == keyboard.Key.esc:
            print("Recording stopped. Exiting...")
            self.save_poses_to_json()
            self.collecting = False
            return False
        
    def on_release(self, key):
        pass

    def record_waypoint(self):
        threading.Thread(target=self.keyboard_listener, daemon=True).start()

        # Main loop for recording waypoints
        try:
            while self.collecting:
                # The loop will run until ESC is pressed.

                pass
        finally:
            print("\nSaving poses...")
            self.save_poses_to_json()

    def calculate_Tmarker_to_ee(self):
        # Load saved end effector poses
        with open("ee_poses.json", "r") as file:
            ee_poses = json.load(file)["matrix"]

        # Load Tee_to_camera
        with open("Tee_to_camera.json", "r") as file:
            Tee_to_camera_data = json.load(file)
            Tee_to_camera = np.array(Tee_to_camera_data["matrix"])

        # Load Tcamera_to_marker_1
        with open("Tcamera_to_marker_1.json", "r") as file:
            Tcamera_to_marker_1_data = json.load(file)
            Tcamera_to_marker_1 = np.array(Tcamera_to_marker_1_data["matrix"])

        # Get the first ee pose (Tbase_to_ee_1)
        Tbase_to_ee_1 = np.array(ee_poses[0])

        Tmarker_to_ee = []

        # Calculate Tmarker_to_ee_i for each ee pose
        for Tbase_to_ee_i in ee_poses:
            Tbase_to_ee_i = np.array(Tbase_to_ee_i)
            Tmarker_to_ee_i = self.multiply_transforms(
                self.invert_transform(Tbase_to_ee_i),
                Tbase_to_ee_1,
                Tee_to_camera,
                Tcamera_to_marker_1
            )
            Tmarker_to_ee.append(self.invert_transform(Tmarker_to_ee_i).tolist())

        with open("Tmarker_to_ee.json", "w") as file:
            json.dump({"matrix": Tmarker_to_ee}, file, indent=4)

    def save_Tbase_to_ee(self):
        # The code to send data to KUKA goes here.
        # ideally, i need to send Tbase_to_ee
        # get Tcamera_to_marker_c (c represents current), then 
        # calculate Tbase_to_marker_c = Tbase_to_ee_1 (this is fixed config for checking the marker) * Tee_to_camera * Tcamera_to_marker_c
        # finally calculate Tbase_to_ee = Tbase_to_marker_c * Tmarker_to_ee (saved json from save_Tcamera_to_marker_c)
        
        # Load the saved transformations
        with open("ee_poses.json", "r") as file:
            matrices = json.load(file)["matrix"]
        
        Tbase_to_ee_1 = np.array(matrices[0])
        Tee_to_camera = self.load_transformation_from_json('Tee_to_camera.json') # E2H matrix
        Tmarker_to_ee = self.load_transformation_from_json('Tmarker_to_ee.json') # series of ee poses in terms of the marker
        Tcamera_to_marker_c = self.load_transformation_from_json('Tcamera_to_marker_c.json')

        # Load first saved ee pose
        # Tbase_to_ee_1 = self.load_transformation_from_json('Tbase_to_ee_1.json')

        # Calculate Tbase_to_marker_c
        Tbase_to_marker_c = self.multiply_transforms(Tbase_to_ee_1, Tee_to_camera, Tcamera_to_marker_c)

        # Calculate final Tbase_to_ee
        Tbase_to_ee = np.matmul(Tbase_to_marker_c, Tmarker_to_ee)
        with open("Tbase_to_ee.json", "w") as file:
            json.dump({"matrix": Tbase_to_ee.tolist()}, file, indent=4)
    
    def execute_saved_waypoints(self):
        with open("Tbase_to_ee.json", "r") as file:
            matrices = json.load(file)["matrix"]
        # gripper command here
        # gripper = frankx.Gripper('10.8.11.204')
        for i, matrix_values in enumerate(matrices):
            matrix = np.array(matrix_values)

            translation = matrix[:3, 3]
            rotation_matrix_4x4 = np.eye(4)
            rotation_matrix_4x4[:3, :3] = matrix[:3, :3]
            quaternion = tft.quaternion_from_matrix(rotation_matrix_4x4)

            self.robot.move(LinearMotion(Affine(*translation, quaternion[3], quaternion[0], quaternion[1], quaternion[2])))
            time.sleep(2)
            
            # if i == 8:
            #     gripper.grasp(0.04, force=40)

    def go_to_check(self):
        with open("ee_poses.json", "r") as file:
            matrices = json.load(file)["matrix"]
        
        matrix = np.array(matrices[0])
        translation = matrix[:3, 3]
        rotation_matrix_4x4 = np.eye(4)
        rotation_matrix_4x4[:3, :3] = matrix[:3, :3]
        # translation = tuple(matrix[:3, 3])
        # rotation_matrix = matrix[:3, :3]
        quaternion = tft.quaternion_from_matrix(rotation_matrix_4x4)
        self.robot.move(LinearMotion(Affine(*translation, quaternion[3], quaternion[0], quaternion[1], quaternion[2])))
        time.sleep(1)


if __name__ == "__main__":

    franka_qr = franka_marker()
   
    # Here is for testing the script step by step
    args = parse_arguments()
    if args.mode == "save_Tcamera_to_marker_1":
        franka_qr.save_Tcamera_to_marker_1()
    elif args.mode == "record_waypoint":
        franka_qr.record_waypoint()
    elif args.mode == "calculate_Tmarker":
        franka_qr.calculate_Tmarker_to_ee()
    elif args.mode == "save_current_Tcamera_to_marker":
        franka_qr.save_Tcamera_to_marker_c()
    elif args.mode == "save_Tbase_to_ee":
        franka_qr.save_Tbase_to_ee()
    elif args.mode == "execute_saved_waypoints":
        franka_qr.execute_saved_waypoints()
    elif args.mode == "save_eye_hand":
        franka_qr.save_Tee_to_camera_to_json()
    elif args.mode == "go_to_check":
        franka_qr.go_to_check()

    else:
        print("Unknown mode. Exiting.")