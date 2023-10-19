#!/usr/bin/env python

# /***************************************************************************

# 
# @package: franka_interface
# @metapackage: franka_ros_interface
# @author: Saif Sidhik <sxs1412@bham.ac.uk>
# 

# **************************************************************************/

# /***************************************************************************
# Copyright (c) 2019-2021, Saif Sidhik
 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# **************************************************************************/


"""
:info: 
   commands robot to move to neutral pose

"""

import rospy
from franka_interface import ArmInterface
from franka_gripper.msg import MoveActionGoal
from panda_robot import PandaArm

if __name__ == '__main__':
   rospy.init_node('gripper_publisher')

# Create a publisher
   r = ArmInterface()
   p = PandaArm()
   g = p.get_gripper()
   pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)
   # Create a MoveActionGoal message
   msg = MoveActionGoal()
   msg.goal.width = 0.00
   msg.goal.speed = 0.1

   

   #position.z += 0.05  # Move 5 cm (0.05 m)
   #pos[2] += 0.05
   #joint control
   p.move_to_joint_position([-8.48556818e-02, -8.88127666e-02, -6.59622769e-01, -1.57569726e+00, -4.82374882e-04,  2.15975946e+00,  4.36766917e-01])
   #print('pos1',p.ee_pose())
   rospy.sleep(1)
   g.open()
   rospy.sleep(1)
   #fetch current pose
   pos,ori = p.ee_pose()
   pos[0] -= 0.05
   p.move_to_cartesian_pose(pos,ori)
   rospy.sleep(1)
   pos,ori = p.ee_pose()
   pos[0] += 0.05
   p.move_to_cartesian_pose(pos,ori)
   g.close()
   #r.move_to_cartesian_pose(position, orientation)
   #p.move_to_cartesian_pose(pos,ori)
   #rospy.init_node("move_to_neutral_node")
   rospy.sleep(1)
   r.move_to_neutral()
   pub.publish(msg)
   # Print current joint angles
   print('robot_joint',r.joint_angles())
   # Check robot status
   #print(r.get_robot_status())
   pose = r.endpoint_pose()
   print('robot_pose',pose)



   
   # Move the end effector to the new pose
   
