#!/usr/bin/python
# -*- coding: UTF-8 -*-
# Control UR5 by using MoveIt API

from __future__ import print_function
import time
from math import pi
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class MoveGrasp(object):
  def __init__(self):
    super(MoveGrasp, self).__init__()
    # 初始化`moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化node
    rospy.init_node('move_grasp', anonymous=True)
    # 实例化机器人控制器
    robot = moveit_commander.RobotCommander()
    # 在 .srdf 文件里得到 group 名称列表
    arm = moveit_commander.MoveGroupCommander("arm")
    gripper = moveit_commander.MoveGroupCommander("gripper")

    # 设置目标位置所使用的参考坐标系
    reference_frame = 'world'

    # group 名称列表:
    group_names = robot.get_group_names()
    print ("============ Robot Groups:", robot.get_group_names())
    # 机器人状态:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    print (" ")
    
    # Misc variables
    self.robot = robot
    self.arm = arm
    self.gripper = gripper
    self.group_names = group_names
    self.reference_frame = reference_frame

 

  def call_back(self): # 根据末端位姿规划
    arm = self.arm
    grp = self.gripper
    
    arm.set_pose_reference_frame(self.reference_frame)
    grp.set_pose_reference_frame(self.reference_frame)

    # 手臂,手爪初始化
    grp.set_named_target('open')
    grp.go(wait=True)
    grp.stop()
    grp.clear_pose_targets()
    print("Step 1")

    arm.set_named_target('home')
    arm.go(wait=True)
    print("Step 2")
    
# # # 末端的目标位姿（四元数）

    x_bias = -0.030
    y_bias = 0.052
    z_bias = 0.007

    pose_goal = arm.get_current_pose()
    pose_goal.pose.position.x = 0.4 
    pose_goal.pose.position.y = 0.0 
    arm.set_pose_target(pose_goal)
    arm.go(wait=True)
    print("Step 3")

    pose = arm.get_current_pose().pose
    pose.position.z -= 0.12
    arm.set_pose_target(pose)
    arm.go(wait=True)
    print("Step 4")

    # arm.stop() # 停止保证没有残余运动
    # arm.clear_pose_targets()

    grp.set_named_target('close')
    grp.go(wait=True)
    print("Step 5")

    pose = arm.get_current_pose().pose
    pose.position.z += 0.3
    arm.set_pose_target(pose)
    arm.go(wait=True)
    print("Step 6")

    pose_goal = arm.get_current_pose()
    pose_goal.pose.position.x = 0.5
    pose_goal.pose.position.y = 0.2
    arm.set_pose_target(pose_goal)
    arm.go(wait=True)
    print("Step 7")

    grp.set_named_target('open')
    grp.go(wait=True)
    grp.stop()
    grp.clear_pose_targets()
    print("Step 8")



    current_pose = arm.get_current_pose().pose
    print("New current pose: ", current_pose)



def main(args):
    try:
      move_demo = MoveGrasp()
      print  ("============ Press `Enter` to execute a movement using a pose goal ...")
      raw_input()
      move_demo.call_back()
      rospy.spin()
    except rospy.ROSInterruptException:
      print ("Shutting down")
        
if __name__ == '__main__':
    main(sys.argv)
