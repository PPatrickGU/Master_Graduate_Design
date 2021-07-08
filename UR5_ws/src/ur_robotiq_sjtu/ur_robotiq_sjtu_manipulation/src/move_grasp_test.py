#!/usr/bin/python
# -*- coding: UTF-8 -*-
# Control UR5 by using MoveIt API

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

class MoveGrasp(object):
  def __init__(self):
    super(MoveGrasp, self).__init__()
    # 初始化`moveit_commander`
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化node
    rospy.init_node('grasp_object', anonymous=True)
    # 实例化机器人控制器
    robot = moveit_commander.RobotCommander()
    # 实例化机器人工作场景
    scene = moveit_commander.PlanningSceneInterface()
    # 在 .srdf 文件里得到 group 名称列表
    arm = moveit_commander.MoveGroupCommander("arm")
    gripper = moveit_commander.MoveGroupCommander("gripper")
    # group 名称列表:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    # 机器人状态:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    
    # Misc variables
    self.robot = robot
    self.scene = scene
    self.arm = arm
    self.gripper = gripper
    self.group_names = group_names

  def call_back(self): # 根据末端位姿规划
    arm = self.arm
    grp = self.gripper

    # 手臂,手爪初始化
    grp.set_named_target('open')
    grp.go(wait=True)
    grp.stop()
    grp.clear_pose_targets()
    
    arm.set_named_target('home')
    arm.go(wait=True)
    arm.stop() # 停止保证没有残余运动
    arm.clear_pose_targets()

    print(1)
    time.sleep(3)

    # 设定目标位置
    current_pose = arm.get_current_pose().pose
    print("Current pose: ", current_pose)
    # 末端的目标位姿（四元数）
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 1.2
    # pose_goal.position.y = 0
    # pose_goal.position.z = 1.2
    # arm.set_pose_target(pose_goal)

    arm.set_named_target('down')
    arm.go(wait=True)
    arm.stop() # 停止保证没有残余运动
    arm.clear_pose_targets()

    grp.set_named_target('close')
    grp.go(wait=True)
    grp.stop()
    grp.clear_pose_targets()

    print(2)
    time.sleep(3)




    pose = arm.get_current_pose().pose
    pose.position.z += 1
    arm.set_pose_target(pose)
    arm.go(wait=True)
    arm.stop() 
    arm.clear_pose_targets()

    arm.set_named_target('home')
    arm.go(wait=True)
    arm.stop() # 停止保证没有残余运动
    arm.clear_pose_targets()

    grp.set_named_target('open')
    grp.go(wait=True)
    grp.stop()
    grp.clear_pose_targets()

    print(2)
    time.sleep(3)

    print(3)
    time.sleep(2)


    current_pose = arm.get_current_pose().pose
    print("New current pose: ", current_pose)



def main(args):
    try:
      move_demo = MoveGrasp()
      print "============ Press `Enter` to execute a movement using a pose goal ..."
      raw_input()
      move_demo.call_back()
      rospy.spin()
    except rospy.ROSInterruptException:
      print ("Shutting down")
        
if __name__ == '__main__':
    main(sys.argv)
