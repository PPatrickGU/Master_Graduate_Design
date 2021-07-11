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

class Grasp(object):
  def __init__(self):
    super(Grasp, self).__init__()
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
    
    for i in range(10):

      grp.set_named_target('open')
      grp.go(wait=True)
      grp.stop()
      grp.clear_pose_targets()
  
      time.sleep(3)

      grp.set_named_target('close')
      grp.go(wait=True)
      grp.stop()
      grp.clear_pose_targets()

      time.sleep(3)



def main(args):
    try:
      grasp_demo = Grasp()
      print "============ Press `Enter` to execute a movement using a pose goal ..."
      raw_input()
      grasp_demo.call_back()
      rospy.spin()
    except rospy.ROSInterruptException:
      print ("Shutting down")
        
if __name__ == '__main__':
    main(sys.argv)
