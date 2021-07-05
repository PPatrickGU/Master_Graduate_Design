#!/usr/bin/python
# -*- coding: UTF-8 -*-
# Rviz UR5
# Control UR5 by using MoveIt API

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


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)
  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  return True

class MoveGroup(object):
  """MoveGroupTutorial"""
  def __init__(self):
    super(MoveGroup, self).__init__()
    # 初始化`moveit_commander`和`rospy`节点
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_tutorial_ur5', anonymous=True)
    # 实例化机器人控制器
    robot = moveit_commander.RobotCommander()
    # 实例化机器人工作场景
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm" # See .srdf file to get available group names
    group = moveit_commander.MoveGroupCommander(group_name)
    # 生成publisher，在Rviz中可视化
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
    # reference frame:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame
    # name of the end-effector link:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link
    # list of all the groups:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()
    # entire state of the robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_up_state(self): # 关节角度规划
    group = self.group
    joint_goal = group.get_current_joint_values()
    print(type(joint_goal), joint_goal)
    # joint goal is a list of 7 elements : (x,y,z,qx,qy,qz,qw) can be composed of pose_msg
    joint_goal[0] = 0
    joint_goal[1] = -pi * 0.5
    joint_goal[2] = 0
    joint_goal[3] = -pi * 0.5
    joint_goal[4] = 0
    joint_goal[5] = 0    

    group.go(joint_goal, wait=True)
    group.stop() # 停止保证没有残余运动

    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self): # 根据末端位姿规划
    group = self.group
    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)
    # 末端的目标位姿
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop() # 停止保证没有残余运动
    # 在运动后清空目标位姿， 注意：无 clear_joint_value_targets()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    print("New current pose: ", current_pose)

    current_pose = self.group.get_current_pose().pose #测试
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1): # 笛卡尔，根据轨迹点进行运动规划
    group = self.group
    current_pose = group.get_current_pose().pose
    print("Current pose: ", current_pose)

    # Cartesian Paths
    # You can plan a Cartesian path directly by specifying a list of waypoints
    # for the end-effector to go through:
    waypoints = []

    wpose = group.get_current_pose().pose
    wpose.position.x = 0.2  
    wpose.position.y = 0.01
    wpose.position.z = 0.2
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= 0.2
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan): # 展示轨迹
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    # 功能和group.plan()相同
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state() # 以现在的轨迹作为起始点
    display_trajectory.trajectory.append(plan) # 把规划的轨迹加入路径
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan): # 按计算好的规划路径进行实际运动
    group = self.group
    group.execute(plan, wait=True)
    # **Note:** The robot's current joint state must be within some tolerance of the
    # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def main():
  try:
    print "============ Press `Enter` to sett up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    move_demo = MoveGroup()

    print "============ Press `Enter` to execute a movement using a joint state goal ..."
    raw_input()
    move_demo.go_to_up_state()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    move_demo.go_to_pose_goal()

    print "============ Press `Enter` to execute go to up state ..."
    raw_input()
    move_demo.go_to_up_state()

    # print "============ Press `Enter` to plan and display a Cartesian path ..."
    # raw_input()
    # cartesian_plan, fraction = move_demo.plan_cartesian_path()

    # print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    # raw_input()
    # move_demo.display_trajectory(cartesian_plan)

    # print "============ Press `Enter` to execute a saved path ..."
    # raw_input()
    # move_demo.execute_plan(cartesian_plan)
    # print "============ Python move demo complete!"

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")

