#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory, PlanningScene, ObjectColor
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItBinpickingDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('binpicking_demo')

        # 初始化场景对象
        scene = PlanningSceneInterface()
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
        
        # 等待场景准备就绪
        rospy.sleep(1)
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
        
        # 设置场景物体的名称
        table_id = 'table'

        box1_1_id = 'box1_1'
        box1_2_id = 'box1_2'
        box1_3_id = 'box1_3'
        box1_4_id = 'box1_4'
        box1_5_id = 'box1_5'

        box2_1_id = 'box2_1'
        box2_2_id = 'box2_2'
        box2_3_id = 'box2_3'
        box2_4_id = 'box2_4'
        box2_5_id = 'box2_5'

        box3_1_id = 'box3_1'
        box3_2_id = 'box3_2'
        box3_3_id = 'box3_3'
        box3_4_id = 'box3_4'
        box3_5_id = 'box3_5'

        box4_1_id = 'box4_1'
        box4_2_id = 'box4_2'
        box4_3_id = 'box4_3'
        box4_4_id = 'box4_4'
        box4_5_id = 'box4_5'

        box5_1_id = 'box5_1'
        box5_2_id = 'box5_2'
        box5_3_id = 'box5_3'
        box5_4_id = 'box5_4'
        box5_5_id = 'box5_5'

        box6_1_id = 'box6_1'
        box6_2_id = 'box6_2'
        box6_3_id = 'box6_3'
        box6_4_id = 'box6_4'
        box6_5_id = 'box6_5'

        tool_rack_id = 'tool_rack'

        place_zone_id = 'place_zone'

        rear_frame_id = 'rear_frame'

        top_frame_id = 'top_frame'

        left_frame_id = 'left_frame'

        right_frame_id = 'right_frame'

        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)

        scene.remove_world_object(box1_1_id)
        scene.remove_world_object(box1_2_id)
        scene.remove_world_object(box1_3_id)
        scene.remove_world_object(box1_4_id)
        scene.remove_world_object(box1_5_id)

        scene.remove_world_object(box2_1_id)
        scene.remove_world_object(box2_2_id)
        scene.remove_world_object(box2_3_id)
        scene.remove_world_object(box2_4_id)
        scene.remove_world_object(box2_5_id)

        scene.remove_world_object(box3_1_id)
        scene.remove_world_object(box3_2_id)
        scene.remove_world_object(box3_3_id)
        scene.remove_world_object(box3_4_id)
        scene.remove_world_object(box3_5_id)

        scene.remove_world_object(box4_1_id)
        scene.remove_world_object(box4_2_id)
        scene.remove_world_object(box4_3_id)
        scene.remove_world_object(box4_4_id)
        scene.remove_world_object(box4_5_id)

        scene.remove_world_object(box5_1_id)
        scene.remove_world_object(box5_2_id)
        scene.remove_world_object(box5_3_id)
        scene.remove_world_object(box5_4_id)
        scene.remove_world_object(box5_5_id)

        scene.remove_world_object(box6_1_id)
        scene.remove_world_object(box6_2_id)
        scene.remove_world_object(box6_3_id)
        scene.remove_world_object(box6_4_id)
        scene.remove_world_object(box6_5_id)

        scene.remove_world_object(tool_rack_id)

        scene.remove_world_object(place_zone_id)

        scene.remove_world_object(rear_frame_id)

        scene.remove_world_object(top_frame_id)

        scene.remove_world_object(left_frame_id)

        scene.remove_world_object(right_frame_id)

        rospy.sleep(1)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)

        # 设置物体的三维尺寸
        table_size = [2.5, 1.4, 0.01]

        box1_1_size = [0.325, 0.0175, 0.21]
        box1_2_size = [0.325, 0.0175, 0.21]
        box1_3_size = [0.0175, 0.4, 0.21]
        box1_4_size = [0.0175, 0.4, 0.21]
        box1_5_size = [0.29, 0.4, 0.015]

        box2_1_size = [0.325, 0.0175, 0.21]
        box2_2_size = [0.325, 0.0175, 0.21]
        box2_3_size = [0.0175, 0.4, 0.21]
        box2_4_size = [0.0175, 0.4, 0.21]
        box2_5_size = [0.29, 0.4, 0.015]

        box3_1_size = [0.325, 0.0175, 0.21]
        box3_2_size = [0.325, 0.0175, 0.21]
        box3_3_size = [0.0175, 0.4, 0.21]
        box3_4_size = [0.0175, 0.4, 0.21]
        box3_5_size = [0.29, 0.4, 0.015]

        box4_1_size = [0.325, 0.0175, 0.21]
        box4_2_size = [0.325, 0.0175, 0.21]
        box4_3_size = [0.0175, 0.4, 0.21]
        box4_4_size = [0.0175, 0.4, 0.21]
        box4_5_size = [0.29, 0.4, 0.015]

        box5_1_size = [0.325, 0.0175, 0.21]
        box5_2_size = [0.325, 0.0175, 0.21]
        box5_3_size = [0.0175, 0.4, 0.21]
        box5_4_size = [0.0175, 0.4, 0.21]
        box5_5_size = [0.29, 0.4, 0.015]

        box6_1_size = [0.325, 0.0175, 0.21]
        box6_2_size = [0.325, 0.0175, 0.21]
        box6_3_size = [0.0175, 0.4, 0.21]
        box6_4_size = [0.0175, 0.4, 0.21]
        box6_5_size = [0.29, 0.4, 0.015]

        tool_rack_size = [0.16, 0.4, 0.32]

        place_zone_size = [0.39, 0.3, 0.2]

        rear_frame_size = [2.41, 0.045, 1.015]

        top_frame_size = [2.5, 0.8775, 0.02]

        left_frame_size = [0.045, 0.636, 1.015]

        right_frame_size = [0.045, 0.636, 1.015]

        # 将物体加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = -0.2
        table_pose.pose.position.z = -0.02
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        box1_1_pose = PoseStamped()
        box1_1_pose.header.frame_id = reference_frame
        box1_1_pose.pose.position.x = 0.8875
        box1_1_pose.pose.position.y = -0.27375
        box1_1_pose.pose.position.z = 0.105
        box1_1_pose.pose.orientation.w = 1.0
        scene.add_box(box1_1_id, box1_1_pose, box1_1_size)
        
        box1_2_pose = PoseStamped()
        box1_2_pose.header.frame_id = reference_frame
        box1_2_pose.pose.position.x = 0.8875
        box1_2_pose.pose.position.y = -0.69125
        box1_2_pose.pose.position.z = 0.105
        box1_2_pose.pose.orientation.w = 1.0
        scene.add_box(box1_2_id, box1_2_pose, box1_2_size)

        box1_3_pose = PoseStamped()
        box1_3_pose.header.frame_id = reference_frame
        box1_3_pose.pose.position.x = 1.04125
        box1_3_pose.pose.position.y = -0.4825
        box1_3_pose.pose.position.z = 0.105
        box1_3_pose.pose.orientation.w = 1.0
        scene.add_box(box1_3_id, box1_3_pose, box1_3_size)

        box1_4_pose = PoseStamped()
        box1_4_pose.header.frame_id = reference_frame
        box1_4_pose.pose.position.x = 0.73375
        box1_4_pose.pose.position.y = -0.4825
        box1_4_pose.pose.position.z = 0.105
        box1_4_pose.pose.orientation.w = 1.0
        scene.add_box(box1_4_id, box1_4_pose, box1_4_size)

        box1_5_pose = PoseStamped()
        box1_5_pose.header.frame_id = reference_frame
        box1_5_pose.pose.position.x = 0.8875
        box1_5_pose.pose.position.y = -0.4825
        box1_5_pose.pose.position.z = 0.0075
        box1_5_pose.pose.orientation.w = 1.0
        scene.add_box(box1_5_id, box1_5_pose, box1_5_size)

        box2_1_pose = PoseStamped()
        box2_1_pose.header.frame_id = reference_frame
        box2_1_pose.pose.position.x = 0.5325
        box2_1_pose.pose.position.y = -0.27375
        box2_1_pose.pose.position.z = 0.105
        box2_1_pose.pose.orientation.w = 1.0
        scene.add_box(box2_1_id, box2_1_pose, box2_1_size)
        
        box2_2_pose = PoseStamped()
        box2_2_pose.header.frame_id = reference_frame
        box2_2_pose.pose.position.x = 0.5325
        box2_2_pose.pose.position.y = -0.69125
        box2_2_pose.pose.position.z = 0.105
        box2_2_pose.pose.orientation.w = 1.0
        scene.add_box(box2_2_id, box2_2_pose, box2_2_size)

        box2_3_pose = PoseStamped()
        box2_3_pose.header.frame_id = reference_frame
        box2_3_pose.pose.position.x = 0.68625
        box2_3_pose.pose.position.y = -0.4825
        box2_3_pose.pose.position.z = 0.105
        box2_3_pose.pose.orientation.w = 1.0
        scene.add_box(box2_3_id, box2_3_pose, box2_3_size)

        box2_4_pose = PoseStamped()
        box2_4_pose.header.frame_id = reference_frame
        box2_4_pose.pose.position.x = 0.37875
        box2_4_pose.pose.position.y = -0.4825
        box2_4_pose.pose.position.z = 0.105
        box2_4_pose.pose.orientation.w = 1.0
        scene.add_box(box2_4_id, box2_4_pose, box2_4_size)

        box2_5_pose = PoseStamped()
        box2_5_pose.header.frame_id = reference_frame
        box2_5_pose.pose.position.x = 0.5325
        box2_5_pose.pose.position.y = -0.4825
        box2_5_pose.pose.position.z = 0.0075
        box2_5_pose.pose.orientation.w = 1.0
        scene.add_box(box2_5_id, box2_5_pose, box2_5_size)
                
        box3_1_pose = PoseStamped()
        box3_1_pose.header.frame_id = reference_frame
        box3_1_pose.pose.position.x = 0.1775
        box3_1_pose.pose.position.y = -0.27375
        box3_1_pose.pose.position.z = 0.105
        box3_1_pose.pose.orientation.w = 1.0
        scene.add_box(box3_1_id, box3_1_pose, box3_1_size)
        
        box3_2_pose = PoseStamped()
        box3_2_pose.header.frame_id = reference_frame
        box3_2_pose.pose.position.x = 0.1775
        box3_2_pose.pose.position.y = -0.69125
        box3_2_pose.pose.position.z = 0.105
        box3_2_pose.pose.orientation.w = 1.0
        scene.add_box(box3_2_id, box3_2_pose, box3_2_size)

        box3_3_pose = PoseStamped()
        box3_3_pose.header.frame_id = reference_frame
        box3_3_pose.pose.position.x = 0.33125
        box3_3_pose.pose.position.y = -0.4825
        box3_3_pose.pose.position.z = 0.105
        box3_3_pose.pose.orientation.w = 1.0
        scene.add_box(box3_3_id, box3_3_pose, box3_3_size)

        box3_4_pose = PoseStamped()
        box3_4_pose.header.frame_id = reference_frame
        box3_4_pose.pose.position.x = 0.02375
        box3_4_pose.pose.position.y = -0.4825
        box3_4_pose.pose.position.z = 0.105
        box3_4_pose.pose.orientation.w = 1.0
        scene.add_box(box3_4_id, box3_4_pose, box3_4_size)

        box3_5_pose = PoseStamped()
        box3_5_pose.header.frame_id = reference_frame
        box3_5_pose.pose.position.x = 0.1775
        box3_5_pose.pose.position.y = -0.4825
        box3_5_pose.pose.position.z = 0.0075
        box3_5_pose.pose.orientation.w = 1.0
        scene.add_box(box3_5_id, box3_5_pose, box3_5_size)

        box4_1_pose = PoseStamped()
        box4_1_pose.header.frame_id = reference_frame
        box4_1_pose.pose.position.x = -0.1775
        box4_1_pose.pose.position.y = -0.27375
        box4_1_pose.pose.position.z = 0.105
        box4_1_pose.pose.orientation.w = 1.0
        scene.add_box(box4_1_id, box4_1_pose, box4_1_size)
        
        box4_2_pose = PoseStamped()
        box4_2_pose.header.frame_id = reference_frame
        box4_2_pose.pose.position.x = -0.1775
        box4_2_pose.pose.position.y = -0.69125
        box4_2_pose.pose.position.z = 0.105
        box4_2_pose.pose.orientation.w = 1.0
        scene.add_box(box4_2_id, box4_2_pose, box4_2_size)

        box4_3_pose = PoseStamped()
        box4_3_pose.header.frame_id = reference_frame
        box4_3_pose.pose.position.x = -0.33125
        box4_3_pose.pose.position.y = -0.4825
        box4_3_pose.pose.position.z = 0.105
        box4_3_pose.pose.orientation.w = 1.0
        scene.add_box(box4_3_id, box4_3_pose, box4_3_size)

        box4_4_pose = PoseStamped()
        box4_4_pose.header.frame_id = reference_frame
        box4_4_pose.pose.position.x = -0.02375
        box4_4_pose.pose.position.y = -0.4825
        box4_4_pose.pose.position.z = 0.105
        box4_4_pose.pose.orientation.w = 1.0
        scene.add_box(box4_4_id, box4_4_pose, box4_4_size)

        box4_5_pose = PoseStamped()
        box4_5_pose.header.frame_id = reference_frame
        box4_5_pose.pose.position.x = -0.1775
        box4_5_pose.pose.position.y = -0.4825
        box4_5_pose.pose.position.z = 0.0075
        box4_5_pose.pose.orientation.w = 1.0
        scene.add_box(box4_5_id, box4_5_pose, box4_5_size)

        box5_1_pose = PoseStamped()
        box5_1_pose.header.frame_id = reference_frame
        box5_1_pose.pose.position.x = -0.5325
        box5_1_pose.pose.position.y = -0.27375
        box5_1_pose.pose.position.z = 0.105
        box5_1_pose.pose.orientation.w = 1.0
        scene.add_box(box5_1_id, box5_1_pose, box5_1_size)
        
        box5_2_pose = PoseStamped()
        box5_2_pose.header.frame_id = reference_frame
        box5_2_pose.pose.position.x = -0.5325
        box5_2_pose.pose.position.y = -0.69125
        box5_2_pose.pose.position.z = 0.105
        box5_2_pose.pose.orientation.w = 1.0
        scene.add_box(box5_2_id, box5_2_pose, box5_2_size)

        box5_3_pose = PoseStamped()
        box5_3_pose.header.frame_id = reference_frame
        box5_3_pose.pose.position.x = -0.68625
        box5_3_pose.pose.position.y = -0.4825
        box5_3_pose.pose.position.z = 0.105
        box5_3_pose.pose.orientation.w = 1.0
        scene.add_box(box5_3_id, box5_3_pose, box5_3_size)

        box5_4_pose = PoseStamped()
        box5_4_pose.header.frame_id = reference_frame
        box5_4_pose.pose.position.x = -0.37875
        box5_4_pose.pose.position.y = -0.4825
        box5_4_pose.pose.position.z = 0.105
        box5_4_pose.pose.orientation.w = 1.0
        scene.add_box(box5_4_id, box5_4_pose, box5_4_size)

        box5_5_pose = PoseStamped()
        box5_5_pose.header.frame_id = reference_frame
        box5_5_pose.pose.position.x = -0.5325
        box5_5_pose.pose.position.y = -0.4825
        box5_5_pose.pose.position.z = 0.0075
        box5_5_pose.pose.orientation.w = 1.0
        scene.add_box(box5_5_id, box5_5_pose, box5_5_size)

        box6_1_pose = PoseStamped()
        box6_1_pose.header.frame_id = reference_frame
        box6_1_pose.pose.position.x = -0.8875
        box6_1_pose.pose.position.y = -0.27375
        box6_1_pose.pose.position.z = 0.105
        box6_1_pose.pose.orientation.w = 1.0
        scene.add_box(box6_1_id, box6_1_pose, box6_1_size)
        
        box6_2_pose = PoseStamped()
        box6_2_pose.header.frame_id = reference_frame
        box6_2_pose.pose.position.x = -0.8875
        box6_2_pose.pose.position.y = -0.69125
        box6_2_pose.pose.position.z = 0.105
        box6_2_pose.pose.orientation.w = 1.0
        scene.add_box(box6_2_id, box6_2_pose, box6_2_size)

        box6_3_pose = PoseStamped()
        box6_3_pose.header.frame_id = reference_frame
        box6_3_pose.pose.position.x = -1.04125
        box6_3_pose.pose.position.y = -0.4825
        box6_3_pose.pose.position.z = 0.105
        box6_3_pose.pose.orientation.w = 1.0
        scene.add_box(box6_3_id, box6_3_pose, box6_3_size)

        box6_4_pose = PoseStamped()
        box6_4_pose.header.frame_id = reference_frame
        box6_4_pose.pose.position.x = -0.73375
        box6_4_pose.pose.position.y = -0.4825
        box6_4_pose.pose.position.z = 0.105
        box6_4_pose.pose.orientation.w = 1.0
        scene.add_box(box6_4_id, box6_4_pose, box6_4_size)

        box6_5_pose = PoseStamped()
        box6_5_pose.header.frame_id = reference_frame
        box6_5_pose.pose.position.x = -0.8875
        box6_5_pose.pose.position.y = -0.4825
        box6_5_pose.pose.position.z = 0.0075
        box6_5_pose.pose.orientation.w = 1.0
        scene.add_box(box6_5_id, box6_5_pose, box6_5_size)

        tool_rack_pose = PoseStamped()
        tool_rack_pose.header.frame_id = reference_frame
        tool_rack_pose.pose.position.x = 0.9449
        tool_rack_pose.pose.position.y = 0.18
        tool_rack_pose.pose.position.z = 0.145
        tool_rack_pose.pose.orientation.w = 1.0
        scene.add_box(tool_rack_id, tool_rack_pose, tool_rack_size)

        place_zone_pose = PoseStamped()
        place_zone_pose.header.frame_id = reference_frame
        place_zone_pose.pose.position.x = -0.8101
        place_zone_pose.pose.position.y = 0.18
        place_zone_pose.pose.position.z = 0.085
        place_zone_pose.pose.orientation.w = 1.0
        scene.add_box(place_zone_id, place_zone_pose, place_zone_size)

        rear_frame_pose = PoseStamped()
        rear_frame_pose.header.frame_id = reference_frame
        rear_frame_pose.pose.position.x = 0
        rear_frame_pose.pose.position.y = -0.87815
        rear_frame_pose.pose.position.z = 0.4925
        rear_frame_pose.pose.orientation.w = 1.0
        scene.add_box(rear_frame_id, rear_frame_pose, rear_frame_size)

        top_frame_pose = PoseStamped()
        top_frame_pose.header.frame_id = reference_frame
        top_frame_pose.pose.position.x = 0
        top_frame_pose.pose.position.y = -0.46122
        top_frame_pose.pose.position.z = 1.01
        top_frame_pose.pose.orientation.w = 1.0
        scene.add_box(top_frame_id, top_frame_pose, top_frame_size)

        left_frame_pose = PoseStamped()
        left_frame_pose.header.frame_id = reference_frame
        left_frame_pose.pose.position.x = 1.2274
        left_frame_pose.pose.position.y = -0.582
        left_frame_pose.pose.position.z = 0.4925
        left_frame_pose.pose.orientation.w = 1.0
        scene.add_box(left_frame_id, left_frame_pose, left_frame_size)

        right_frame_pose = PoseStamped()
        right_frame_pose.header.frame_id = reference_frame
        right_frame_pose.pose.position.x = -1.2274
        right_frame_pose.pose.position.y = -0.582
        right_frame_pose.pose.position.z = 0.4925
        right_frame_pose.pose.orientation.w = 1.0
        scene.add_box(right_frame_id, right_frame_pose, right_frame_size)

        # 将桌子设置成红色，box设置成橙色
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box1_2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box1_3_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box1_4_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box1_5_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_3_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_4_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_5_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box3_1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box3_2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box3_3_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box3_4_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box3_5_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box4_1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box4_2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box4_3_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box4_4_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box4_5_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box5_1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box5_2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box5_3_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box5_4_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box5_5_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box6_1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box6_2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box6_3_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box6_4_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box6_5_id, 0.8, 0.4, 0, 1.0)
        self.setColor(tool_rack_id, 0.8, 0.4, 0, 1.0)
        self.setColor(place_zone_id, 0.8, 0.4, 0, 1.0)
        
        # 将场景中的颜色设置发布
        self.sendColors()


        # 设置机械臂工作空间中的目标位姿:<<物料盒正上方位置>>；位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose_01 = PoseStamped()
        target_pose_01.header.frame_id = reference_frame
        target_pose_01.header.stamp = rospy.Time.now()
        target_pose_01.pose.position.x = -0.1975
        target_pose_01.pose.position.y = -0.7470
        target_pose_01.pose.position.z = 0.29651
        # target_pose_01.pose.orientation.x = -0.00212
        # target_pose_01.pose.orientation.y = -0.99999
        # target_pose_01.pose.orientation.z = -0.00188
        # target_pose_01.pose.orientation.w = 0.00189
        
        q = quaternion_from_euler(3.13785, -0.00378, 3.13737)
        target_pose_01.pose.orientation.x = q[0]
        target_pose_01.pose.orientation.y = q[1]
        target_pose_01.pose.orientation.z = q[2]
        target_pose_01.pose.orientation.w = q[3]
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose_01, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
                          
        # 按照规划的运动路径控制机械臂运动
        arm.go()
        rospy.sleep(1)

        # 设置机械臂工作空间中的目标位姿:<<抓取点>>；位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose_02 = PoseStamped()
        target_pose_02.header.frame_id = reference_frame
        target_pose_02.header.stamp = rospy.Time.now()     
        target_pose_02.pose.position.x = -0.1975
        target_pose_02.pose.position.y = -0.7470
        target_pose_02.pose.position.z = 0.09
        # target_pose_02.pose.orientation.x = -0.34758
        # target_pose_02.pose.orientation.y = 0.93725
        # target_pose_02.pose.orientation.z = 0.02612
        # target_pose_02.pose.orientation.w = 0.00769

        q = quaternion_from_euler(0.9824, 0.01322, -0.14639)
        target_pose_02.pose.orientation.x = q[0]
        target_pose_02.pose.orientation.y = q[1]
        target_pose_02.pose.orientation.z = q[2]
        target_pose_02.pose.orientation.w = q[3]
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose_02, end_effector_link)
        # arm.set_pose_target([0.16984, -0.49628, 0.03084, 3.09793, 0.032581, -2.43204], end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()

        # 按照规划的运动路径控制机械臂运动
        arm.go()
        # print(traj)
        rospy.sleep(1)
	    
        # 设置机械臂工作空间中的目标位姿:<<物料盒正上方位置>>；位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose_03 = PoseStamped()
        target_pose_03.header.frame_id = reference_frame
        target_pose_03.header.stamp = rospy.Time.now()
        target_pose_03.pose.position.x = 0.16984
        target_pose_03.pose.position.y = -0.49299
        target_pose_03.pose.position.z = 0.29651
        # target_pose_03.pose.orientation.x = -0.00212
        # target_pose_03.pose.orientation.y = -0.99999
        # target_pose_03.pose.orientation.z = -0.00188
        # target_pose_03.pose.orientation.w = 0.00189
        
        q = quaternion_from_euler(3.13785, -0.00378, 3.13737)
        target_pose_03.pose.orientation.x = q[0]
        target_pose_03.pose.orientation.y = q[1]
        target_pose_03.pose.orientation.z = q[2]
        target_pose_03.pose.orientation.w = q[3]
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose_03, end_effector_link)
        
        # 规划运动路径
        traj = arm.plan()
        # print(traj)

        # 按照规划的运动路径控制机械臂运动
        arm.go()
        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()
	

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # 设置场景物体的颜色
    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItBinpickingDemo()

    
    
