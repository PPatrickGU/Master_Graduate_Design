#!/usr/bin/python3
#-*- coding:UTF-8 -*-

import rospy
from ur_robotiq_sjtu_vision.msg  import complex 

def MyInfoCallback(msg):
    rospy.loginfo("Subscribe complex message[%d, %d]", 
				 msg.real, msg.imaginary)

def my_subcriber():
    # ROS节点初始化
    rospy.init_node('my_subcriber', anonymous=True)

    # 创建一个Subcriber，订阅名为 /my_topic 的topic，注册回调函数 MyInfoCallback()
    rospy.Subscriber("/my_topic", complex, MyInfoCallback)

    # 循环等待回调函数
    rospy.spin()


if __name__ == '__main__':
    my_subcriber()

