#!/usr/bin/python3
#-*- coding:UTF-8 -*-

# 导入ros的python依赖
import rospy
from ur_gripper_vision.msg import complex 

def my_publisher():
    # Ros 节点初始化
    rospy.init_node('my_publisher', anonymous=True)

    # 创建一个Publisher，发布名为/my_topic，消息类型为std_msgs/String，队列长度 10
    my_topic = rospy.Publisher('/my_topic', complex , queue_size=10)

    # 设置循环的频率
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 初始化 std_msgs/String 类型的 msg 对象
        complex_msg = complex()
        complex_msg.real = 10
        complex_msg.imaginary  = 10 

        # 发消息
        my_topic.publish(complex_msg)
        rospy.loginfo("Publish complex message[%d, %d]", 
				 complex_msg.real, complex_msg.imaginary)
        
        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    try:
        my_publisher()
    except rospy.ROSInterruptException:
        pass

