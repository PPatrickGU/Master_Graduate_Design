#!/usr/bin/env python

from __future__ import print_function
import roslib
import sys
import rospy
import numpy as np
import ros_numpy 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import time
import message_filters
import math


class showImg:

	def callback(self, data_rgb, data_depth, data_points):
		try:
			# transform ROS rgb image to Opencv rgb image
			cv_rgb = self.bridge.imgmsg_to_cv2(data_rgb, "bgr8")
			cv_depth = self.bridge.imgmsg_to_cv2(data_depth, desired_encoding='passthrough')
		except CvBridgeError as e:
			print(e)
		ground_rgb = cv_rgb[475, 320]
		
		pc = ros_numpy.numpify(data_points)
        # pc is an array composed of 480*640 data in according to each pixel, each data is composed of 'x', 'y', 'z' and 'rgb' info. Get them use for example pc[100,100]['z']
		# according to camera_frame_optical frame, x(width) increase from left to right, y (-height) increase from high to low, z (distance) increase from near to far 
		
		# get the intensity image 
		cv_grey = cv2.cvtColor(cv_rgb,cv2.COLOR_RGB2GRAY)
		
		cv2.imshow('kinect', cv_rgb)
		cv2.waitKey(1000)
		# obstacle clustering
		cv_depth.flags.writeable = True
		pc.flags.writeable = True # only avaible for numpy 1.15
		
		msg_points = ros_numpy.msgify(PointCloud2, pc)
		msg_points.header = data_points.header
		msg_rgb = self.bridge.cv2_to_imgmsg(cv_rgb, "bgr8")
		msg_rgb.header = data_rgb.header
		msg_depth = self.bridge.cv2_to_imgmsg(cv_depth, encoding = "passthrough")
		msg_depth.header = data_depth.header
		
		try:
			self.image_rgb_pub.publish(msg_rgb)
			self.image_depth_pub.publish(msg_depth)
			self.pointcloud_pub.publish(msg_points)
		except CvBridgeError as e:
			print(e)

	def __init__(self):
		self.image_rgb_pub = rospy.Publisher("kinect2/rgb/erl",Image, queue_size = 500)
		self.image_depth_pub = rospy.Publisher("kinect2/depth/erl", Image, queue_size = 500)
		self.pointcloud_pub = rospy.Publisher("kinect2/points/erl", PointCloud2, queue_size = 500)
		self.image_rgb_sub = message_filters.Subscriber('kinect2/rgb/image_raw',Image)
		self.image_depth_sub = message_filters.Subscriber("kinect2/depth/image_raw", Image)
		self.pointcloud_sub = message_filters.Subscriber('kinect2/depth/points', PointCloud2)
		self.ts = message_filters.TimeSynchronizer([self.image_rgb_sub, self.image_depth_sub, self.pointcloud_sub], 3) # in order to deal with messages from 3 topics simultaneously
		self.ts.registerCallback(self.callback)
		self.bridge = CvBridge() 
	
		
							
def main(args):
	
	rospy.init_node("image_converter", anonymous=True)
	
	ic = showImg()
	
	try:
		rospy.spin()
		
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
