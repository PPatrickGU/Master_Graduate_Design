#!/usr/bin/env python

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
from collections import Counter
import message_filters
import math
import tf

class DynamicDetection:
	
	
	def callback(self, data_depth, data_points, data_rgb, data_seg):
		try:
			# transform ROS rgb image to Opencv rgb image
			cv_rgb = self.bridge.imgmsg_to_cv2(data_rgb, "bgr8")
			cv_segmentation = self.bridge.imgmsg_to_cv2(data_seg, "bgr8")
			grey_segmentation = cv2.cvtColor(cv_segmentation,cv2.COLOR_RGB2GRAY)
			cv_depth = self.bridge.imgmsg_to_cv2(data_depth, desired_encoding='passthrough')
		except CvBridgeError as e:
			print(e)
		cv_grey = cv2.cvtColor(cv_rgb,cv2.COLOR_RGB2GRAY)
		
		if self.ifstart == 0:
			self.pc_frame1 = self.pc_frame2
			self.grey_frame1 = self.grey_frame2
			self.pos1, self.ori1 = self.pos2, self.ori2
		else:
			self.grey_frame1 = cv_grey
			self.pc_frame1 = ros_numpy.numpify(data_points)
			self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(0), rospy.Duration(3.0))
			self.pos1, self.ori1 = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
			self.ifstart = 0
		
		# update the transform relation from odom to base_link
		self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(0), rospy.Duration(3.0))
		self.pos2, self.ori2 = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
		self.pc_frame2 = ros_numpy.numpify(data_points)
		self.grey_frame2 = cv_grey
		
        # pc is an array composed of 480*640 data in according to each pixel, each data is composed of 'x', 'y', 'z' and 'rgb' info. Get them use for example pc[100,100]['z']
		# according to camera_frame_optical frame, x(width) increase from left to right, y (-height) increase from high to low, z (distance) increase from near to far 
		
		### calculate the rotation and transformation difference between two frames
		pos_diff = np.array(self.pos2) - np.array(self.pos1)
		ori_diff = np.dot(np.transpose(self.calcRotationMat(self.ori1)), self.calcRotationMat(self.ori2))
		
		### Image Processing
		# Detection and description of keypoints
		pts1, desc1 = self.kp1.detectAndCompute(self.grey_frame1,None)
		pts2, desc2 = self.kp2.detectAndCompute(self.grey_frame2,None)
		
		# transform the points into numpy.array form
		pts1= cv2.KeyPoint_convert(pts1)
		pts2= cv2.KeyPoint_convert(pts2)
		
		# Pairing of keypoints
		bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
		
		# there are 4 objects in matches, see https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html
		matches = bf.match(desc1,desc2)
		matches = sorted(matches, key = lambda x:x.distance)
		
		# Pick out the best-matched points' coordinates 
		Nbest = len(matches)
		pts_sorted1 = np.zeros((Nbest,2), dtype=int)
		pts_sorted2 = np.zeros((Nbest,2), dtype=int)
		for i in range(Nbest):
			pts_sorted1[i][0] = pts1[matches[i].queryIdx][1]
			pts_sorted1[i][1] = pts1[matches[i].queryIdx][0]
			pts_sorted2[i][0] = pts2[matches[i].trainIdx][1]
			pts_sorted2[i][1] = pts2[matches[i].trainIdx][0]

		# Calculate the differences of distance between the caracteristic points of two frames
		ccc = np.array(self.pc_frame2.tolist())
		diff_vec = np.ones((Nbest, 1, 3))
		for i in range(Nbest):
			diff_vec[i] = np.matmul(ori_diff, ccc[pts_sorted2[i][0]][pts_sorted2[i][1]][0:3]) + pos_diff - ccc[pts_sorted1[i][0]][pts_sorted1[i][1]][0:3]
		diff_vec = diff_vec.reshape(Nbest,3)
		diff = np.linalg.norm(diff_vec, axis=1)

		pt_list = []
		for i in range(Nbest):
			if(~np.isnan(diff[i]) and diff[i] > 0.0001):
				pt_list.append(pts_sorted2[i].tolist())
		grey_list = []
		for i in range(len(pt_list)):
			grey_list.append(grey_segmentation[pt_list[i][0], pt_list[i][1]])

		cv_depth.flags.writeable = True
		self.pc_frame2.flags.writeable = True # only avaible for numpy 1.15
		if (len(grey_list) != 0):
			index = np.where(grey_segmentation == Counter(grey_list).most_common(1)[0][0])
			for i in range(len(grey_list)):
				cv_depth[index[0][i], index[1][i]] = 'nan'
				ccc[index[0][i], index[1][i]] = ['nan', 'nan', 'nan', 'nan']
			#print(type(ccc[index[0][0], index[1][0]]))

		# img3 = cv2.drawMatches(self.grey_frame1,pts1,self.grey_frame2,pts2,matches[:Nbest],None,flags=2)
		# print(pts_sorted2[1])
		# print(self.pc_frame2.shape)
		# Compare the position differences of keypoints between two frames
		#

		# calculate the difference of tf between frames
		
		msg_points = ros_numpy.msgify(PointCloud2, ccc)
		msg_points.header = data_points.header
		msg_depth = self.bridge.cv2_to_imgmsg(cv_depth, encoding = "passthrough")
		msg_depth.header = data_depth.header
		#msg_rgb = self.bridge.cv2_to_imgmsg(cv_rgb, "bgr8")
		#msg_rgb.header = data_rgb.header
		
		try:
			#self.image_rgb_pub.publish(msg_rgb)
			self.image_depth_pub.publish(msg_depth)
			self.pointcloud_pub.publish(msg_points)
		except CvBridgeError as e:
			print(e)

	def __init__(self):
		
		self.ifstart = 1
		# initialize ORB detecter
		self.kp1 = cv2.ORB_create(nfeatures = 300, scaleFactor = 1.2, nlevels = 8)
		self.kp2 = cv2.ORB_create(nfeatures=300, scaleFactor = 1.2, nlevels = 8)
		
		#self.image_rgb_pub = rospy.Publisher("camera/rgb/dynamic",Image, queue_size = 500)
		self.image_depth_pub = rospy.Publisher("ROB314/depth/dynamic", Image, queue_size = 500)
		self.pointcloud_pub = rospy.Publisher("ROB314/points/dynamic", PointCloud2, queue_size = 500)
		self.tf_listener = tf.TransformListener()
		
		
		#TODO
		#self.image_division_sub = message_filters.Subscriber('',Image)
		self.image_segmentation_sub = message_filters.Subscriber("camera/color/image_raw", Image)
		self.image_depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)
		self.pointcloud_sub = message_filters.Subscriber('camera/depth/points', PointCloud2)
		self.image_rgb_sub = message_filters.Subscriber('camera/rgb/image_raw',Image)
		self.ts = message_filters.TimeSynchronizer([self.image_depth_sub, self.pointcloud_sub, self.image_rgb_sub, self.image_segmentation_sub], 4) # in order to deal with messages from 2 topics simultaneously
		self.ts.registerCallback(self.callback)
		self.bridge = CvBridge() 
		
		# initialize two-frame data 
		self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(0), rospy.Duration(3.0));
		self.pos2, self.ori2 = self.tf_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
	
	# calculate 3*3 Rotation Matrix from quaternion
	def calcRotationMat(self, q):
		Mat = np.zeros((3,3))
		Mat[0,0] = 1 - 2*np.square(q[2]) - 2*np.square(q[3])
		Mat[0,1] = 2*q[1]*q[2] - 2*q[0]*q[3]
		Mat[0,2] = 2*q[1]*q[3] + 2*q[0]*q[2]
		Mat[1,0] = 2*q[1]*q[2] + 2*q[0]*q[3]
		Mat[1,1] = 1 - 2*np.square(q[1])- 2*np.square(q[3])
		Mat[1,2] = 2*q[2]*q[3] - 2*q[0]*q[1]
		Mat[2,0] = 2*q[1]*q[3] - 2*q[0]*q[2]
		Mat[2,1] = 2*q[2]*q[3] + 2*q[3]*q[1]
		Mat[2,2] = 1 - 2*np.square(q[1]) - 2*np.square(q[2])
		return Mat
		
							
def main(args):
	rospy.init_node("Dynamic_Detection", anonymous=True)
	ic = DynamicDetection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
