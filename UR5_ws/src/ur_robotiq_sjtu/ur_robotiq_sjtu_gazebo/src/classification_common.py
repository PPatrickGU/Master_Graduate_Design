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
#from sklearn_rvm import EMRVC  

class Class_obstacle:

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
		# I changed the rpy of camera_frame_optical to make it parallel with the ground.
		
		# get the intensity image 
		cv_grey = cv2.cvtColor(cv_rgb,cv2.COLOR_RGB2GRAY)
		
		# Irrelevant region detection
		
		# Remove far background & sky
		distance_thresh = 8
		self.remove_far(cv_grey, pc, distance_thresh)
		
		# Remove ground
		ground_length = 80
		ground_width = 100
		[height_ground, height_sigma] = self.calculate_gaussian(pc, ground_width, ground_length)
		self.remove_ground(pc, cv_grey, height_ground, height_sigma)
		
		# segmentation
		self.segmentation(cv_grey) 
		
		# erosion followed by dilation
		kernel = np.ones((5,5), dtype = int)
		opening = cv2.morphologyEx(cv_grey, cv2.MORPH_OPEN, kernel)
		
		cv2.imshow('test', cv_grey)
		cv2.waitKey(3)
		# obstacle clustering
		
		cx = 100
		cy = 1
		cz = 1
		dj = 100
		obs_coor = self.obstacle_clustering(cv_grey, pc, cx, cy, cz, dj) 
		# cx, cy, cz measures the weights of distance in 3 dimensions, dj is the distance standard 
		# obs_coor is a 3-D list, in the form of {{obs1: {coor1}, {coor2}...}, {obs2: {coor1},{coor2}...}, ...}
		# calculate vectors to perform RVM, the order is : width, height, ratio width/height, depth, distance, surface
		obs_vectors = self.RVM_vectors(pc, obs_coor, height_ground)
		# remove small obstacles in point cloud & depth image and change their color into that of the ground
		cv_depth.flags.writeable = True
		pc.flags.writeable = True # only avaible for numpy 1.15
		self.small_obs_remove(obs_coor, obs_vectors, pc, cv_rgb, cv_depth, height_ground, ground_rgb)
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
		self.image_rgb_pub = rospy.Publisher("camera/rgb/erl",Image, queue_size = 500)
		self.image_depth_pub = rospy.Publisher("camera/depth/erl", Image, queue_size = 500)
		self.pointcloud_pub = rospy.Publisher("camera/points/erl", PointCloud2, queue_size = 500)
		self.image_rgb_sub = message_filters.Subscriber('camera/rgb/image_raw',Image)
		self.image_depth_sub = message_filters.Subscriber("camera/depth/image_raw", Image)
		self.pointcloud_sub = message_filters.Subscriber('camera/depth/points', PointCloud2)
		self.ts = message_filters.TimeSynchronizer([self.image_rgb_sub, self.image_depth_sub, self.pointcloud_sub], 3) # in order to deal with messages from 3 topics simultaneously
		self.ts.registerCallback(self.callback)
		self.bridge = CvBridge() 
	
	def remove_far(self, cv_grey, pc, distance_thresh):
		[rows, cols] = [cv_grey.shape[0], cv_grey.shape[1]]
		for i in range(rows):
			for j in range(cols):
				if math.isnan(pc[i,j]['z']) == True or pc[i,j]['z'] > distance_thresh:
					cv_grey[i,j] = 0
	
	def calculate_gaussian(self, pc, ground_width, ground_length):
		[rows, cols] = pc.shape
		h_sum = 0
		point_counter = 0
		for i in range((rows - ground_length), rows):
			for j in range(((cols/2)-(ground_width/2)), ((cols/2)+(ground_width/2))):
				h_sum += pc[i,j]['y']
				point_counter += 1
		ground_height = h_sum/point_counter
		
		sigma_sum = 0
		for i in range((rows - ground_length), rows):
			for j in range(((cols/2)-(ground_width/2)), ((cols/2)+(ground_width/2))):				
				sigma_sum += np.square(pc[i,j]['y'] - ground_height)
		ground_sigma = math.sqrt(sigma_sum/point_counter)
		return [ground_height, ground_sigma]
		
	def remove_ground(self, pc, cv_grey, ground_height, ground_sigma):
		[rows, cols] = [cv_grey.shape[0], cv_grey.shape[1]]
		for i in range(rows):
			for j in range(cols):
				if abs(pc[i,j]['y'] - ground_height) < (0.02): # if the ground is not very plane, we can calculate by gaussian distribution
					cv_grey[i,j] = 0
					
	def segmentation(self, cv_grey):
		[rows, cols] = [cv_grey.shape[0], cv_grey.shape[1]]
		for i in range(rows):
			for j in range(cols):
				if cv_grey[i,j] > 0:
					cv_grey[i,j] = 255
	
	def obstacle_clustering(self, cv_grey, pc, cx, cy, cz, dj):
		[rows, cols] = cv_grey.shape
		result = []
		obs_num = 0
		for i in range(rows):
			for j in range(cols):
				if cv_grey[i,j] == 0:
					continue
				else:
					if obs_num == 0:
						l = list()
						result.append(l)
						l.append([i,j])
						obs_num += 1
					else:
						is_exist = 0 # verrify if the current point belongs to an existing obstacle
						for obstacles in result:
							d_mah = abs(cx*(pc[i,j]['x'] - pc[obstacles[0][0], obstacles[0][1]]['x'])) + abs(cy*(pc[i,j]['y'] - pc[obstacles[0][0], obstacles[0][1]]['y'])) + abs(cz*(pc[i,j]['z'] - pc[obstacles[0][0], obstacles[0][1]]['z']))
							if d_mah < dj:
								obstacles.append([i,j])
								is_exist = 1
								break
						if is_exist == 0:
							l = list()
							result.append(l)
							l.append([i,j])
							obs_num += 1
		print(obs_num)
		return result
	
	def RVM_vectors(self, pc, obs_coor, height_ground):
		obs_vectors = []
		obs_num = len(obs_coor)
		for i in range(obs_num):
			obs_vector = []
			
			# calculate width
			width = self.max_xcoor(obs_coor[i], pc) - self.min_xcoor(obs_coor[i], pc)
			obs_vector.append(width)
			
			# calculate height, remember -y is the right height
			height = height_ground - self.min_ycoor(obs_coor[i], pc)
			obs_vector.append(height)
			
			# Ratio between width and height
			ratio_wh = width/height
			obs_vector.append(ratio_wh)
			
			# calculate depth
			depth = self.max_zcoor(obs_coor[i], pc) - self.min_zcoor(obs_coor[i], pc)
			obs_vector.append(depth)
			
			# calculate distance between robot and object
			distance = self.calculate_distance(obs_coor[i], pc)
			obs_vector.append(distance)
			
			# calculate area (need to be retrained if the revolution is changed)
			area = len(obs_coor[i]) * distance
			obs_vector.append(area)
			obs_vectors.append(obs_vector)
		
		return obs_vectors		
			
	
	def max_xcoor(self, list_coor, pc):
		result = pc[list_coor[0][0], list_coor[0][1]]['x']
		for coors in list_coor:
			if pc[coors[0], coors[1]]['x'] > result:
				result = pc[coors[0], coors[1]]['x']
		return result
	
	def min_xcoor(self, list_coor, pc):
		result = pc[list_coor[0][0], list_coor[0][1]]['x']
		for coors in list_coor:
			if pc[coors[0], coors[1]]['x'] < result:
				result = pc[coors[0], coors[1]]['x']
		return result
		
	def min_ycoor(self, list_coor, pc):
		result = pc[list_coor[0][0], list_coor[0][1]]['y']
		for coors in list_coor:
			if pc[coors[0], coors[1]]['y'] < result:
				result = pc[coors[0], coors[1]]['y']
		return result
	
	def max_zcoor(self, list_coor, pc):
		result = pc[list_coor[0][0], list_coor[0][1]]['z']
		for coors in list_coor:
			if pc[coors[0], coors[1]]['z'] > result:
				result = pc[coors[0], coors[1]]['z']
		return result
		
	def min_zcoor(self, list_coor, pc):
		result = pc[list_coor[0][0], list_coor[0][1]]['z']
		for coors in list_coor:
			if pc[coors[0], coors[1]]['z'] < result:
				result = pc[coors[0], coors[1]]['z']
		return result
	
	def calculate_distance(self, list_coor, pc):
		sum_z = 0
		for coors in list_coor:
			sum_z += pc[coors[0], coors[1]]['z']
		return sum_z/len(list_coor)
		
	def small_obs_remove(self,obs_coor, obs_vectors, pc, cv_rgb, cv_depth, height_ground, ground_rgb):
		for i in range(len(obs_coor)):
			ignorable = False
			if obs_vectors[i][1] < 0: # case "ditch"
				if obs_vectors[i][0] < 0.15 or obs_vectors[i][3] < 100:
						ignorable = True
				
			elif obs_vectors[i][1] < 1: # case "rock" or "stone"
				ignorable = True
					
			if ignorable == True:
				for pixels in obs_coor[i]:
					cv_depth[pixels[0], pixels[1]] = 0
					pc[pixels[0], pixels[1]]['y'] = height_ground + 1 # set the height of the point to underground in order to ignore it in the map 
					cv_rgb[pixels[0], pixels[1]] = ground_rgb
		
							
def main(args):
	rospy.init_node("image_converter", anonymous=True)
	ic = Class_obstacle()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
