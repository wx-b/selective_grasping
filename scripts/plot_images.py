#! /usr/bin/env python
from sensor_msgs.msg import Image
import cv2
import rospy
from cv_bridge import CvBridge
import matplotlib.pyplot as plt 
from std_msgs.msg import Int32MultiArray
import numpy as np

class plot_images(object):
	def __init__(self):
		rospy.init_node('ggcnn_ssd_detection')

		# Transform from ROS image to OpenCV image type
		self.bridge = CvBridge()

		camera_topic = rospy.get_param("/GGCNN/camera_topic")
		print('camera depth topic: ', camera_topic)

		self.center_calibrated_point = np.array([312, 240]) # x, y
		self.offset_ = 10 # Increase the bounding box size
		
		# Subscribe to the image published in Gazebo
		rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size=10)	
		rospy.Subscriber(camera_topic, Image, self.get_depth_callback, queue_size=10)
		rospy.Subscriber('bb_points_array', Int32MultiArray, self.bounding_boxes_callback, queue_size=10)        
		rospy.sleep(2.0)

		self.fig = plt.gcf()
		self.fig.show()
		self.fig.canvas.draw()
	
	def bounding_boxes_callback(self, msg):        
		center_calibrated_point = self.center_calibrated_point
		box_number = len(msg.data) / 4
		
		box_points = list(msg.data)
		
		i, index_inf, index_sup = 0, 0, 4
		points_vec = []
		offset = self.offset_
		K = 0.2
		
		# Adjust the RGB B.B.C. (Bounding Box Coordinates) to the Depth image B.B.C.
		while i < box_number:
			# Get each one of the 4 coordinates of each bounding box
			points_from_box = box_points[index_inf: index_sup]

			# Since we don't have the RGB image and depth aligned, we need to 
			# adjust the bounding box location from the RGB image to the depth image manually
			center = ((points_from_box[0] + points_from_box[2])/2, (points_from_box[1] + points_from_box[3])/2)
			dist = [int(center[0] - center_calibrated_point[0]), int(center[1] - center_calibrated_point[1])]
			final_distance = [int(dist[0]*K), int(dist[1]*K)]
			start_point = (points_from_box[0] + final_distance[0] - offset, points_from_box[1] + final_distance[1] - offset)
			end_point = (points_from_box[2] + final_distance[0] + offset, points_from_box[3] + final_distance[1] + offset)
			
			# The new adjusted bounding box coordinates 
			new_bb_coordinates = [start_point[0], start_point[1], end_point[0], end_point[1]]
			points_vec.append(new_bb_coordinates)
			index_inf += 4
			index_sup += 4
			i += 1
			
		self.points_vec = points_vec
		print(self.points_vec)
	
	def get_depth_callback(self, depth_image):
		self.depth_image = self.bridge.imgmsg_to_cv2(depth_image)
	
	def image_callback(self, color_msg):
		self.color_img = self.bridge.imgmsg_to_cv2(color_msg)
	
	def plot_images_overlapped(self):
		depth_image = self.depth_image
		points_vec = self.points_vec

		depth_image = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
		depth_height_res, depth_width_res, _ = depth_image.shape

		# color_img_gray = cv2.cvtColor(self.color_img, cv2.COLOR_BGR2GRAY)
		color_img_gray = self.color_img
		color_height_res, color_width_res, _ = color_img_gray.shape

		depth_image = depth_image.astype('uint8')
		color_img_gray = color_img_gray.astype('uint8')
		
		added_image = cv2.addWeighted(color_img_gray, 0.5, depth_image, 1.0, 0)

		for bbox in points_vec:
			added_image = cv2.rectangle(added_image, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 1)

		plt.imshow(added_image)
		self.fig.canvas.draw()

if __name__ == "__main__":
	plt_imgs = plot_images()

	while not rospy.is_shutdown():
		plt_imgs.plot_images_overlapped()
		




	
