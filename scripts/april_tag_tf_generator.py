#!/usr/bin/env python
import numpy as np
import rospy
import tf
import rospkg
import time
from tf import TransformBroadcaster, TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Int16MultiArray

class publish_tag_tf(object):
	def __init__(self):
		rospy.init_node('republish_tag_tf')
		self.tf = TransformListener()
		self.br = TransformBroadcaster()
		
		rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tags_callback, queue_size=10)
		
		self.tag_detections = rospy.Publisher('/selective_grasping/tag_detections', Int16MultiArray, queue_size=10)
		self.detections_list = np.zeros(8)
	
	def tags_callback(self, msg):
		detections = len(msg.detections)
		self.detections_list = np.zeros(8)
		for idx in range(detections):
			detected_id = msg.detections[idx].id[0]
			self.detections_list[detected_id] = 1

	def publish(self, tag):
		try:
			self.tf.waitForTransform("camera_link", tag, rospy.Time(0), rospy.Duration(1.0)) # rospy.Time.now()
			ptFinal, oriFinal = self.tf.lookupTransform("camera_link", tag, rospy.Time(0))

			oriFinal_euler = euler_from_quaternion(oriFinal)
			new_ori = [0, -0.244, 0]
			new_ori[0] = oriFinal_euler[2]
			new_ori = quaternion_from_euler(new_ori[0], new_ori[1], new_ori[2])		

			self.br.sendTransform((ptFinal[2], 
								-ptFinal[0], 
								-ptFinal[1]),
								new_ori,
								rospy.Time.now(),
								tag + "_corrected",
								"camera_link")
		except:
			pass

	def detection_main(self):
		tags = ['tag_0', 'tag_1', 'tag_2', 'tag_3',
				'tag_4', 'tag_5', 'tag_6', 'tag_7']
		tags_to_send = Int16MultiArray()
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			print(self.detections_list)
			tags_to_send.data = self.detections_list
			self.tag_detections.publish(tags_to_send)
			for idx, detected_id in enumerate(self.detections_list):
				if detected_id:
					self.publish(tags[idx])
				else:
					print("Tag ID: '{}' not found".format(tags[idx]))

			rate.sleep()
			

def main():
	republish_tf = publish_tag_tf()
	republish_tf.detection_main()	

if __name__ == '__main__':
	main()