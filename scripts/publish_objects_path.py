#!/usr/bin/python
import rospy

# ROS
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose

# Gazebo
from gazebo_msgs.msg import ModelStates, LinkState
from gazebo_msgs.srv import GetLinkState

class objects_path_track(object):
	def __init__(self, joint_values = None):
		rospy.init_node('publish_objects_path')
		
		##################
		# Gazebo Related #
		##################
		# Publisher
		self.pub_model_position = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)

		# Service
		self.get_model_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

		# Subscriers
		rospy.Subscriber('grasp_started', Bool, self.grasping_flag_callback, queue_size=1)
		rospy.Subscriber('grasp_object_name', String, self.grasping_object_name_callback, queue_size=1)
		rospy.Subscriber('grasp_object_position', Pose, self.object_pose_callback, queue_size=1)

		# Initialize some variables
		self.grasping_flag = False
		self.object_name = ''
		self.object_picking_obj = LinkState()
			
	def grasping_flag_callback(self, msg):
		self.grasping_flag = msg.data
	
	def grasping_object_name_callback(self, msg):
		self.object_name = msg.data
	
	def object_pose_callback(self, msg):
		self.object_pose = msg
		# print(self.object_pose)

	def object_picking(self):
		picking = self.grasping_flag
		# print(picking)
		if picking:
			# print('object_name: ', self.object_name)			
			self.object_picking_obj.link_name = self.object_name
			self.object_picking_obj.pose = self.object_pose
			self.object_picking_obj.reference_frame = "wrist_3_link"
			self.pub_model_position.publish(self.object_picking_obj)
		else:
			self.object_name = ''

	def spin(self):
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			self.object_picking()
			# rate.sleep()
						
def main():
	path_track = objects_path_track()
	path_track.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion")
