#!/usr/bin/python

import rospy
import actionlib
import numpy as np
import argparse
import copy
from copy import deepcopy
import rosservice
import sys
import re
import random

# ROS Msgs
from std_msgs.msg import Int16MultiArray, Float64MultiArray, Float32MultiArray, String, Bool, Int8
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController

# Gazebo
from gazebo_msgs.msg import ModelStates, ContactsState, ContactState, LinkState
from gazebo_msgs.srv import GetLinkState, DeleteModel

from tf import TransformListener, TransformBroadcaster
from tf.transformations import quaternion_from_euler

# Inverse kinematics
from trac_ik_python.trac_ik import IK

from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

def parse_args():
	parser = argparse.ArgumentParser(description='AAPF_Orientation')
	parser.add_argument('--gazebo', action='store_true', help='Set the parameters related to the simulated enviroonment in Gazebo')
	parser.add_argument('--ggcnn', action='store_true', help='run only the ggcnn')
	parser.add_argument('--place', action='store_true', help='run only if you want to place the objects based on the tags')
	args = parser.parse_args()
	return args

args = parse_args()

class ur5_grasp_project(object):
	def __init__(self, joint_values = None):
		rospy.init_node('command_GGCNN_ur5')
		self.joint_values_home = joint_values

		self.tf = TransformListener()

		# Used to change the controller
		self.controller_switch = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)

		# actionClient used to send joint positions
		self.client = actionlib.SimpleActionClient('pos_based_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		print("Waiting for server (pos_based_pos_traj_controller)...")
		self.client.wait_for_server()
		print("Connected to server (pos_based_pos_traj_controller)")

		self.picking = False # Tells the node that the object must follow the gripper
		
		##################
		# Gazebo Related #
		##################
		if args.gazebo:
			# For picking
			self.pub_model_position = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)
			self.get_model_coordinates = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
			self.delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
			rospy.Subscriber('gazebo/model_states', ModelStates, self.get_model_state_callback, queue_size=1)
			
			# Subscriber used to read joint values
			rospy.Subscriber('/joint_states', JointState, self.ur5_actual_position_callback, queue_size=1)
			rospy.sleep(1.0)
			
			# USED FOR COLLISION DETECTION
			self.finger_links = ['robotiq_85_right_finger_tip_link', 'robotiq_85_left_finger_tip_link']
			# LEFT GRIPPER
			self.string = ""
			rospy.Subscriber('/left_finger_bumper_vals', ContactsState, self.monitor_contacts_left_finger_callback) # ContactState
			self.left_collision = False
			self.contactState_left = ContactState()
			# RIGHT GRIPPER
			rospy.Subscriber('/right_finger_bumper_vals', ContactsState, self.monitor_contacts_right_finger_callback) # ContactState
			self.right_collision = False
			self.contactState_right = ContactState()

			self.client_gripper = actionlib.SimpleActionClient('gripper_controller_pos/follow_joint_trajectory', FollowJointTrajectoryAction)
			print("Waiting for server (gripper_controller_pos)...")
			self.client_gripper.wait_for_server()
			print("Connected to server (gripper_controller_pos)")
			
			
		#################
		# GGCNN Related #
		#################
		self.posCB = []
		self.ori = []
		self.grasp_cartesian_pose = []
		self.gripper_angle_grasp = 0.0
		self.final_orientation = 0.0
		if args.gazebo:
			self.GGCNN_offset_x = 0.0
			self.GGCNN_offset_y = 0.0
			self.GGCNN_offset_z = 0.020 # 0.019
		else:
			self.GGCNN_offset_x = -0.03 # 0.002
			self.GGCNN_offset_y = 0.02  # -0.05
			self.GGCNN_offset_z = 0.058 # 0.013

		self.ur5_joint_names = rospy.get_param("/ur5_joint_names")
		self.robotiq_joint_name = rospy.get_param("/robotiq_joint_name")

		# Topic published from GG-CNN Node
		rospy.Subscriber('ggcnn/out/command', Float32MultiArray, self.ggcnn_command_callback, queue_size=1)

		####################
		# Pipeline Related #
		####################
		self.classes = rospy.get_param("/classes")

		# Topics related to the new grasping pipeline
		rospy.Subscriber('flags/grasp_ready', Bool, self.grasp_ready_callback, queue_size=1) # Grasp flag
		rospy.Subscriber('flags/reposition_robot_flag', Bool, self.reposition_robot_callback, queue_size=1) # Reposition flag
		rospy.Subscriber('flags/detection_ready', Bool, self.detection_ready_callback ,queue_size=1) # Detection flag
		rospy.Subscriber('reposition_coord', Float32MultiArray, self.reposition_coord_callback, queue_size=1)
		rospy.Subscriber('/selective_grasping/tag_detections', Int16MultiArray, self.tags_callback, queue_size=1)

		self.tags = ['tag_0_corrected', 'tag_1_corrected', 'tag_2_corrected', 'tag_3_corrected', 
				     'tag_4_corrected', 'tag_5_corrected', 'tag_6_corrected', 'tag_7_corrected']
		
		if args.gazebo:
			self.tags_position_offset = [0.062, 0.0, 0.062]

		self.grasp_flag = False
		self.detected_tags = []

		self.require_class = rospy.Publisher('pipeline/required_class', Int8, queue_size=1)
				
		###################
		# Robotiq Related #
		###################
		self.pub_gripper_command = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=1)
		self.d = None # msg received from GGCN
		self.gripper_max_width = 0.14

	def turn_velocity_controller_on(self):
		self.controller_switch(['joint_group_vel_controller'], ['pos_based_pos_traj_controller'], 1)

	def turn_position_controller_on(self):
		self.controller_switch(['pos_based_pos_traj_controller'], ['joint_group_vel_controller'], 1)

	def turn_gripper_velocity_controller_on(self):
		self.controller_switch(['gripper_controller_vel'], ['gripper_controller_pos'], 1)

	def turn_gripper_position_controller_on(self):
		self.controller_switch(['gripper_controller_pos'], ['gripper_controller_vel'], 1)

	def tags_callback(self, msg):
		self.detected_tags = msg.data

	def grasp_ready_callback(self, msg):
		self.grasp_flag = msg.data
	
	def detection_ready_callback(self, msg):
		self.detection_ready_flag = msg.data
	
	def reposition_robot_callback(self, msg):
		self.reposition_robot_flag = msg.data
	
	def reposition_coord_callback(self, msg):
		self.reposition_coords = msg.data
	
	def monitor_contacts_left_finger_callback(self, msg):
		if msg.states:
			self.left_collision = True
			string = msg.states[0].collision1_name
			string_collision = re.findall(r'::(.+?)::',string)[0]
			# print("Left String_collision: ", string_collision)
			if string_collision in self.finger_links:
				string = msg.states[0].collision2_name
				print("Left Real string (object): ", string)
				self.string = re.findall(r'::(.+?)::', string)[0]
				# print("Left before: ", self.string)
			else:
				self.string = string_collision
				# print("Left in else: ", string_collision)
		else:
			self.left_collision = False

	def monitor_contacts_right_finger_callback(self, msg):
		if msg.states:
			self.right_collision = True
			string = msg.states[0].collision1_name
			string_collision = re.findall(r'::(.+?)::',string)[0]
			# print("Right String_collision: ", string_collision)
			if string_collision in self.finger_links:
				string = msg.states[0].collision2_name
				print("Right Real string (object): ", string)
				self.string = re.findall(r'::(.+?)::',string)[0]
				# print("Right before: ", self.string)
			else:
				self.string = string_collision
				# print("Right in else: ", self.string)
		else:
			self.right_collision = False

	def delete_model_service_method(self):
		"""
		Delete a model in Gazebo
		"""
		string = self.string
		model = string.replace("_link", "")
		self.delete_model_service(model)

	def ur5_actual_position_callback(self, joint_values_from_ur5):
		"""Get UR5 joint angles
		
		The joint states published by /joint_staes of the UR5 robot are in wrong order.
		/joint_states topic normally publishes the joint in the following order:
		[elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
		But the correct order of the joints that must be sent to the robot is:
		['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		
		Arguments:
			joint_values_from_ur5 {list} -- Actual angles of the UR5 Robot
		"""
		if args.gazebo:
			self.th3, self.robotic, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
			# print("Robotic angle: ", self.robotic)
		else:
			self.th3, self.th2, self.th1, self.th4, self.th5, self.th6 = joint_values_from_ur5.position
		
		self.actual_position = [self.th1, self.th2, self.th3, self.th4, self.th5, self.th6]

	def get_model_state_callback(self, msg):
		self.object_picking()

	def ggcnn_command_callback(self, msg):
		"""
		GGCNN Command Subscriber Callback
		"""
		self.tf.waitForTransform("base_link", "object_detected", rospy.Time(0), rospy.Duration(4.0)) # rospy.Time.now()
		object_pose, object_ori = self.tf.lookupTransform("base_link", "object_detected", rospy.Time(0))
		self.d = list(msg.data)
		object_pose[0] += self.GGCNN_offset_x
		object_pose[1] += self.GGCNN_offset_y
		object_pose[2] += self.GGCNN_offset_z
		
		self.posCB = object_pose

		self.ori = self.d[3]
		br = TransformBroadcaster()
		br.sendTransform((object_pose[0], 
							object_pose[1],
							object_pose[2]), 
							quaternion_from_euler(0.0, 0.0, self.ori),
							rospy.Time.now(),
							"object_link",
							"base_link")
		
	def get_link_position_picking(self):
		link_name = self.string
		print('String: ', link_name)
		model_coordinates = self.get_model_coordinates(self.string, 'wrist_3_link')
		self.model_pose_picking = model_coordinates.link_state.pose

	def reset_link_name(self):
		self.string = ""

	def object_picking(self):
		picking = self.picking
		if picking:
			# angle = quaternion_from_euler(1.57, 0.0, 0.0)
			object_picking = LinkState()
			object_picking.link_name = self.string
			object_picking.pose = Pose(self.model_pose_picking.position, self.model_pose_picking.orientation)
			object_picking.reference_frame = "wrist_3_link"
			self.pub_model_position.publish(object_picking)            

	def get_ik(self, position):
		"""Get the inverse kinematics 
		
		Get the inverse kinematics of the UR5 robot using track_IK package giving a desired intial position
		
		Arguments:
			position {list} -- A position representing x, y and z

		Returns:
			sol {list} -- Joint angles or None if track_ik is not able to find a valid solution
		"""

		camera_support_angle_offset = 0.0
		
		q = quaternion_from_euler(0.0, -3.14 + camera_support_angle_offset, 0.0)
		# Joint order:
		# ('shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'grasping_link')            
		ik_solver = IK("base_link", "grasping_link", solve_type="Distance")
		sol = ik_solver.get_ik([0.2201039360819781, -1.573845095552878, -1.521853400505349, -1.6151347051274518, 1.5704492904506875, 0.0], 
				position[0], position[1], position[2], q[0], q[1], q[2], q[3])
		
		if sol is not None:
			sol = list(sol)
			sol[-1] = 0.0
		else:
			print("IK didn't return any solution")
			
		return sol

	def __build_goal_message_ur5(self):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = JointTrajectory()
		goal.trajectory.joint_names = self.ur5_joint_names
		goal.goal_tolerance.append(JointTolerance('joint_tolerance', 0.1, 0.1, 0))
		goal.goal_time_tolerance = rospy.Duration(5,0)
		return goal

	def traj_planner(self, cart_pos, grasp_step='move', way_points_number=10, movement='slow'):
		"""Quintic Trajectory Planner
		
		Publish a trajectory to UR5 using quintic splines. 
		
		Arguments:
			cart_pos {[float]} -- Grasp position [x, y, z]
		
		Keyword Arguments:
			grasp_step {str} -- Set UR5 movement type (default: {'move'})
			way_points_number {number} -- Number of points considered in trajectory (default: {10})
			movement {str} -- Movement speed (default: {'slow'})
		"""
		
		offset_from_object = 0.05
		if grasp_step == 'pregrasp':
			self.grasp_cartesian_pose = deepcopy(self.posCB)
			self.grasp_cartesian_pose[-1] += offset_from_object
			joint_pos = self.get_ik(self.grasp_cartesian_pose)
			if joint_pos is not None:
				joint_pos[-1] = self.ori
				self.final_orientation = deepcopy(self.ori)
				self.gripper_angle_grasp = deepcopy(self.d[-2])
		elif grasp_step == 'grasp':
			self.grasp_cartesian_pose[-1] -= offset_from_object
			joint_pos = self.get_ik(self.grasp_cartesian_pose)
			if joint_pos is not None:
				joint_pos[-1] = self.final_orientation
		elif grasp_step == 'move':
			joint_pos = self.get_ik(cart_pos)
			if joint_pos is not None:
				joint_pos[-1] = 0.0
		
		if joint_pos is not None:
			if movement=='slow':
				final_traj_duration = 500.0 # total iteractions
			elif movement=='fast':
				final_traj_duration = 350.0

			v0 = a0 = vf = af = 0
			t0 = 5.0
			tf = (t0 + final_traj_duration) / way_points_number # tf by way point
			t = tf / 10 # for each movement
			ta = tf / 10 # to complete each movement
			a = [0.0]*6
			pos_points, vel_points, acc_points = [0.0]*6, [0.0]*6, [0.0]*6
			
			goal = self.__build_goal_message_ur5()

			for i in range(6):
				q0 = self.actual_position[i]
				qf = joint_pos[i]

				b = np.array([q0,v0,a0,qf,vf,af]).transpose()
				m = np.array([[1, t0, t0**2,   t0**3,    t0**4,    t0**5],
							[0,  1,  2*t0, 3*t0**2,  4*t0**3,  5*t0**4],
							[0,  0,     2,    6*t0, 12*t0**2, 20*t0**3],
							[1, tf, tf**2,   tf**3,    tf**4,    tf**5],
							[0,  1,  2*tf, 3*tf**2,  4*tf**3,  5*tf**4],
							[0,  0,     2,    6*tf, 12*tf**2, 20*tf**3]])
				a[i] = np.linalg.inv(m).dot(b)

			for i in range(way_points_number):
				for j in range(6):
					pos_points[j] =   a[j][0] +   a[j][1]*t +    a[j][2]*t**2 +    a[j][3]*t**3 +   a[j][4]*t**4 + a[j][5]*t**5
					vel_points[j] =   a[j][1] + 2*a[j][2]*t +  3*a[j][3]*t**2 +  4*a[j][4]*t**3 + 5*a[j][5]*t**4
					acc_points[j] = 2*a[j][2] + 6*a[j][3]*t + 12*a[j][4]*t**2 + 20*a[j][5]*t**3

				goal.trajectory.points.append(JointTrajectoryPoint(positions=pos_points,
																velocities=vel_points,
																accelerations=acc_points,
																time_from_start=rospy.Duration(t))) #default 0.1*i + 5
				t += ta

			self.client.send_goal(goal)
			self.all_close(joint_pos)
		else:
			print('Could not find a IK solution')
		
	
	def all_close(self, goal, tolerance=0.00005):
		"""Wait until goal is reached in configuration space
		
		This method check if the robot reached goal position since wait_for_result seems to be broken

		
		Arguments:
			goal {[list]} -- Goal in configuration space (joint values)
		
		Keyword Arguments:
			tolerance {number} -- Minimum error allowed to consider the trajectory completed (default: {0.00005})
		"""

		error = np.sum([(self.actual_position[i] - goal[i])**2 for i in range(6)])
		print("> Waiting for the trajectory to finish...")
		while not rospy.is_shutdown() and error > tolerance:
			error = np.sum([(self.actual_position[i] - goal[i])**2 for i in range(6)])
		if error < tolerance:
			print("> Trajectory Suceeded!\n") # whithin the tolerance specified
		else:
			rospy.logerr("> Trajectory aborted.")

	def genCommand(self, char, command, pos=None):
		"""
		Update the command according to the character entered by the user.
		"""
		if char == 'a':
			# command = outputMsg.Robotiq2FGripper_robot_output();
			command.rACT = 1 # Gripper activation
			command.rGTO = 1 # Go to position request
			command.rSP  = 255 # Speed
			command.rFR  = 150 # Force

		if char == 'r':
			command.rACT = 0

		if char == 'c':
			command.rACT = 1
			command.rGTO = 1
			command.rATR = 0
			command.rPR = 255
			command.rSP = 40
			command.rFR = 150
			
		# @param pos Gripper width in meters. [0, 0.087]
		if char == 'p':
			command.rACT = 1
			command.rGTO = 1
			command.rATR = 0
			command.rPR = int(np.clip((13.-230.)/self.gripper_max_width * self.ori + 230., 0, 255))
			command.rSP = 40
			command.rFR = 150

		if char == 'o':
			command.rACT = 1
			command.rGTO = 1
			command.rATR = 0
			command.rPR = 0
			command.rSP = 40
			command.rFR = 150

		return command

	def command_gripper(self, action):
		command = outputMsg.Robotiq2FGripper_robot_output();
		command = self.genCommand(action, command)
		self.pub_gripper_command.publish(command)  

	def gripper_send_position_goal(self, position=0.3, velocity=0.4, action='move'):
		"""Send position goal to the gripper
		
		Keyword Arguments:
			position {float} -- Gripper angle (default: {0.3})
			velocity {float} -- Gripper velocity profile (default: {0.4})
			action {str} -- Gripper movement (default: {'move'})
		"""

		self.turn_gripper_position_controller_on()
		duration = 0.2
		if action == 'pre_grasp_angle':
			max_distance = 0.085
			angular_coeff = 0.11
			K = 1.1 # lower numbers = higher angles
			angle = (max_distance - self.gripper_angle_grasp) / angular_coeff * K
			position = angle
			velocity = 0.4
		elif action == 'pick':
			position = 0.7
			velocity = 0.05
			duration = 8.0

		goal = FollowJointTrajectoryGoal()
		goal.trajectory = JointTrajectory()
		goal.trajectory.joint_names = self.robotiq_joint_name
		goal.trajectory.points.append(JointTrajectoryPoint(positions=[position],
														   velocities=[velocity],
														   accelerations=[0.0],
														   time_from_start=rospy.Duration(duration)))
		self.client_gripper.send_goal(goal)
		if action == 'pick':
			while not rospy.is_shutdown() and not self.left_collision and not self.right_collision:
				pass
			self.client_gripper.cancel_goal()
		
	def move_home_on_shutdown(self):
		self.client.cancel_goal()
		print("Shutting down node...")
	
	def grasp_main(self, point_init_home, depth_shot_point):
		random_classes = [i for i in range(len(self.classes))]

		bin_location = [[-0.65, -0.1, 0.2],
						[-0.65, 0.1, 0.2]]
		garbage_location = [-0.4, -0.30, 0.10]
		
		while not rospy.is_shutdown():
			print('\n')
			# The grasp is performed randomly and the chosen object is published to
			# the detection node
			if not len(random_classes):
				print('> There is no object remaining in the workspace. Resetting the objects list...')
				random_classes = [i for i in range(len(self.classes))]
			
			random_object_id = random.sample(random_classes, 1)[0]
			random_classes.pop(random_classes.index(random_object_id))
			print('> Object to pick: ', self.classes[random_object_id])
				
			self.require_class.publish(random_object_id)

			# Before pressing ENTER you should observe if the GG-CNN is getting the
			# right grasp on the selected object (green point in the grasp image)
			# just for safety
			raw_input("==== Press enter to start the grasping process!")

			if self.detection_ready_flag:
				if self.reposition_robot_flag:
					print('repos flag: ', self.reposition_robot_flag)
					print('> Repositioning robot...')
					self.tf.waitForTransform("base_link", "grasping_link", rospy.Time.now(), rospy.Duration(4.0))
					eef_pose, _ = self.tf.lookupTransform("base_link", "grasping_link", rospy.Time(0))
					
					corrected_position = [eef_pose[0] - self.reposition_coords[1],
										  eef_pose[1] + self.reposition_coords[0],
										  eef_pose[2]]
	
					self.traj_planner(corrected_position, movement='fast')			
					raw_input("==== Press enter when the trajectory finishes!")
							
				if self.grasp_flag and self.detection_ready_flag and not self.reposition_robot_flag:
					print('> Moving to the grasping position... \n')
					# print("Grasp flag: " + str(self.grasp_flag))
					# print("Reposition robot flag: " + str( self.reposition_robot_flag))
					self.traj_planner([], 'pregrasp', movement='fast')
					
					# It closes the gripper before approaching the object
					# It prevents the gripper to collide with other objects when grasping				
					if args.gazebo:
						self.gripper_send_position_goal(action='pre_grasp_angle')
					else:
						self.command_gripper('p')

					# Moving the robot to pick the object - BE CAREFULL!
					self.traj_planner([], 'grasp', movement='slow')
					
					# print("Picking object...")				
					# if args.gazebo:
					# 	self.gripper_send_position_goal(action='pick')
					# 	self.get_link_position_picking()
					# else:
					# 	raw_input("==== Press enter to close the gripper!")
					# 	self.command_gripper('c')
					
					print("Moving object to the bin...")				
					# After a collision is detected, the arm will start the picking action
					# self.picking = True # Attach object
					raw_input("==== go to bin location")

					if random_object_id < 5:
						self.traj_planner(bin_location[0], movement='fast')
					else:
						self.traj_planner(bin_location[1], movement='fast')

					rospy.sleep(2.0) # waiting for the tag detections
					number_of_detected_tags = sum(self.detected_tags)			

					if number_of_detected_tags:
						selected_tag_status = self.detected_tags[random_object_id] # if the tag was detected or not
						print('Selected tag status: ', selected_tag_status)
						print('selected tag: ', self.tags[random_object_id])
						if selected_tag_status:
							self.tf.waitForTransform("base_link", self.tags[random_object_id], rospy.Time(0), rospy.Duration(2.0)) # rospy.Time.now()
							ptFinal, oriFinal = self.tf.lookupTransform("base_link", self.tags[random_object_id], rospy.Time(0))
							ptFinal = [ptFinal[i] + self.tags_position_offset[i] for i in range(3)]

							raw_input("==== go to tag")	
							self.traj_planner(ptFinal, movement='fast')
						else:
							# In this case, the camera identifies some other tag but not the tag corresponding
							# to the object in case
							print('> Could not find the box tag. Placing the object anywhere \n')
							raw_input('=== Pres enter to proceed')
							self.traj_planner(garbage_location, movement='fast')
					else:
						# In this case, the robot cannot find any tag
						print('> Could not find any tag. Placing the object anywhere \n')
						raw_input('=== Pres enter to proceed')
						self.traj_planner(garbage_location, movement='fast')

					raw_input("==== Continue!")

					# print("Placing object...")					
					# # After the bin location is reached, the robot will place the object and move back
					# # to the initial position
					# self.picking = False # Detach object
					# if args.gazebo:
					# 	self.gripper_send_position_goal(0.3)
					# 	# Not working anymore - need to investigate
					# 	# self.delete_model_service_method() 
					# 	self.reset_link_name()
					# else:
					# 	self.command_gripper('o')

					print("> Moving back to home position...\n")
					# self.traj_planner([-0.45, -0.16, 0.15], movement='fast')
					self.traj_planner(point_init_home, movement='fast')
					self.traj_planner(depth_shot_point, movement='slow')
					print('> Grasping finished \n')
				else:
					print('> The requested object could not be grasped! \n')
					self.traj_planner(depth_shot_point, movement='fast')

			else:
				print('> The requested object was not detected! \n')
				
def main():
	ur5_vel = ur5_grasp_project()
	point_init_home = [-0.37, 0.11, 0.15]
	joint_values_home = ur5_vel.get_ik(point_init_home)
	ur5_vel.joint_values_home = joint_values_home

	# Send the robot to the custom HOME position
	raw_input("==== Press enter to 'home' the robot!")
	rospy.on_shutdown(ur5_vel.move_home_on_shutdown)
	ur5_vel.traj_planner(point_init_home, movement='fast')

	depth_shot_point = [-0.37, 0.11, 0.05]

	if not args.ggcnn:
		# Remove all objects from the scene and press enter
		raw_input("==== Press enter to move the robot to the 'depth cam shot' position!")
		ur5_vel.traj_planner(depth_shot_point, movement='fast')

	if args.gazebo:
		print("Starting the gripper in Gazebo! Please wait...")
		ur5_vel.gripper_send_position_goal(0.4)
	else:
		print("Starting the real gripper! Please wait...")
		ur5_vel.command_gripper('r')
		rospy.sleep(0.5)
		ur5_vel.command_gripper('a')
		ur5_vel.command_gripper('o')
	
	ur5_vel.grasp_main(point_init_home, depth_shot_point)

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print("Program interrupted before completion")
