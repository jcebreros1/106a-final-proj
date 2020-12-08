#!/usr/bin/env python
"""
Path Planning Script for Lab 5
Author: Tiffany Cappellari
"""
import sys

from baxter_interface import Limb

import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import gripper as robot_gripper

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import argparse


blockPositions = [0]*2
height = 0
width = 0
cv_image = np.array([0,1,2])

def main():
	"""
	Main Script
	"""

	# Make sure that you've looked at and understand path_planner.py before starting
	def add_obstacle_to_left_arm(obstacle):
		position = PoseStamped()
		position.header.frame_id = "cube1"
		#x, y, and z position
		position.pose.position.x = obstacle.position.x
		position.pose.position.y = obstacle.position.y
		position.pose.position.z = obstacle.position.z -.92

		#Orientation as a quaternion
		position.pose.orientation.x = obstacle.orientation.x
		position.pose.orientation.y = obstacle.orientation.y
		position.pose.orientation.z = obstacle.orientation.z
		position.pose.orientation.w = obstacle.orientation.w

		size =  [0.025,.025,0.025]
		left_arm_planner.add_box_obstacle(size, 'cube', position)

	def callback(messageModels):
		blockPositions[0] = messageModels.pose[3]
		blockPositions[1] = messageModels.pose[4]

	def image_plan():
		data = rospy.wait_for_message("/cameras/left_hand_camera/image",Image)

		try:
			height = data.height
			width = data.width
			#print("width", width)
			#print("height", height)
			cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
			cv2.imwrite('/home/hames10/ros_workspaces/106a-final-proj/src/baxter_view.png', cv_image)
			#print("shape", cv_image.shape)
		except CvBridgeError as e:
			print(e)
		#rospy.init_node('image_converter', anonymous=True)
		path = '/home/hames10/ros_workspaces/106a-final-proj/src/baxter_view.png'
		return detectColor(path)
		
	def detectColor(path):

		image = cv2.imread(path)
		#hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		greenTup = ((37,0,0), (73, 255, 255))
		isGreen = mask(greenTup, image)
		
		blueTup = ((93, 0, 0), (129, 255, 255))
		isBlue = mask(blueTup, image)

		if (isGreen):
			return "green"
		elif isBlue:
			return "blue"

	def mask(boundary, image):
		lower = boundary[0]
		upper = boundary[1]
		hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, lower, upper)

		output = cv2.bitwise_and(image, image, mask = mask)
		# show the images
		cv2.imwrite("/home/hames10/ros_workspaces/106a-final-proj/src/output.png", output)
		return not np.all(output==0)


	left_arm_planner = PathPlanner("left_arm")
	#right_arm_planner = PathPlanner("right_arm")

	rospy.init_node('moveit_node')
	models = rospy.Subscriber("gazebo/model_states",ModelStates, callback)

	left_gripper = robot_gripper.Gripper('left')
	right_gripper = robot_gripper.Gripper('right')

	def closeRightGripper():
		right_gripper.close()
		rospy.sleep(1.0)
	def openRightGripper():
		right_gripper.open()
		rospy.sleep(1.0)
	def closeLeftGripper():
		left_gripper.close()
		rospy.sleep(1.0)
	def openLeftGripper():
		left_gripper.open()
		rospy.sleep(1.0)
	print('Calibrating...')
	#right_gripper.calibrate()
	left_gripper.calibrate()
<<<<<<< HEAD
	rospy.sleep(1.0)
	
=======
	#rospy.sleep(1.0)
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62
	#-----------------------------------------------------#
	## Add table as obstacle
	position = PoseStamped()
	position.header.frame_id = "base"
	#x, y, and z position
	position.pose.position.x = .75
	position.pose.position.y = 0
	position.pose.position.z = 0.772499999999 -.92

	#Orientation as a quaternion
	position.pose.orientation.x = 0
	position.pose.orientation.y = 0
	position.pose.orientation.z = 0
	position.pose.orientation.w = 1.0

	size =  [0.40,1.20,0.10]
	left_arm_planner.add_box_obstacle(size, 'table', position)
	#right_arm_planner.add_box_obstacle(size, 'table', position)
	
	#-----------------------------------------------------#

	# #Create a path constraint for the arm
	# #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
	#orien_const = OrientationConstraint()
	#orien_const.link_name = "left_gripper";
	#orien_const.header.frame_id = "base";
	#orien_const.orientation.y = -1.0;
	#orien_const.absolute_x_axis_tolerance = 0.1;
	#orien_const.absolute_y_axis_tolerance = 0.1;
	#orien_const.absolute_z_axis_tolerance = 0.1;
	#orien_const.weight = 1.0;
	
	def moveUp_left_arm(x, y, z,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
		try:
			goal = PoseStamped()
			goal.header.frame_id = "base"
			hoverDist = 0.015
			y_threshhold = -0.04
			zoffset = .92

				#x, y, and z position
			goal.pose.position.x = x
			goal.pose.position.y = y+y_threshhold
			goal.pose.position.z = z-zoffset+hoverDist

				#Orientation as a quaternion
			goal.pose.orientation.x = or_x
			goal.pose.orientation.y = or_y
			goal.pose.orientation.z = or_z
			goal.pose.orientation.w = or_w

			plan = left_arm_planner.plan_to_pose(goal, orien_const)

<<<<<<< HEAD
			if not left_arm_planner.execute_plan(plan):
				raise Exception("Execution failed")
			else:
				closeLeftGripper()
=======
				# Might have to edit this for part 5
			#if not left_arm_planner.execute_plan(plan):
				#raise Exception("Execution failed")
			left_arm_planner.execute_plan(plan)
			#openLeftGripper()
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62
		except Exception as e:
			print e
			traceback.print_exc()

<<<<<<< HEAD
	def move_on_top_of_cube(x, y, z,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
		while not rospy.is_shutdown():
			try:
				goal = PoseStamped()
				goal.header.frame_id = "base"
				hoverDist = 0.020
				y_threshhold = 0.02
=======

	def grasph_cube_with_left_arm(x, y, z,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
		#while not rospy.is_shutdown():
			try:
				goal = PoseStamped()
				goal.header.frame_id = "base"
				hoverDist = 0
				x_threshhold = 0.005
				y_threshhold = -0.03
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62
				zoffset = .92

				#x, y, and z position
				goal.pose.position.x = x
				goal.pose.position.y = y+y_threshhold
				goal.pose.position.z = z-zoffset+hoverDist

				#Orientation as a quaternion
				goal.pose.orientation.x = or_x
				goal.pose.orientation.y = or_y
				goal.pose.orientation.z = or_z
				goal.pose.orientation.w = or_w

				plan = left_arm_planner.plan_to_pose(goal, orien_const)

				raw_input("Press <Enter> to move the left arm to block pose: ")
				openLeftGripper()
				# Might have to edit this for part 5
<<<<<<< HEAD
				if not left_arm_planner.execute_plan(plan):
					raise Exception("Execution failed")
				else:
					closeLeftGripper()
					break
=======
				#if not left_arm_planner.execute_plan(plan):
				#	raise Exception("Execution failed")
				#else:
				left_arm_planner.execute_plan(plan)
				closeLeftGripper()
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62
			except Exception as e:
				print e
				traceback.print_exc()

	def moveUp_right_arm(x, y, z,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
		try:
			goal = PoseStamped()
			goal.header.frame_id = "base"
			hoverDist = 0.015
			y_threshhold = 0.04
			zoffset = .92

<<<<<<< HEAD
	def grasp_cube_with_left_arm(x, y, z,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
		while not rospy.is_shutdown():
			try:
				goal = PoseStamped()
				goal.header.frame_id = "base"
				hoverDist = 0.015
				y_threshhold = 0.03
=======
				#x, y, and z position
			goal.pose.position.x = x
			goal.pose.position.y = y+y_threshhold
			goal.pose.position.z = z-zoffset+hoverDist+.05

				#Orientation as a quaternion
			goal.pose.orientation.x = or_x
			goal.pose.orientation.y = or_y
			goal.pose.orientation.z = or_z
			goal.pose.orientation.w = or_w

			plan = right_arm_planner.plan_to_pose(goal, orien_const)

				# Might have to edit this for part 5
			#if not left_arm_planner.execute_plan(plan):
				#raise Exception("Execution failed")
			right_arm_planner.execute_plan(plan)
			#openRightGripper()
		except Exception as e:
			print e
			traceback.print_exc()


	def grasph_cube_with_right_arm(x, y, z,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
		#while not rospy.is_shutdown():
			try:
				goal = PoseStamped()
				goal.header.frame_id = "base"
				hoverDist = 0.01
				x_threshhold = 0.0
				y_threshhold = 0.05
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62
				zoffset = .92

				#x, y, and z position
				goal.pose.position.x = x
				goal.pose.position.y = y+y_threshhold
				goal.pose.position.z = z-zoffset+hoverDist

				#Orientation as a quaternion
				goal.pose.orientation.x = or_x
				goal.pose.orientation.y = or_y
				goal.pose.orientation.z = or_z
				goal.pose.orientation.w = or_w

				plan = right_arm_planner.plan_to_pose(goal, orien_const)

				#raw_input("Press <Enter> to move the left arm to block pose: ")
				openRightGripper()
				# Might have to edit this for part 5
				#if not left_arm_planner.execute_plan(plan):
				#	raise Exception("Execution failed")
				#else:
				right_arm_planner.execute_plan(plan)
				closeRightGripper()
			except Exception as e:
				print e
				traceback.print_exc()

	def drop_left_cube_in_neutral_pos(x=0.8225, y=0.1265, z=0.7725, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
			try:
				goal = PoseStamped()
				goal.header.frame_id = "base"
				hoverDist = 0
				x_threshhold = 0.0
				y_threshhold = 0.05
				zoffset = .92

				#x, y, and z position
				goal.pose.position.x = x
				goal.pose.position.y = y+y_threshhold
				goal.pose.position.z = z-zoffset+hoverDist

				#Orientation as a quaternion
				goal.pose.orientation.x = or_x
				goal.pose.orientation.y = or_y
				goal.pose.orientation.z = or_z
				goal.pose.orientation.w = or_w

				plan = right_arm_planner.plan_to_pose(goal, orien_const)

				#raw_input("Press <Enter> to move the left arm to block pose: ")
<<<<<<< HEAD
				openLeftGripper()
				
				if not left_arm_planner.execute_plan(plan):
					raise Exception("Execution failed")
				else:
					closeLeftGripper()

=======
				# Might have to edit this for part 5
				#if not left_arm_planner.execute_plan(plan):
				#	raise Exception("Execution failed")
				#else:
				right_arm_planner.execute_plan(plan)
				openRightGripper()
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62
			except Exception as e:
				print e
				traceback.print_exc()


	def getBlockPosition():
		return blockPositions[0].position.x,blockPositions[0].position.y,blockPositions[0].position.z

<<<<<<< HEAD
	
	while not rospy.is_shutdown():
	
		x,y,z = getBlockPosition()
		#print(x,y,z)
		#print(blockPositions[1])
		move_on_top_of_cube(x, y, z)
		raw_input("press Enter to check if cube is correct color")
		print(image_plan())
		#moveUp_left_arm(x,y,z)
		#moveUp_left_arm(x, y, z)
		#move_to_block(x, y, z + 0.05, True)
=======
	#while not rospy.is_shutdown():
	x,y,z = getBlockPosition()
	#print(x,y,z)
	#print(blockPositions[1])
	#grasph_cube_with_left_arm(x, y, z)
	#moveUp_left_arm(x,y,z)
	grasph_cube_with_right_arm(x, y, z)
	moveUp_right_arm(x,y,z)
	#move_to_block(x, y, z + 0.05, True)
>>>>>>> 2489e992afeb7410abbe0262405acc0000f91f62


if __name__ == '__main__':
	main()
