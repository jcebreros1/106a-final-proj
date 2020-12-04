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

# Uncomment this line for part 5 of Lab 5
# from controller import Controller

blockPositions = [0]*2

def main():
    """
    Main Script
    """
    
    # Make sure that you've looked at and understand path_planner.py before starting

    def callback(message):
        blockPositions[0] = message.pose[3]
        blockPositions[1] = message.pose[4]
        #print('----')
        #print(blockPositions[1].position.x)
        #print(message.name)
        #print(message.pose)
        #rospy.wait_for_message(topic, data type)

    rospy.init_node('moveit_node')
    rospy.Subscriber("gazebo/model_states",ModelStates, callback)
    planner = PathPlanner("right_arm")
    right_gripper = robot_gripper.Gripper('right')
    def closeGripper():
        right_gripper.close()
        rospy.sleep(1.0)
    def openGripper():
        right_gripper.open()
        rospy.sleep(1.0)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)
    openGripper()
    #-----------------------------------------------------#
    ## Add any obstacles to the planning scene here
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
    planner.add_box_obstacle(size, 'table', position)
    #-----------------------------------------------------#

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper";
    orien_const.header.frame_id = "base";
    orien_const.orientation.y = -1.0;
    orien_const.absolute_x_axis_tolerance = 0.1;
    orien_const.absolute_y_axis_tolerance = 0.1;
    orien_const.absolute_z_axis_tolerance = 0.1;
    orien_const.weight = 1.0;
    

    def move_to_block(x, y, z, openGrip,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"
                hoverDist = 0.015
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

                plan = planner.plan_to_pose(goal, orien_const)

                raw_input("Press <Enter> to move the right arm to block pose: ")

                # Might have to edit this for part 5
                if not planner.execute_plan(plan):
                    raise Exception("Execution failed")
                else:
                    if openGrip:
                        openGripper()
                    else:
                        closeGripper()      
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
    #pos = raw_input("Enter a goal Position for the cubes: [x, y, z]")
    #pos = eval(pos)
    #rospy.init_node('my_node_name', anonymous=True)
    #rospy.Subscriber("gazebo/model_states",ModelStates, callback)

    def getBlockPosition():
        return blockPositions[1].position.x,blockPositions[1].position.y,blockPositions[1].position.z

    while not rospy.is_shutdown():

    # Set your goal positions here
        #zoffset = .92
        #x= 0.4225
        #y= -0.1265
        #z= 0.772499999999
        #hoverDist = 0.015
        #y_threshhold = 0.05
        #/cameras/head_camera/image
        x,y,z = getBlockPosition()
    	move_to_block(x, y, z, False)
        move_to_block(x, y, z + 0.05, True)


if __name__ == '__main__':
    main()
