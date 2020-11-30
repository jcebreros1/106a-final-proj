#!/usr/bin/env python
"""
Path Planning Script for Lab 5
Author: Tiffany Cappellari
"""
import sys

from baxter_interface import Limb

import rospy
from gazebo_msgs.msg import ModelState
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner
from baxter_interface import gripper as robot_gripper

# Uncomment this line for part 5 of Lab 5
# from controller import Controller


def main():
    """
    Main Script
    """

    # Make sure that you've looked at and understand path_planner.py before starting


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
    

    def move_to_block(x, y, z, gX, gY, gZ, orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
        while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"

                #x, y, and z position
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z

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
                    closeGripper()
                    move_to_goalPosition(gX, gY, gZ)
                    
            except Exception as e:
                print e
                traceback.print_exc()
            else:
                break
    #move to goal
    def move_to_goalPosition(x, y, z):
        try:
            goal = PoseStamped()
            goal.header.frame_id = "base"

            #x, y, and z position
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = z

            #Orientation as a quaternion
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = -1.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 0.0

            plan = planner.plan_to_pose(goal, [])
            raw_input("Press <Enter> to move the right arm to goal pose: ")

        
            if not planner.execute_plan(plan):
                raise Exception("Execution failed")
            else:
                #open the Gripper and move from there
                openGripper()

                return
        except Exception as e:
            print e
            traceback.print_exc()

    def callback(message):
        print(message.name)
        print(message.pose)

    pos = raw_input("Enter a goal Position for the cubes: [x, y, z]")
    eval(pos)
    rospy.Subscriber("gazebo/model_states",ModelState, callback)
    while not rospy.is_shutdown():

    # Set your goal positions here
        zoffset = .92
        x= 0.4225
        y= -0.1265
        z= 0.772499999999
        hoverDist = 0.015
    	move_to_block(x, y, z-zoffset+hoverDist, pos[0], pos[1], pos[2])
        


if __name__ == '__main__':
    rospy.init_node('moveit_node')
    main()
