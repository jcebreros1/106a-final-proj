#!/usr/bin/env python

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
map_Arms = {}
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
        for pose in messageModels.pose[3:]:
            if pose.position.y > 0:
                if "left_arm" in map_Arms.keys():
                    lsValues = map_Arms["left_arm"]
                    lsValues.append(pose.position)
                    map_Arms["left_arm"] = lsValues
                else:
                    map_Arms["left_arm"] = [pose.position]
            else:
                if "right_arm" in map_Arms.keys():
                    lsValues = map_Arms["right_arm"]
                    lsValues.append(pose.position)
                    map_Arms["right_arm"] = lsValues
                else:
                    map_Arms["right_arm"] = [pose.position]
        #blockPositions[0] = messageModels.pose[3]
        #blockPositions[1] = messageModels.pose[4]

    def image_plan():
        data = rospy.wait_for_message("/cameras/left_hand_camera/image",Image)

        try:
            height = data.height
            width = data.width
            #print("width", width)
            #print("height", height)
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite('/home/jesuscebreros/ros_workspaces/106a-final-proj/src/baxter_view.png', cv_image)
            #print("shape", cv_image.shape)
        except CvBridgeError as e:
            print(e)
        #rospy.init_node('image_converter', anonymous=True)
        path = '/home/jesuscebreros/ros_workspaces/106a-final-proj/src/baxter_view.png'
        return detectColor(path)
        
    def detectColor(path):

        image = cv2.imread(path)
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        greenTup = ((37,0,0), (73, 255, 255))
        isGreen = mask(greenTup, image)
        
        blueTup = ((93, 0, 0), (129, 255, 255))
        isBlue = mask(blueTup, image)

        purpleTup = ((138,0,0), (161, 255, 255))
        isPurple = mask(purpleTup, image)
        
        yellowTup = ((21, 0, 0), (35, 255, 255))
        isYellow = mask(yellowTup, image)

        if isGreen:
            return "green"
        elif isBlue:
            return "blue"
        elif isPurple:
            return "purple"
        elif isYellow:
            return "yellow"
        return "none"

    def mask(boundary, image):
        lower = boundary[0]
        upper = boundary[1]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        output = cv2.bitwise_and(image, image, mask = mask)
        # show the images
        cv2.imwrite("/home/jesuscebreros/ros_workspaces/106a-final-proj/src/output.png", output)
        return not np.all(output==0)


    #left_arm_planner = PathPlanner("left_arm")
    #right_arm_planner = PathPlanner("right_arm")

    rospy.init_node('moveit_node')
    models = rospy.wait_for_message("gazebo/model_states",ModelStates)
    callback(models)
    #print(map_Arms)
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
    #left_gripper.calibrate()
    #rospy.sleep(1.0)
    
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
    #left_arm_planner.add_box_obstacle(size, 'table', position)
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
    

    def move_arm(key,x, y, z,x_threshhold = 0.02,y_threshhold = 0.02,hoverDist = 0.02,zoffset = .92,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
        #while not rospy.is_shutdown():
            try:
                goal = PoseStamped()
                goal.header.frame_id = "base"

                #x, y, and z position
                goal.pose.position.x = x+x_threshhold
                goal.pose.position.y = y+y_threshhold
                goal.pose.position.z = z-zoffset+hoverDist

                #Orientation as a quaternion
                goal.pose.orientation.x = or_x
                goal.pose.orientation.y = or_y
                goal.pose.orientation.z = or_z
                goal.pose.orientation.w = or_w
                if key == 'left_arm':
                    left_arm_planner = PathPlanner("left_arm")
                    plan = left_arm_planner.plan_to_pose(goal, orien_const)
                    left_arm_planner.execute_plan(plan)
                else:
                    right_arm_planner = PathPlanner("right_arm")
                    plan = right_arm_planner.plan_to_pose(goal, orien_const)
                    right_arm_planner.execute_plan(plan)
                #openLeftGripper()
                # Might have to edit this for part 5
            except Exception as e:
                print e
                traceback.print_exc()


    def grasp(key,x, y, z,x_threshhold = 0.02,y_threshhold = 0.02,hoverDist = 0.020,zoffset = .92,  orien_const=[], or_x=0.0, or_y=-1.0, or_z=0.0, or_w=0.0):
        #while not rospy.is_shutdown():
            try:
                left_arm_planner = PathPlanner("left_arm")
                right_arm_planner = PathPlanner("right_arm")


                goal = PoseStamped()
                goal.header.frame_id = "base"

                #x, y, and z position
                goal.pose.position.x = x+x_threshhold
                goal.pose.position.y = y+y_threshhold
                goal.pose.position.z = z-zoffset+hoverDist

                #Orientation as a quaternion
                goal.pose.orientation.x = or_x
                goal.pose.orientation.y = or_y
                goal.pose.orientation.z = or_z
                goal.pose.orientation.w = or_w

                if key == 'left_arm':
                    plan = left_arm_planner.plan_to_pose(goal, orien_const)
                    openLeftGripper()
                    left_arm_planner.execute_plan(plan)
                    closeLeftGripper()
                else:
                    plan = right_arm_planner.plan_to_pose(goal, orien_const)
                    openRightGripper()
                    right_arm_planner.execute_plan(plan)
                    closeRightGripper()

            except Exception as e:
                print e
                traceback.print_exc()
    
    def getBlockPosition():
        return blockPositions[1].position.x,blockPositions[1].position.y,blockPositions[1].position.z

    while not rospy.is_shutdown():
        seq = raw_input("What sequence would you like []:" )
        seq = eval(seq)  #iterate throught the list and give it a proper position to be displayed as
        hardCodedPositions = [[.4225,0.0,.7725],[.55,0.0,.775],[.7,0,.775],[.85,0,.775]]
        for key in map_Arms.keys():
            allPositionList = map_Arms[key] #key is either left_arm or right_arm
            for position in allPositionList:
                x,y,z = position.x, position.y, position.z
                print(key)
                move_arm(key,x, y, z+.1)
                grasp(key,x, y, z)
                raw_input("press Enter to check cube color:")
                color = image_plan()
                print(color)
                print(seq)
                #['blue','green','yellow','purple']
                if color in seq:
                    index = seq.index(color)
                    #print(index)
                    toPos = hardCodedPositions[index] 
                    print(toPos)
                    #changing
                    move_arm(key,x, y, z+.1)
                    move_arm(key,toPos[0],toPos[1],toPos[2]+.05)
                    #print('moved')
                    if key == 'left_arm':
                        openLeftGripper()
                        move_arm(key,x, y, z+.1)
                    else:
                        openRightGripper()
                        move_arm(key,x, y, z+.1)
                    #we then move arm away from there

if __name__ == '__main__':
    main()
