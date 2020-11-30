#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
import rospy
import lab3_skeleton
from sensor_msgs.msg import JointState
import numpy as np
#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(positions):

    #Print the contents of the message to the console
    #These are all names or arguments shown from running rostopic echo /roboct/joint_states
    
    #positions = message.position

    left_s0_theta = positions[1] #4
    left_s1_theta = positions[2] #5
    left_e0_theta = positions[3] #2
    left_e1_theta = positions[4] #3
    left_w0_theta = positions[5] #6
    left_w1_theta = positions[6] #7
    left_w2_theta = positions[7] #8

    thetas = np.array([left_s0_theta, left_s1_theta, left_e0_theta,
                       left_e1_theta, left_w0_theta, left_w1_theta, left_w2_theta])
    g = lab3_skeleton.lab3(thetas)
    print("Matrix Transform: ")
    print(g)
    

#Define the method which contains the node's main functionality
def listener():

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("robot/joint_states", JointState, callback)


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('jointListener', anonymous=True)

    listener()