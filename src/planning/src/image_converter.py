#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class image_converter:

  def __init__(self):
<<<<<<< HEAD
    self.image_pub = rospy.Publisher("/cameras/right_hand_camera/image",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)
=======
>>>>>>> b4eb61329d7230ce856b16cfacad5833ed4a5c0f
    self.width = 0
    self.height = 0
    self.image_pub = rospy.Publisher("/cameras/left_hand_camera/image",Image)
    self.bridge = CvBridge()
    self.cv_image = np.array([0, 1, 2])
    self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image",Image,self.callback)


  def callback(self,data):
<<<<<<< HEAD
    #print("What is this width", data.width)
    #print("What is this height", data.height)
=======
>>>>>>> b4eb61329d7230ce856b16cfacad5833ed4a5c0f
    self.height = data.height
    self.width = data.width
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
<<<<<<< HEAD
    print("This is shape or array", cv_image)
    cv2.imwrite('/home/jesuscebreros/ros_workspaces/106a-final-proj/src/baxter_view.png', cv_image)
=======
    cv2.imwrite('/home/hames10/ros_workspaces/106a-final-proj/src/baxter_view.png', cv_image)
>>>>>>> b4eb61329d7230ce856b16cfacad5833ed4a5c0f

def main():
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main()