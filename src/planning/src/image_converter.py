#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/cameras/head_camera/image",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cameras/head_camera/image",Image,self.callback)
    self.width = 0
    self.height = 0

  def callback(self,data):
    print("What is this width", data.width)
    print("What is this height", data.height)
    self.height = data.height
    self.width = data.width
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    print("This is shape or array", cv_image)
    cv2.imwrite('/home/hames10/ros_workspaces/106a-final-proj/src/baxter_view.png', cv_image)


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