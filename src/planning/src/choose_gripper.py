#!/usr/bin/env python
import webcolors
from image_converter import image_converter

class choose_gripper:
	colorOfBlocks = ["green", "blue"]
	def __init__(self, width=0, height=0):
		self.width = width
		self.height = height

	def iteratePixels(self, matrix):
		arm = ("left_arm", "right_arm")
		print(matrix)
		split = self.width/2
		hashMap = {}
		counter = 0
		for x in range(self.width/2, self.width):
			for y in range(100, self.height-100):
				narray = matrix[x][y]
				red = narray[0]
				green = narray[1]
				blue = narray[2]
				rgb = (red, green, blue)
				if webcolors.rgb_to_name(rgb) in colorOfBlocks and colorOfBlocks not in hashMap:
					if x < split:
						tup = (counter, arm[0])
						hashMap[colorOfBlocks] = tup
						counter += 1
					else:
						tup = (counter, arm[1])
						hashMap[colorOfBlocks] = tup
						counter += 1

		return hashMap


def main():
	cv_img = image_converter()
	rospy.init_node('image_converter', anonymous=True)
	gripper = choose_gripper(cv_img.width, cv_img.height)
	gripper.iteratePixels(cv_img)
	try:
		rospy.spin()
	except KeyboardInterrupt:
	    print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
    main()