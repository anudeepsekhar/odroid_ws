#! /usr/bin/env python

import roslib
import numpy as np
import rospy
import cv2
import sys, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

v1 = np.zeros((480, 640), np.uint8)
v2 = np.zeros((480, 640), np.uint8)

class ellipse_detection:

	def __init__(self):
        	cv2.namedWindow('mywindow')
        	cv2.namedWindow('mywindow1')
        	self.bridge = CvBridge()
        	self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)
		self.image_sub1 = rospy.Subscriber("usb_cam1/image_raw",Image,self.callback1)

	def callback(self,data):
        	try:
        		global v1
           		v1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        	except CvBridgeError as e:
           		print(e)

    	def callback1(self,data):
        	try:
	    		global v2 
	    		v2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
           		self.showImage()
        	except CvBridgeError as e:
            		print(e)


	def showImage(self):
	        imgray2 = cv2.cvtColor(v1, cv2.COLOR_BGR2GRAY)
	        imgray1 = cv2.cvtColor(v2, cv2.COLOR_BGR2GRAY)
	        cv2.imshow('mywindow1', imgray2)
	        cv2.imshow('mywindow', imgray1)
        	cv2.waitKey(1)   

def main(args):
	ed = ellipse_detection()
	#showImage()
	rospy.init_node('qwerty', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
        	print("Shutting down ellipse_detection")
        
        cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
