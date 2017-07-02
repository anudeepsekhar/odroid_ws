#! /usr/bin/env python

import roslib
import rospy
import cv2
import sys, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ellipse_detection:

    def __init__(self):
        cv2.namedWindow('mywindow')

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('mywindow', imgray)
        cv2.waitKey(1)

    def getTrackValue(self, value):
        return value


def main(args):
    ed = ellipse_detection()
    rospy.init_node('ellipse_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ellipse_detection")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
