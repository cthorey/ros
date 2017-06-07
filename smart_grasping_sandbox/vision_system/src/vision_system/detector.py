#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class CircleDetector(object):
    def __init__(self):
        self.image_pub = rospy.Publisher(
            "/vision/processed_image", Image, queue_size=5)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect_camera/image_raw", Image,
                                          self.callback)

    def callback(self, data):
        try:
            cimg = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cimg = cv2.medianBlur(cimg, 5)
            ori = cimg

            #threshold processed
            hsv = cv2.cvtColor(cimg, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1,
                                       int(cimg.shape[0] / 8.0), 20)
            mask2 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            # ensure at least some circles were found
            print('circle', circles)
            if circles is not None:
                # convert the (x, y) coordinates and radius of the circles to integers
                circles = np.uint16(np.around(circles.astype('float')))
                circles = np.round(circles[0, :].astype('float')).astype("int")
                # loop over the (x, y) coordinates and radius of the circles
                for (x, y, r) in circles:
                    # draw the circle in the output image, then draw a rectangle
                    # corresponding to the center of the circle
                    cv2.circle(cimg, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(cimg, (x - 5, y - 5), (x + 5, y + 5),
                                  (0, 128, 255), -1)
            print('Successfully process image')
        except CvBridgeError as e:
            print(e)
            raise Exception
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(mask2, "bgr8"))
        except CvBridgeError as e:
            print(e)
            raise Exception
