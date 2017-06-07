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
from skimage import data, color
from skimage.transform import hough_circle, hough_circle_peaks
from skimage.feature import canny
from skimage.draw import circle_perimeter
from skimage.util import img_as_ubyte


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

            #threshold img
            hsv = cv2.cvtColor(cimg, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # detect edges
            edges = canny(mask, sigma=3, low_threshold=10, high_threshold=50)
            # Detect two radii
            hough_radii = np.arange(20, 35, 2)
            hough_res = hough_circle(edges, hough_radii)

            # Select the most prominent 5 circles
            accums, cx, cy, radii = hough_circle_peaks(
                hough_res, hough_radii, total_num_peaks=1)
            for center_y, center_x, radius in zip(cy, cx, radii):
                circy, circx = circle_perimeter(center_y, center_x, radius)
                cimg[circy, circx] = (220, 20, 20)

            print(cx, cy)
            print('Successfully process image')

        except CvBridgeError as e:
            print(e)
            raise Exception
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cimg, "bgr8"))
        except CvBridgeError as e:
            print(e)
            raise Exception
