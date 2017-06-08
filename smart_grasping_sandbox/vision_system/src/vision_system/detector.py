#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import image_geometry
from cv_bridge import CvBridge, CvBridgeError
import message_filters
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
        self.point_sub = rospy.Publisher(
            "/vision/bowl_center", Point, queue_size=5)

        self.bridge = CvBridge()
        self.image_sub = message_filters.Subscriber("/kinect_camera/image_raw",
                                                    Image)
        self.depth_sub = message_filters.Subscriber(
            "/kinect_camera/depth/image_raw", Image)

        self.ts = message_filters.TimeSynchronizer(
            [self.image_sub, self.depth_sub], 10)

        self.camera_model = image_geometry.PinholeCameraModel()
        self.cam_info = rospy.Subscriber(
            "/kinect_camera/camera_info",
            CameraInfo,
            self.init_camera,
            queue_size=6)

    def init_camera(self, data):
        self.camera_model.fromCameraInfo(data)

    def process_img(self, img_data):
        cimg = self.bridge.imgmsg_to_cv2(img_data, "bgr8")
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
        return cimg, cx, cy

    def get_depth(self, depth_data, cx, cy):
        cdepth = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
        return cdepth[cy, cx]

    def pixel2xyz(self, u, v, d):
        """
        See this for the detail
        http://answers.ros.org/question/120126/conversion-of-depht-image-coordinates-to-world-coordinates-uvd-to-xyz/
        """
        # pixel 2 normalized ray pointing toward the object
        p = self.camera_model.projectPixelTo3dRay([u, v])
        assert np.abs(np.sqrt(np.sum(np.array(p)**2)) -
                      1) < 1e-6, 'Not normalized'
        # Multiply by d to get the x,y,z relative to the camera frame
        p = map(lambda a: d * a, p)
        p = Point(*p)
        return p

    def callback(self, img_data, depth_data):
        try:
            cimg, cx, cy = self.process_img(img_data)
            u = cx[0]
            v = cy[0]
            d = self.get_depth(depth_data, u, v)
        except CvBridgeError as e:
            print(e)
            raise Exception
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cimg, "bgr8"))
            p = self.pixel2xyz(u, v, d)
            self.point_sub.publish(p)
            print('Successfully publish', p)
        except CvBridgeError as e:
            print(e)
            raise Exception
