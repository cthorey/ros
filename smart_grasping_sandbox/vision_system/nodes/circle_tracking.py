#!/usr/bin/env python
import rospy
import roslib
import cv2
roslib.load_manifest('vision_system')
from vision_system.detector import CircleDetector


def run_my_node():
    detector = CircleDetector()
    detector.ts.registerCallback(detector.callback)
    rospy.init_node('circle_tracking', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    run_my_node()
