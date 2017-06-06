#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('opencv')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher(
            "/vision/processed_image", Image, queue_size=5)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect_camera/depth/image_raw",
                                          Image, self.callback)

    def callback(self, data):
        try:
            cimg = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            try:
                cimg = cv2.medianBlur(cimg, 5)
                cimg = cv2.cvtColor(cimg, cv2.COLOR_GRAY2BGR)

                circles = cv2.HoughCircles(
                    cimg,
                    cv2.HOUGH_GRADIENT,
                    1,
                    20,
                    param1=50,
                    param2=30,
                    minRadius=0,
                    maxRadius=0)

                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    # draw the outer circle
                    cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                    # draw the center of the circle
                    cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
                cv2.imshow("Image window", cimg)
                cv2.waitKey(3)
            except Exception as e:
                print(e)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cimg, "bgr8"))
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = image_converter()
    rospy.init_node('circle_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
