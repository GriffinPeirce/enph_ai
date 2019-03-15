#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import glob
import os
import imutils

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String,Int32

from sensor_msgs.msg import (
    Image,
    CameraInfo,
)

class FindRed():

    def __init__(self):
        self.reference_image = []
        self.bridge = CvBridge()
        self._image_sub = rospy.Subscriber('/vector_camera/image', Image, self._image_callback, queue_size=10)
        self._image_pub = rospy.Publisher('red_image', Image, queue_size=10)
        self._heading_pub = rospy.Publisher('red_heading', Int32, queue_size=1)

    def _image_callback(self, image_msg):

        if image_msg is not None:
            vector_image = np.array(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough"))
            blurred = cv2.GaussianBlur(vector_image, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_RGB2HSV)

            mask1 = cv2.inRange(hsv, (0,50,20), (10,255,255))
            mask2 = cv2.inRange(hsv, (170,50,20), (190,255,255))
            mask = cv2.bitwise_or(mask1, mask2 )

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            centre = None

            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                centre = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                if radius > 10:
                    cv2.circle(vector_image, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                    # cv2.circle(vector_image, centre, 5, (0, 0, 255), -1)
                try:
                    self._image_pub.publish(self.bridge.cv2_to_imgmsg(vector_image, "rgb8"))
                except CvBridgeError as e:
                    print(e)
                # print(vector_image.shape)
                # print(vector_image.shape[1])
                self._heading_pub.publish(centre[0] - vector_image.shape[1]/2)
                print("Heading",centre[0] - vector_image.shape[1]/2)


            else:
                matchesMask = None


if __name__ == '__main__':
    findRed = FindRed()
    rospy.init_node('find_red')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # r = rospy.Rate(10)
 #        while not rospy.is_shutdown():
 #            self._publish_image()
 #            r.sleep()