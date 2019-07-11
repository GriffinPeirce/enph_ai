#!/usr/bin/python
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import Image
    
class OverheadLocalization(object):

    def __init__(self):
        self._bridge = CvBridge()

        self._image_sub = rospy.Subscriber('camera_image', Image, self._image_callback, queue_size=10)
        self._image_pub_blue = rospy.Publisher('filtered/blue', Image, queue_size=1)
        

    def _image_callback(self, img_msg):
        if img_msg is not None:
            cv_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            blue_mask = cv2.inRange(hsv, self._lower_blue, self._upper_blue)
            #red_mask = cv2.inRange(hsv, self._lower_red, self._upper_red)
            #red_img = self._bridge.cv2_to_imgmsg(red_mask, encoding="passthrough")
            blue_img = self._bridge.cv2_to_imgmsg(blue_mask, encoding="passthrough")
            #self._image_pub_red.publish(red_img)
            self._image_pub_blue.publish(blue_img)

if __name__ == '__main__':
    rospy.init_node('overhead_camera_localization')
    loc = OverheadLocalization()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
