#!/usr/bin/env python
import cv2
import rospy
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class line_follower():
    def __init__(self):
        self.bridge = CvBridge()
        self.vel_pub = rospy.Publisher("ai_vel", Twist, queue_size=1)
        self.image_pub = rospy.Publisher("/pi_camera/debug", Image, queue_size=1)
        self.image_sub = rospy.Subscriber("/pi_camera/image", Image, self.callback, queue_size = 1)
        self.lower_blue = np.array([97,0,00])
        self.upper_blue = np.array([150,255,255])
        self.lower_black = np.array([030,015,052])
        self.upper_black = np.array([071,103,117])
        self.deadband_factor = 0.2

    def callback(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
            # print("Got image")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        # mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
        # mask = cv2.inRange(hsv, np.array([0,0,0]),np.array([255,255,255]))
        h, w, d = cv_image.shape
        search_top = 2*h/4
        # Mask off the top region to avoid external lighting or distant track influencing the course direction
        mask[0:search_top, 0:w] = 0
        M = cv2.moments(mask)
        vel =  Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        if M['m00'] > 0:
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
          # BEGIN CONTROL
          err = cx - w/2
          print(err)
          if (abs(err) < w*self.deadband_factor):
            vel.linear.x = 0.271
          else:
            vel.angular.z = -float(err) / 800        
          # END CONTROL
        cv2.line(cv_image,(int(w*(0.5-self.deadband_factor)),0),(int(w*(0.5-self.deadband_factor)),h),(0,255,0),thickness=5)
        cv2.line(cv_image,(int(w*(0.5+self.deadband_factor)),0),(int(w*(0.5+self.deadband_factor)),h),(0,255,0),thickness=5)
        self.vel_pub.publish(vel)
        res = cv2.bitwise_and(hsv,hsv, mask= mask)
        cv2.imshow("window", cv_image)
        cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            # print("Published image")
        except CvBridgeError as e:
            print(e)

def main(args):
    lf = line_follower()
    rospy.init_node('line_follower')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)