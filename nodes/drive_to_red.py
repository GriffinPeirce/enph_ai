#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import String,Int32
from geometry_msgs.msg import Twist

class DriveRed():

    def __init__(self):
        self.deadband = 10
        self._image_sub = rospy.Subscriber('/red_heading', Int32, self._heading_callback, queue_size=1)
        self._vel_pub = rospy.Publisher('/ai/cmd_vel', Twist, queue_size=1, latch=False)

    def _heading_callback(self, heading):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        angular_z = 0
        print("Heading",heading)
        if heading.data < (0 - self.deadband):
            angular_z = 0.5# vel_msg.angular.z = 1
        elif heading.data > (0 + self.deadband):
            angular_z = -0.5
        print(angular_z)
        vel_msg.angular.z = angular_z
        self._vel_pub.publish(vel_msg)


if __name__ == '__main__':
    driveRed = DriveRed()
    rospy.init_node('drive_red')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass