#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import Twist, PoseStamped, Pose
from tf.transformations import euler_from_quaternion
class DrivePoint2Point():

    def __init__(self):
        self.pose_sub = rospy.Subscriber('/gazebo/adeept_awr_chassis', PoseStamped, self.pose_feedback_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber('/goal_pose',PoseStamped, self.pose_goal_callback, queue_size=1)
        self.vel_pub = rospy.Publisher('/ai_vel', Twist, queue_size=1, latch=False)
        self.pose = None
        self.pose_goal = None
        self.orientation_deadband = 0.1
        self.position_deadband = 0.1
        self.max_linear_vel = 0.271
        self.max_angular_vel = 1.8

    def pose_goal_callback(self, pose_msg):
        self.pose_goal = pose_msg.pose

    def pose_feedback_callback(self, pose_msg):
        self.pose = pose_msg.pose

        if self.pose is not None and self.pose_goal is not None:
            current_rpy = euler_from_quaternion((self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w))
            dest_rpy = euler_from_quaternion((self.pose_goal.orientation.x, self.pose_goal.orientation.y, self.pose_goal.orientation.z, self.pose_goal.orientation.w))
            dx = self.pose_goal.position.x - self.pose.position.x
            dy = self.pose_goal.position.y - self.pose.position.y
            heading = math.atan2(dy, dx)
            dist = math.hypot(dy,dx)
            cmd_vel = Twist()
            cmd_vel.angular.z = 0
            cmd_vel.linear.x = 0
            # Rotate to the direct goal heading
            if abs(current_rpy[2] - heading) > self.orientation_deadband and dist > self.position_deadband:
            	print("Aligning with heading")
                if current_rpy[2] - heading > 0:
                    cmd_vel.angular.z = -self.max_angular_vel
                else:
                    cmd_vel.angular.z = self.max_angular_vel
            # Drive forwards to goal position
            elif dist > self.position_deadband:
            	print("Driving to destination position",dist)
            	cmd_vel.linear.x = self.max_linear_vel
           	# Rotate to align with goal orientation
            elif dist < self.position_deadband and abs(current_rpy[2] - dest_rpy[2]) < self.orientation_deadband:
            	print("Aligning with desintation orientation")
            	if current_rpy[2] - dest_rpy[2] > 0:
                    cmd_vel.angular.z = -self.max_angular_vel
                else:
                    cmd_vel.angular.z = self.max_angular_vel
            else:
            	print("Reached destination")

            self.vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    p2p = DrivePoint2Point()
    rospy.init_node('drive_point_to_point')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass