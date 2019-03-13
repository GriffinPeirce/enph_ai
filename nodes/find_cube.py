#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import glob
import os

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String

from sensor_msgs.msg import (
    Image,
    CameraInfo,
)

class FindCube():

    def __init__(self):
        self.reference_image = []
        self.bridge = CvBridge()
        self._image_sub = rospy.Subscriber('/vector_camera/image', Image, self._image_callback, queue_size=1)
        self._image_pub = rospy.Publisher('cube_image', Image, queue_size=1)
        for img in glob.glob(os.path.join(os.path.dirname(os.path.realpath(__file__)),"*_cube.jpg")):
            self.reference_image.append(cv2.imread(img))
        print(len(self.reference_image))

    def _image_callback(self, image):

        if self._image_pub.get_num_connections() > 0 and image is not None:

            # convert image to gray scale
            # img = camera_image.convert('RGB')
            # 640,360 image size?
            # img = camera_image
            # ros_img = Image()
            # ros_img.encoding = 'rgb8'
            # ros_img.width = img.size[0]
            # ros_img.height = img.size[1]
            # ros_img.step = ros_img.width
            # ros_img.data = img.tobytes()
            # ros_img.header.frame_id = 'vector_camera'
            ros_img = Image()
            ros_img = image
            # vector_time = camera_image.image_recv_time
            # ros_img.header.stamp = rospy.Time.from_sec(vector_time)
            ros_img.header.stamp = rospy.Time.now()
            # publish images and camera info
            if(len(self.reference_image)) > 0:
                try:
                    self._image_pub.publish(self.bridge.cv2_to_imgmsg(self.reference_image[0], "bgr8"))
                except CvBridgeError as e:
                    print(e)
            # camera_info = self._camera_info_manager.getCameraInfo()
            # camera_info.header = ros_img.header
            # self._camera_info_pub.publish(camera_info)

if __name__ == '__main__':
    findCube = FindCube()
    rospy.init_node('find_cube')
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # r = rospy.Rate(10)
 #        while not rospy.is_shutdown():
 #            self._publish_image()
 #            r.sleep()