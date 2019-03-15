#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import glob
import os

from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String,Int32

from sensor_msgs.msg import (
    Image,
    CameraInfo,
)

MIN_MATCH_COUNT = 10

class FindCube():

    def __init__(self):
        self.reference_image = []
        self.bridge = CvBridge()
        self.sift = cv2.xfeatures2d.SIFT_create()
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 1)
        search_params = dict(checks = 10)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self._image_sub = rospy.Subscriber('/vector_camera/image_throttle', Image, self._image_callback, queue_size=10)
        self._image_pub = rospy.Publisher('cube_image', Image, queue_size=10)
        self._heading_pub = rospy.Publisher('cube_heading', Int32, queue_size=1)
        for img in glob.glob(os.path.join(os.path.dirname(os.path.realpath(__file__)),"*_cube.jpg")):
            self.reference_image.append(cv2.imread(img))
        self.kp1, self.des1 = self.sift.detectAndCompute(self.reference_image[0], None)
        print(len(self.reference_image))

    def _image_callback(self, image_msg):

        if image_msg is not None:
            vector_image = np.array(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough"))
            kp2, des2 = self.sift.detectAndCompute(vector_image, None)

            matches = self.flann.knnMatch(self.des1, des2, k=2)

            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)

            # print(good)
            if len(good)>MIN_MATCH_COUNT:
                src_pts = np.float32([ self.kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,1.0)
                matchesMask = mask.ravel().tolist()

                h,w,d = self.reference_image[0].shape
                pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
                dst = cv2.perspectiveTransform(pts,M)
                # print(dst)
                average = (np.average(np.int32(dst),0)).astype(np.int32)
                print(type(average[0][0]))

                image = cv2.polylines(vector_image,[np.int32(dst)],True,255,3, cv2.LINE_AA)
                try:
                    self._image_pub.publish(self.bridge.cv2_to_imgmsg(vector_image, "bgr8"))
                except CvBridgeError as e:
                    print(e)
                print(vector_image.shape)
                print(vector_image.shape[1])
                self._heading_pub.publish(vector_image.shape[1]/2 - int(average[0][0]))
                print("Heading",vector_image.shape[1]/2 - int(average[0][0]))


            else:
                matchesMask = None


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