#!/usr/bin/env python3

import urllib.request, http
import numpy as np
import cv2
import utils
import rospy
import cv_bridge
from sensor_msgs.msg import Image


class FrameFetcher:

    def __init__(self):
        self.rate = rospy.Rate(1)
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher("/frame", Image, queue_size=10)

        # Server settings
        self.URL_server = "http://192.168.100.22/image/image.png"

        def publish_frame(image):
            if rospy.is_shutdown():
                quit()
            img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(img_msg)

        while not rospy.is_shutdown():
            img = None

            with urllib.request.urlopen(self.URL_server) as resp:
                try:
                    img = np.array(bytearray(resp.read()), dtype="uint8")
                    img = cv2.imdecode(img, cv2.IMREAD_COLOR)
                    img = cv2.resize(img, (utils.IMG_SHAPE[0]*utils.RESOLUTION_MODE, utils.IMG_SHAPE[1]*utils.RESOLUTION_MODE))

                    map1, map2 = cv2.fisheye.initUndistortRectifyMap(utils.K, utils.D, np.eye(3), utils.K,
                                                                    (img.shape[1], img.shape[0]), cv2.CV_16SC2)
                    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

                    publish_frame(img)
                except Exception as e:
                    rospy.logerr(f"Fuck U: " + str(e))

            self.rate.sleep()


### EXAMPLE USAGE ###
if __name__ == "__main__":
    rospy.init_node("frame_node")
    frame_fetcher = FrameFetcher()
