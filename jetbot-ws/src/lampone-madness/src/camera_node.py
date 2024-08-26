#!/usr/bin/env python3

import cv2
import numpy as np
import utils
import rospy
import cv_bridge
from sensor_msgs.msg import Image

class CameraNode:
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher("camera", Image, queue_size=10)

        def publish_camera(image):
            if rospy.is_shutdown():
                quit()
            img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(img_msg)

        utils.run_camera_with_callback(publish_camera)
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("camera_node")
    cam_publisher = CameraNode()
