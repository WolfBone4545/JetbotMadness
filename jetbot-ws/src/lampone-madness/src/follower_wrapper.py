#!/usr/bin/env python3

import time
import cv2
import rospy
import cv_bridge
from sensor_msgs.msg import Image
import line_follower
import rospy
from std_msgs.msg import String

class FollowerWrapper:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.subscriber = rospy.Subscriber("camera", Image, callback=self.image_callback, queue_size=1)
        self.publisher = rospy.Publisher("line_data", String, queue_size=10)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        self.publisher.publish(str(line_follower.line_follower(image)))

        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("camera_sub")
    node = FollowerWrapper()
    rospy.spin()
