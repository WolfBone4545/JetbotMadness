#!/usr/bin/env python3

from jetbot import Robot
import rospy
import cv2
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

class Main:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node('main')
        rospy.Subscriber("line_data", String, self.line_callback, queue_size=10)
        rospy.Subscriber("camera", Image, self.camera_callback, queue_size=1)
        rospy.Subscriber("frame", Image, self.frame_callback, queue_size=1)
        self.muj_image = None
        

    def line_callback(self, data):
        points = eval(data.data)
        
        rospy.loginfo(data.data)
        tmp_img = self.muj_image.copy()

        if points is None:
            return

        for dev in points[0]:
            cv2.circle(tmp_img, (dev[0], dev[1]), 5, (0, 127, 127), -1)

        for dev in points[1]:
            cv2.circle(tmp_img, (dev[0], dev[1]), 5, (127, 127, 127), -1)

        cv2.imshow("points", tmp_img)
        cv2.waitKey(1)
        # rospy.loginfo(data.data)

        # Motors are controlled like this:

        # robot = Robot()
        # robot.set_motors(0.5, 0)
        # robot.set_motors(0, 0.5)
        # robot.stop()

    def camera_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        self.muj_image = image
        cv2.imshow("camera", self.muj_image)
        cv2.waitKey(1)

    def frame_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("frame", image)
        cv2.waitKey(1)

    def run(self):
        while not rospy.is_shutdown():
            pass



if __name__ == "__main__":
    main = Main()
    main.run()
