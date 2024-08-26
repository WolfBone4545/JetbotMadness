#!/usr/bin/env python3

from jetbot import Robot
import rospy
import cv2
from std_msgs.msg import String, Image

def callback(data):
    rospy.loginfo(data.data)

    # Motors are controlled like this:

    # robot = Robot()
    # robot.set_motors(0.5, 0)
    # robot.set_motors(0, 0.5)
    # robot.stop()

def img_callback(image, name):
    cv2.imshow(name, image)
    cv2.waitKey(1)

def listener():
    rospy.init_node('main')
    rospy.Subscriber("line_data", String, callback)
    rospy.Subscriber("camera", Image, lambda x: img_callback(x, "camera"))
    rospy.Subscriber("camera", Image, lambda x: img_callback(x, "frame"))
    rospy.spin()


if __name__ == "__main__":
    listener()

