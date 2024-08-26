#!/usr/bin/env python3

from jetbot import Robot
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(data.data)

    # Motors are controlled like this:

    # robot = Robot()
    # robot.set_motors(0.5, 0)
    # robot.set_motors(0, 0.5)
    # robot.stop()

def listener():
    rospy.init_node('main')
    rospy.Subscriber("line_data", String, callback)
    rospy.spin()


if __name__ == "__main__":
    listener()

