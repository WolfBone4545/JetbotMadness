#!/usr/bin/env python3

from jetbot import Robot
import rospy
import cv2
import cv_bridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import time


class Main:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        rospy.init_node('main')
        rospy.Subscriber("line_data", String, self.line_callback, queue_size=10)
        rospy.Subscriber("camera", Image, self.camera_callback, queue_size=1)
        rospy.Subscriber("frame", Image, self.frame_callback, queue_size=1)
        self.muj_image = None
        self.zaklad_rychlost = [-0.6, 0.6]
        
        self.minRange = 308
        self.maxRange = 348
        self.minSpeed = 0.01
        
        self.lastPos = 0
        self.lastYPos = 0
        self.lastWPos = 0
        self.lastDirection = ""
        
        self.lastOdchyl = 0
        self.lineDist = 0
        self.lastLineDist = 0
        self.distFromCenter = 0
        self.avgX0 = 0
        self.avgX1 = 0

    
    def stop(self, robot):
        robot.set_motors(0, 0)
        robot.stop()
    
    def straight(self, robot):
        while not rospy.is_shutdown():
            robot.set_motors(self.zaklad_rychlost[0], self.zaklad_rychlost[1])
    
    def screenshot(self, name, format, img):
        time.sleep(5)
        
        for i in range(10):
            filename = name.join(f"_{i}").join(format)
            cv2.imwrite(filename, img)
    
    def move(self, robot, pos, speed, odchyl, lineDist, debug = False):
        if debug:
            robot.stop()
        else:
            
            if pos > self.maxRange and lineDist > 100:
                robot.set_motors(speed[0] / 5, speed[1])
            
            elif pos < self.minRange and lineDist > 100:
                robot.set_motors(speed[0], speed[1] / 5)

            else:
                if odchyl >= 0:
                    robot.set_motors(speed[0], speed[1] + odchyl)

                elif odchyl < 0:
                    robot.set_motors(speed[0] - odchyl, speed[1])

    def average_points(self, points, image, color, default_value):
        sumX = 0
        avgX = 0

        for dev in points:
            cv2.circle(image, (dev[0], dev[1]), 5, color, -1)  
            sumX += dev[0]
            
        try:
            avgX = int(sumX / len(points))    
        except ZeroDivisionError:
            avgX = default_value
        
        return avgX

    def line_callback(self, data):
        robot = Robot()
        avgX = 0
        points = eval(data.data)
        # rospy.loginfo(data.data)
        tmp_img = self.muj_image.copy()
        shape = tmp_img.shape

        if isinstance(points[0], int): # yel_point_dev
            self.lastWPos = shape[1]
            self.lastPos = int((self.lastYPos + self.lastWPos) / 2)
            self.move(robot, self.lastPos, self.zaklad_rychlost, self.lastOdchyl, self.lastLineDist)
            return
        
        if points[0] is None and points[1] is not None:
            avgX = 0

        if points[1] is None and points[0] is not None:
            avgX = 656    
        
        if points is None:
            self.move(robot, 328, [0.6, -0.6], 0, self.lineDist)

        
            
    

        

        # if points[0] is None and points[1] is not None:
        #     self.lastDirection = "left"
        #     self.lastYPos = 0
        #     self.lastPos = int((self.lastYPos + self.lastWPos) / 2)
        #     self.move(robot, self.lastPos, self.zaklad_rychlost, self.lastOdchyl, self.lastLineDist)
        
        # elif points[0] is not None and points[1] is None:
        #     self.lastDirection = "right"
        #     self.lastWPos = shape[1]
        #     self.lastPos = int((self.lastYPos + self.lastWPos) / 2)
        #     self.move(robot, self.lastPos, self.zaklad_rychlost, self.lastOdchyl, self.lastLineDist)

        # if points is None and self.lastDirection == "right":
        #     self.lastWPos = shape[1]
        #     self.lastYPos = int(shape[0] / 2)
        #     self.lastPos = int((self.lastYPos + self.lastWPos) / 2)
        #     self.move(robot, self.lastPos, self.zaklad_rychlost, self.lastOdchyl, self.lastLineDist)
        #     return
        
        # elif points is None and self.lastDirection == "left":
        #     self.lastYPos = 0
        #     self.lastWPos = int(shape[0] / 2)
        #     self.lastPos = int((self.lastYPos + self.lastWPos) / 2)
        #     self.move(robot, self.lastPos, self.zaklad_rychlost, self.lastOdchyl, self.lastLineDist)
        #     return

        self.avgX0 = self.average_points(points[0], tmp_img, (0, 127, 127), 0)
        self.avgX1 = self.average_points(points[1], tmp_img, (127, 127, 127), shape[1])

        if points is not None:
            avgX = int((self.avgX0 + self.avgX1) / 2)
        
        self.lineDist = np.abs(self.avgX0 - self.avgX1)

        cv2.circle(tmp_img, (self.minRange, 400), 5, (0, 255, 0), -1)
        cv2.circle(tmp_img, (self.maxRange, 400), 5, (0, 255, 0), -1)     
        cv2.circle(tmp_img, (avgX, 400), 5, (0, 0, 255), -1)
        
        odchyl = (avgX - 328) / 100

        cv2.imshow("points", tmp_img)
        cv2.waitKey(1)
        

        # self.lastYPos = self.avgX0
        # self.lastWPos = self.avgX1
        # self.lastPos = int((self.lastYPos + self.lastWPos) / 2)

        # self.lastLineDist = np.abs(self.lastYPos - self.lastWPos)
        # self.distFromCenter = np.abs(avgX - 328) / 328

       
        self.move(robot, avgX, self.zaklad_rychlost, odchyl, self.lineDist)
            

    def camera_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

        self.muj_image = image
        cv2.imshow("camera", self.muj_self.muj_image)
        cv2.waitKey(1)
    
    def oddel_barev(self, img, lower, upper):
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      thresh = cv2.inRange(hsv, lower, upper)
      height, width = thresh.shape
      kernel = np.ones((2, 2), np.uint8)
      thresh_erode = cv2.erode(thresh, kernel)
      thresh_dilate = cv2.dilate(thresh_erode, kernel)
      
      return thresh_dilate

    def frame_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
      
        thresh_dilate = self.oddel_barev(image, (140, 40, 40), (180, 200, 200))

        cv2.imshow("frame", thresh_dilate)

        cv2.waitKey(1)

    def run(self):
        while not rospy.is_shutdown():
            pass



if __name__ == "__main__":
    main = Main()
    main.run()
