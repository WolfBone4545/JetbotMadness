from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np
import time


def update(value):
    cv2.imshow("test", value)
    time.sleep(0.01)
    cv2.waitKey(0)


camera = Camera.instance(width=224, height=224)
camera.observe(update, names="value")
