from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np
import time


def update(value):
    img = value["new"]

    cv2.imshow("test", img)
    time.sleep(0.01)
    cv2.waitKey(1)


camera = Camera.instance(width=250, height=250)
update({"new": camera.value})
camera.observe(update, names="value")
