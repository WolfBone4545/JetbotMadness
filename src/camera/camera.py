from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np
import time

# Load camera coefficients
K = np.load("../config/matrix.npy")

# Example distortion coefficients (D)
D = np.load("../config/distortion.npy")
IMG_SHAPE = (328, 246)
RESOLUTION_MODE = 2


def update(value):
    img = value["new"]

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow("test", image)
    time.sleep(0.01)
    cv2.waitKey(1)


camera = Camera.instance(width=int(IMG_SHAPE[0]*RESOLUTION_MODE), height=int(IMG_SHAPE[1]*RESOLUTION_MODE))
update({"new": camera.value})
camera.observe(update, names="value")
