from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np
import time

K = np.array([[1580.0, 0.0, 1640.0],
              [0.0, 1580.0, 1232.0],
              [0.0, 0.0, 1.0]])

# Example distortion coefficients (D)
D = np.array([0.03, -0.05, 0.002, 0.002])

def update(value):
    img = value["new"]

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow("test", image)
    time.sleep(0.01)
    cv2.waitKey(1)


camera = Camera.instance(width=224, height=224)
update({"new": camera.value})
camera.observe(update, names="value")
