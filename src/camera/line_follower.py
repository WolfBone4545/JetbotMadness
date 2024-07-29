from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np
import time

# Load camera coefficients
K = np.load("./config/matrix.npy")

# Example distortion coefficients (D)
D = np.load("./config/distortion.npy")
IMG_SHAPE = (328, 246)
RESOLUTION_MODE = 2


def get_line(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh1 = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

    mask = cv2.erode(thresh1, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask


def undistort(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return image


def update(value):
    img = value["new"]
    image = undistort(img)

    image_mask = get_line(image)

    cv2.imshow("test", image_mask)
    time.sleep(0.01)
    cv2.waitKey(1)


camera = Camera.instance(width=int(IMG_SHAPE[0]*RESOLUTION_MODE), height=int(IMG_SHAPE[1]*RESOLUTION_MODE))
update({"new": camera.value})
camera.observe(update, names="value")
