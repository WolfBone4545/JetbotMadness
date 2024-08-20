import numpy as np
import cv2
from jetbot import Camera, bgr8_jpeg


# Load camera coefficients
K = np.load("./config/matrix.npy")

# Example distortion coefficients (D)
D = np.load("./config/distortion.npy")
IMG_SHAPE = (328, 246)
RESOLUTION_MODE = 2
ARUCO_MARKER_SIZE = 5.0

def camera_calib(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return image


def run_camera_with_callback(callback):
    def update(value):
        img = bgr8_jpeg(value["new"])
        image = camera_calib(img)
        callback(image)

    camera = Camera.instance(width=int(IMG_SHAPE[0] * RESOLUTION_MODE), height=int(IMG_SHAPE[1] * RESOLUTION_MODE), fps = 10)
    update({"new": camera.value})
    camera.observe(update, names="value")
