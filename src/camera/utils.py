import numpy as np
import cv2
from jetbot import Camera
import pathlib

# Load camera coefficients

config_path = pathlib.Path(__file__).parents[1]
matrix_path = config_path.joinpath("config/matrix.npy")

K = np.load(matrix_path)

distortion_path = config_path.joinpath("config/matrix.npy")

# Example distortion coefficients (D)
D = np.load(distortion_path)
IMG_SHAPE = (328, 246)
RESOLUTION_MODE = 2
ARUCO_MARKER_SIZE = 5.0


def camera_calib(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return image


def run_camera_with_callback(callback):
    def update(value):
        image = camera_calib(value["new"])
        callback(image)

    camera = Camera.instance(width=int(IMG_SHAPE[0] * RESOLUTION_MODE), height=int(IMG_SHAPE[1] * RESOLUTION_MODE), fps = 10)
    update({"new": camera.value})
    camera.observe(update, names="value")
