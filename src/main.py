# glob imports
from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np

from camera.line_follower import get_line, get_roi, camera_calib

# variables:
WIDTH = 656 # pls don't touch that
HEIGHT = 492 # pls don't touch that
FPS = 10


def update(value):
    img = value["new"]
    img = camera_calib(img)

    roi, cut_height = get_roi(img, 0.5, 0.2, 0.15)
    point_dev = get_line(roi, cut_height)

    # do your magic here :)
    pass


camera = Camera.instance(width=WIDTH, height=HEIGHT, fps=FPS)
update({"new": camera.value})
camera.observe(update, names="value")
