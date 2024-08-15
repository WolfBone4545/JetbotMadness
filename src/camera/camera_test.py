import utils
import cv2
import numpy as np
import time


def callback(image):
    cv2.imshow("test", image)
    cv2.waitKey(1)


if __name__ == "__main__":
    utils.run_camera_with_callback()
