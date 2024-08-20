import utils
import cv2
import numpy as np

def traffic_light_recognition(image):
    traffic_img = detect(image)

    cv2.imshow("test", traffic_img)
    cv2.waitKey(1)


def detect(img):
    cimg = img.copy()

    return cimg


if __name__ == "__main__":
    utils.run_camera_with_callback(traffic_light_recognition)
