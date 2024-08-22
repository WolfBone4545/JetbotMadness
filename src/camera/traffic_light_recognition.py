import utils
import cv2
import numpy as np


def traffic_light_recognition(image):
    detect_traffic_light(image)


def detect_traffic_light(img):
    blue, green, red = cv2.split(img)

    blur_g = cv2.GaussianBlur(green, (5, 5), 0)
    ret, thresh_g = cv2.threshold(blur_g, 0, 255, cv2.THRESH_OTSU)

    blur_r = cv2.GaussianBlur(red, (5, 5), 0)
    ret, thresh_r = cv2.threshold(blur_r, 0, 255, cv2.THRESH_OTSU)

    cv2.imshow("thresh red", thresh_r)
    cv2.imshow("thresh green", thresh_g)
    cv2.waitKey(1)



if __name__ == "__main__":
    utils.run_camera_with_callback(traffic_light_recognition)
