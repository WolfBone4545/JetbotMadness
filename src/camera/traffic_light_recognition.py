import utils
import cv2
import numpy as np


def traffic_light_recognition(image):
    detect_traffic_light(image)


def detect_traffic_light(img):
    blue, green, red = cv2.split(img)
    cimg = img.copy()

    #blur_g = cv2.GaussianBlur(green, (5, 5), 0)
    #ret, thresh_g = cv2.threshold(blur_g, 200, 255, cv2.THRESH_BINARY)

    #blur_r = cv2.GaussianBlur(red, (5, 5), 0)
    #ret, thresh_r = cv2.threshold(blur_r, 200, 255, cv2.THRESH_BINARY)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)

    circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, 20, param2=10, minRadius=0, maxRadius=30)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    cv2.imshow("thresh red", thresh)
    cv2.imshow("copy img", cimg)
    cv2.waitKey(1)



if __name__ == "__main__":
    utils.run_camera_with_callback(traffic_light_recognition)
