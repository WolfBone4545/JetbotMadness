import utils
import cv2
import numpy as np


def traffic_light_recognition(image):
    detect_traffic_light(image)


def threshold(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    
    print(thresh)
    cut = thresh.shape[0]
    thresh[:cut, :] = 0

    return thresh


def detect_circles(thresh, green, red):
    circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, 20, param2=10, minRadius=0, maxRadius=30)
    circles_list = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles:
            green_c = green[i[1] - i[2] - 10: i[1] + i[2] + 10, i[0] - i[2] - 10: i[0] + i[2] + 10]
            red_c = red[i[1] - i[2] - 10: i[1] + i[2] + 10, i[0] - i[2] - 10: i[0] + i[2] + 10]

            circles_list.append(((i[0], i[1]), i[2], "red"))

            # cv2.imshow("test_green", green_c)
            # cv2.imshow("test_red", red_c)

            # cv2.waitKey(0)

            # draw the outer circle
            # cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            # cv2.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)

    return circles_list


def detect_traffic_light(img):
    blue, green, red = cv2.split(img)
    cimg = img.copy()
    thresh = threshold(img)
    circles = detect_circles(thresh, green, red)
    for circle in circles:
        cv2.circle(cimg, circle[0], circles[1], (0, 255, 0), 2)
        cv2.putText(cimg, 'OpenCV', (circle[0][0] - circles[1], circle[0][1] - circles[1]), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("thresh red", thresh)
    cv2.imshow("copy img", cimg)
    cv2.waitKey(1)


if __name__ == "__main__":
    utils.run_camera_with_callback(traffic_light_recognition)
