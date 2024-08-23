import utils
import cv2
import numpy as np


def traffic_light_recognition(image):
    traffic_lights = detect_traffic_light(image)
    for circle in traffic_lights:
        cv2.circle(image, circle[0], circle[1], (0, 255, 0), 2)
        cv2.putText(image, circle[2], (circle[0][0] - circle[1], circle[0][1] - circle[1]), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imshow("copy img", image)
    cv2.waitKey(1)


def threshold(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    
    cut = int(thresh.shape[0] / 2)
    thresh[:cut, :] = 0

    return thresh


def detect_circles(thresh, green, red):
    circles = cv2.HoughCircles(thresh, cv2.HOUGH_GRADIENT, 1, 20, param2=10, minRadius=0, maxRadius=40)
    circles_list = []
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles:
            circle = i[0]

            green_c = green[circle[1] - circle[2] - 10: circle[1] + circle[2] + 10, circle[0] - circle[2] - 10: circle[0] + circle[2] + 10]
            red_c = red[circle[1] - circle[2] - 10: circle[1] + circle[2] + 10, circle[0] - circle[2] - 10: circle[0] + circle[2] + 10]

            green_c = cv2.GaussianBlur(green_c, (21, 21), 0)
            red_c = cv2.GaussianBlur(red_c, (21, 21), 0)

            dev = np.subtract(green_c, red_c).mean()
            t_cls = ""
            if dev > 130: t_cls = "red"
            else: t_cls = "green"

            circles_list.append(((circle[0], circle[1]), circle[2], t_cls))

    return circles_list


def detect_traffic_light(img):
    blue, green, red = cv2.split(img)
    cimg = img.copy()
    thresh = threshold(img)
    circles = detect_circles(thresh, green, red)

    return circles


if __name__ == "__main__":
    utils.run_camera_with_callback(traffic_light_recognition)
