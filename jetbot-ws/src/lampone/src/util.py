import cv2
import numpy as np


def undistort(img):
    # Honorably stolen from Los Vocelos

    img_res = cv2.resize(img, (1920, 1440))

    # Define the distortion coefficients for experimentation
    k1 = -0.013  # Radial distortion coefficient
    k2 = 0.00014  # Radial distortion coefficient
    p1 = -0.0025  # Tangential distortion coefficient
    p2 = 0.0015  # Tangential distortion coefficient

    # Define the parameters for manual correction
    fov = 160  # Field of view (in degrees)
    dst_size = img_res.shape[:2][::-1]  # Destination image size (width, height)

    # Calculate the focal length based on the field of view
    focal_length = dst_size[0] / (2 * np.tan(np.radians(fov) / 2))

    # Generate a simple perspective transformation matrix
    K = np.array([[focal_length, 0, dst_size[0] / 2],
                [0, focal_length, dst_size[1] / 2],
                [0, 0, 1]])
    dist_coeffs = np.array([k1, k2, p1, p2])

    # Undistort the image using the specified coefficients
    undistorted_image = cv2.undistort(img_res, K, dist_coeffs)
    
    return undistorted_image


def separate_color(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)

    return mask


def get_closest_corner(corners, intersection_center):
    minimum = np.inf
    corner_index = 0
    corners = corners[0][0].astype(int)

    for i in range(len(corners)):
        x1 = corners[i][0]
        x2 = intersection_center[0]
        y1 = corners[i][1]
        y2 = intersection_center[1]

        x_dist = np.abs(x1 - x2)
        y_dist = np.abs(y1 - y2)

        dist = np.sqrt((x_dist * x_dist) + (y_dist * y_dist))

        if dist < minimum:
            minimum = dist
            corner_index = i

    return minimum, corner_index


def check_around_intersections(img, intersects, centers, width, height):
    shape = img.shape
    section_areas = []

    for i in range(len(intersects)):
        cx = centers[i][0]
        cy = centers[i][1]

        lower_height = cy - (height // 2)
        upper_height = cy + (height // 2)
        lower_width = cx - (width // 2)
        upper_width = cx + (width // 2)

        lower_height = np.clip(lower_height, 0, shape[0])
        upper_height = np.clip(upper_height, 0, shape[0])
        lower_width = np.clip(lower_width, 0, shape[1])
        upper_width = np.clip(upper_width, 0, shape[1])

        section_area = img.copy()[lower_height : upper_height, lower_width : upper_width]

        section_areas.append(section_area)

    return section_areas


def process_image(img, mask, pixels, mark):
    marked_img = img.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    intersections = []
    centers = []

    for i in range(len(contours)):
        area = cv2.contourArea(contours[i])

        if area > pixels:
            M = cv2.moments(contours[i])

            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            else:
                cx = -1
                cy = -1
            
            center = (cx, cy)

            if mark:
                cv2.drawContours(marked_img, contours, i, (0, 255, 0), 3)
                cv2.circle(marked_img, center, 3, (0, 0, 255), 2)

            centers.append(center)
            intersections.append(contours[i])

    return marked_img, intersections, centers


def process_intersection_areas(areas, lower_hsv, upper_hsv):    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    marked_areas = []

    for i in range(len(areas)):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(areas[i], aruco_dict, parameters = aruco_params)

        temp = areas[i].copy()

        thresh = separate_color(temp, lower_hsv, upper_hsv)
        
        out, intersections, centers = process_image(temp, thresh, 50, True)

        cv2.circle(temp, centers[0], 3, (0, 255, 255), 3)

        if len(corners) > 0:
            dist, index = get_closest_corner(corners, centers[0])
            coords = corners[0][0].astype(int)
            cv2.circle(temp, tuple(coords[index]), 3, (0, 0, 255), 3) # closest corner
            cv2.circle(temp, tuple(coords[0]), 3, (255, 0, 255), 3) # top left corner

        marked_areas.append(temp)

    return marked_areas
