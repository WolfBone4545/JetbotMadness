
# Import required modules 
from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2 
import numpy as np 
import os 
import glob 

# Define the dimensions of checkerboard
CHECKERBOARD = (8, 6)

# Vector for 3D points
threedpoints = []

# Vector for 2D points
twodpoints = []
img_shape = (224, 224)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


#  3D points real world coordinates
objectp3d = np.zeros((1, CHECKERBOARD[0]
                      * CHECKERBOARD[1],
                      3), np.float32)
objectp3d[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                               0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

i = 0
max_i = 5


def compute_matrix():
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
        threedpoints, twodpoints, img_shape.shape[::-1], None, None)

    # Displaying required output
    print(" Camera matrix:")
    print(matrix)

    print("\n Distortion coefficient:")
    print(distortion)

    print("\n Rotation Vectors:")
    print(r_vecs)

    print("\n Translation Vectors:")
    print(t_vecs)


def update(value):
    if i == max_i:
        compute_matrix()

    image = value["new"]

    grayColor = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    # If desired number of corners are
    # found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(
                    grayColor, CHECKERBOARD,
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK +
                    cv2.CALIB_CB_NORMALIZE_IMAGE)

    # If desired number of corners can be detected then,
    # refine the pixel coordinates and display
    # them on the images of checker board
    if ret:
        threedpoints.append(objectp3d)

        # Refining pixel coordinates
        # for given 2d points.
        corners2 = cv2.cornerSubPix(
            grayColor, corners, (11, 11), (-1, -1), criteria)

        twodpoints.append(corners2)

        # Draw and display the corners
        image = cv2.drawChessboardCorners(image,
                                          CHECKERBOARD,
                                          corners2, ret)

        cv2.imshow('img', image)
        cv2.waitKey(0)

        i += 1


camera = Camera.instance(width=img_shape[0], height=img_shape[1])
update({"new": camera.value})
camera.observe(update, names="value")