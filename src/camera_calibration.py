# Import required modules
import cv2
import numpy as np
import os

# Define the dimensions of checkerboard
CHECKERBOARD = (8, 6)

# Vector for 3D points
threedpoints = []

# Vector for 2D points
twodpoints = []
img_shape = (328*2, 224*2)


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
path = "./imgs/"


def compute_matrix():
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(
        threedpoints, twodpoints, img_shape[::-1], None, None)

    # Displaying required output
    print(" Camera matrix:")
    print(matrix)

    print("\n Distortion coefficient:")
    print(distortion)

    # save matrices
    np.save(matrix, os.path.join(path, "matrix.npy"))
    np.save(distortion, os.path.join(path, "distortion.npy"))


for file in os.listdir(path):
    if file == ".gitkeep":
        continue

    image = cv2.imread(os.path.join(path, file), 0)

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
                                          corners2,
                                          ret)

        cv2.imshow('img', image)
        cv2.waitKey(0)

        i += 1

compute_matrix()
