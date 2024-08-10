from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np
import time

# Load camera coefficients
K = np.load("./config/matrix.npy")

# Example distortion coefficients (D)
D = np.load("./config/distortion.npy")
IMG_SHAPE = (328, 246)
RESOLUTION_MODE = 2


def split_mask_into_n_vert_patches(mask, n):
    patches = []
    step = int(mask.shape[0] / n)
    for i in range(1, n):
        up_step = step * i
        prev_step = step * (i - 1)

        patches.append(mask[prev_step:up_step, :])

    return patches


def compute_dev(patch):
    # computing line deviation

    contours, hierarchy = cv2.findContours(patch, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("Line not found")
        return

    contour = max(contours, key=cv2.contourArea)

    M = cv2.moments(contour)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    return cX, cY


def get_line(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    ret, thresh1 = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)

    mask = cv2.erode(thresh1, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # cut image in half
    input_mask = mask[int(mask.shape[0] / 2):, :]

    # split mask into different patches
    patches = split_mask_into_n_vert_patches(input_mask, 4)
    point_dev = []
    for patch in patches:
        x, y = compute_dev(patch)

        # calculate relative dev
        point_dev.append((x / input_mask.shape[1], y / input_mask.shape[0]))

        # note into patch
        cv2.circle(patch, (x, y), 5, 125, -1)

    return point_dev, patches[0]


def undistort(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return image


def update(value):
    img = value["new"]
    image = undistort(img)

    point_dev, image_mask = get_line(image)
    print(point_dev)

    cv2.imshow("test", image_mask)
    time.sleep(0.01)
    cv2.waitKey(1)

if __name__ == "__main__":
    camera = Camera.instance(width=int(IMG_SHAPE[0]*RESOLUTION_MODE), height=int(IMG_SHAPE[1]*RESOLUTION_MODE))
    update({"new": camera.value})
    camera.observe(update, names="value")
