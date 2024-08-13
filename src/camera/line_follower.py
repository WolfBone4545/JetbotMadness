from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2
import numpy as np

# Load camera coefficients
K = np.load("./config/matrix.npy")

# Example distortion coefficients (D)
D = np.load("./config/distortion.npy")
IMG_SHAPE = (328, 246)
RESOLUTION_MODE = 2


def _split_mask_into_n_vert_patches(mask, n):
    patches = []
    step = int(mask.shape[0] / n)
    for i in range(1, n):
        up_step = step * i
        if up_step > 656: up_step = 656

        prev_step = step * (i - 1)

        patches.append(mask[prev_step:up_step, :])

    return patches


def _compute_dev(patch):
    # computing line deviation

    contours, hierarchy = cv2.findContours(patch, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        print("Line not found")
        return -1, -1

    contour = max(contours, key=cv2.contourArea)

    M = cv2.moments(contour)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    except ZeroDivisionError:
        return -1, -1

    return cX, cY


def get_roi(img,
            vert_cutting_factor,
            corner_vert_factor,
            corner_hor_factor):
    vert_size = int(img.shape[0] * vert_cutting_factor)

    # cut image corners
    triangle_height = int(img.shape[0] * corner_vert_factor)
    triangle_width = int(img.shape[1] * corner_hor_factor)

    tri_img = img.copy()

    # left corner
    triangle_cnt1 = np.array([(0, vert_size + triangle_height),
                             (0, vert_size),
                             (triangle_width, vert_size)]).reshape(-1, 1, 2).astype(np.int32)
    cv2.drawContours(tri_img, [triangle_cnt1], 0, 0, -1)

    triangle_cnt2 = np.array([(img.shape[1], vert_size + triangle_height),
                             (img.shape[1], vert_size),
                             (img.shape[1] - triangle_width, vert_size)]).reshape(-1, 1, 2).astype(np.int32)
    cv2.drawContours(tri_img, [triangle_cnt2], 0, 0, -1)

    # cut using vert cutting factor
    img_mod = tri_img[vert_size:, :]

    return img_mod, vert_size

def thresh(img, iters, threshold):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, threshold, 255, cv2.THRESH_BINARY)

    mask = cv2.erode(thresh, None, iterations=iters)
    mask = cv2.dilate(mask, None, iterations=iters)

    return mask

def parse_patches(img, vert_width):
    patches = _split_mask_into_n_vert_patches(img, 10)
    point_dev = []
    for i, patch in enumerate(patches):
        x_rel, y_rel = _compute_dev(patch)
        if x_rel == -1:
            continue

        x = x_rel
        y = y_rel + i * patch.shape[0] + vert_width

        # calculate dev
        point_dev.append((x, y))

    return point_dev

def get_line(img, vert_width):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    green = img[:, :, 1]
    red = img[:, :, 2]

    only_white = thresh(gray, 2, 160)
    thresh_green = thresh(green, 3, 110)
    thresh_red = thresh(red, 3, 110)

    yellow_and_white = cv2.bitwise_and(thresh_green, thresh_red)

    only_yellow = cv2.bitwise_and(cv2.bitwise_not(only_white), yellow_and_white)
    only_yellow = cv2.erode(only_yellow, None, iterations=3)

    # split mask into different patches

    yellow_dev_points = parse_patches(only_yellow, vert_width)
    white_dev_points = parse_patches(only_white, vert_width)

    return yellow_dev_points, white_dev_points

def camera_calib(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    image = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return image


if __name__ == "__main__":
    def update(value):
        img = value["new"]
        image = camera_calib(img)

        img_mod, vert_split = get_roi(image, 0.5, 0.4, 0.3)

        cv2.imshow("triangle mask", img_mod)
        cv2.waitKey(1)

        yel_point_dev, white_point_dev = get_line(img_mod, vert_split)

        for dev in yel_point_dev:
            cv2.circle(image, (dev[0], dev[1]), 5, (255, 0, 0), -1)

        for dev in white_point_dev:
            cv2.circle(image, (dev[0], dev[1]), 5, (0, 0, 255), -1)

        cv2.imshow("test", image)
        cv2.waitKey(1)


    camera = Camera.instance(width=int(IMG_SHAPE[0]*RESOLUTION_MODE), height=int(IMG_SHAPE[1]*RESOLUTION_MODE), fps=10)
    update({"new": camera.value})
    camera.observe(update, names="value")
