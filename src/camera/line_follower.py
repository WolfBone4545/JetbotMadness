import cv2
import numpy as np
import utils


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
        return -1, -1

    contour = max(contours, key=cv2.contourArea)

    M = cv2.moments(contour)
    try:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    except ZeroDivisionError:
        return -1, -1

    return cX, cY

def four_point_transform(image, rect, vert_size):
    # obtain a consistent order of the points and unpack them
    # individually
    (tl, tr, br, bl) = rect

    # compute the width of the new image, which will be the
    # maximum distance between bottom-right and bottom-left
    # x-coordiates or the top-right and top-left x-coordinates
    widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
    widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
    maxWidth = max(int(widthA), int(widthB))

    # compute the height of the new image, which will be the
    # maximum distance between the top-right and bottom-right
    # y-coordinates or the top-left and bottom-left y-coordinates
    heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
    heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
    maxHeight = max(int(heightA), int(heightB))

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, 0],
        [image.shape[0]-1, 0],
        [image.shape[0]-1, vert_size],
        [0, vert_size]], dtype="float32")

    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))

    # return the warped image
    return warped

def get_roi(img,
            vert_cutting_factor,
            corner_up_factor,
            corner_down_factor):
    vert_size = int(img.shape[0] * vert_cutting_factor)

    # cut image corners
    polygon_up = int(img.shape[0] * corner_up_factor)
    polygon_down = int(img.shape[1] * corner_down_factor)

    tri_img = img.copy()

    rect = np.array([
            [polygon_up, vert_size],
            [img.shape[0]-polygon_up, vert_size],
            [polygon_down, img.shape[1]-1],
            [img.shape[0]-polygon_down, img.shape[1]-1]], dtype="float32")

    img_mod = four_point_transform(tri_img, rect, vert_size)

    return img_mod, vert_size


def thresh(img, iters, threshold):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    ret, thresh = cv2.threshold(blur, threshold, 255, cv2.THRESH_OTSU)

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


def get_right_white_line(white_mask):
    result_mask = np.zeros_like(white_mask)
    contours, hierarchy = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    right_points = []

    if len(contours) == 0 or len(contours) == 3:
        return -1

    for contour in contours:
        right_points.append(tuple(contour[contour[:, :, 0].argmin()][0])[0])

    max_elem = max(right_points)
    right_line_index = right_points.index(max_elem)

    cv2.drawContours(result_mask, contours, right_line_index, 255, -1)
    return result_mask


def get_line(img, vert_width):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    green = img[:, :, 1]
    red = img[:, :, 2]

    only_white = thresh(gray, 2, 160)

    thresh_green = thresh(green, 3, 110)
    thresh_red = thresh(red, 3, 110)

    cv2.imshow("ow", only_white)
    cv2.imshow("gr", only_white)
    cv2.imshow("rd", only_white)

    yellow_and_white = cv2.bitwise_and(thresh_green, thresh_red)

    only_yellow = cv2.bitwise_and(cv2.bitwise_not(only_white), yellow_and_white)
    only_yellow = cv2.erode(only_yellow, None, iterations=3)

    only_white = get_right_white_line(only_white)

    if isinstance(only_white, int):
        return -1, -1

    cv2.imshow("xd", only_white)
    cv2.waitKey(1)

    # split mask into different patches

    yellow_dev_points = parse_patches(only_yellow, vert_width)
    white_dev_points = parse_patches(only_white, vert_width)

    return yellow_dev_points, white_dev_points


def line_follower(image):
    img_mod, vert_split = get_roi(image, 0.5, 0.4, 0.3)

    yel_point_dev, white_point_dev = get_line(img_mod, vert_split)
    if isinstance(yel_point_dev, int):
        return

    for dev in yel_point_dev:
        cv2.circle(image, (dev[0], dev[1]), 5, (255, 0, 0), -1)

    for dev in white_point_dev:
        cv2.circle(image, (dev[0], dev[1]), 5, (0, 0, 255), -1)

    cv2.imshow("test", image)
    cv2.waitKey(1)


if __name__ == "__main__":
    utils.run_camera_with_callback(line_follower)
