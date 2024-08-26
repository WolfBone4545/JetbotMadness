import skimage
import numpy as np
import cv2
import utils


def fetch():
    img = skimage.io.imread(utils.URL_server, as_gray=False)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(utils.K_server, utils.D_server, np.eye(3), utils.K_server, (img.shape[1], img.shape[0]), cv2.CV_16SC2)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    return img
