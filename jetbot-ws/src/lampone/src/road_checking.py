import cv2
import numpy as np
import skimage.io
import util


im = skimage.io.imread("../../../../images/image.png")
undistorted_with_code = skimage.io.imread("../../../../images/undistorted_with_code.png")

img = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
img = util.undistort(im)

server = undistorted_with_code[200:1100, 500:1400]

cv2.imshow("server", server)

lower = (150, 40, 50) # Red HSV
upper = (180, 255, 255)

thresh = util.separate_color(server, lower, upper)

out, intersections, centers = util.process_image(server, thresh, 50, True)

areas = util.check_around_intersections(server, intersections, centers, 250, 250)

processed_areas = util.process_intersection_areas(areas, lower, upper)

for i in range(len(processed_areas)):
    cv2.imshow(f"temp_{i}", processed_areas[i])

cv2.waitKey(0)

cv2.destroyAllWindows()