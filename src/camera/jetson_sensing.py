import utils
import cv2
import time


def sense_jetson(image):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    arucos, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    if ids is not None:
        for i, aruco in enumerate(arucos):
            # aruco dims
            corners = aruco.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # calc aruco dist
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(aruco, utils.ARUCO_MARKER_SIZE, utils.K, utils.D)
            aruco_dist = round(tvec[0][0][2], 2)
            aruco_id = ids[i][0]

            # drawing
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cv2.putText(image, f"id: {aruco_id}, dist: {aruco_dist}",
			(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
			0.5, (0, 255, 0), 2)
    
    cv2.imshow("test", image)
    cv2.waitKey(1)

if __name__ == "__main__":
    utils.run_camera_with_callback(sense_jetson)
