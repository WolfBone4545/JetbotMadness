import utils
import cv2


def sense_jetson(image):
    cv2.imshow("test", image)
    cv2.waitKey(1)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, rejected = detector.detectMarkers(image)
    print(corners)
    if ids is not None:

        marker_size_cm = 15.0 # TODO
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size_cm, utils.K, utils.D)
        print(rvec, tvec)

if __name__ == "__main__":
    utils.run_camera_with_callback(sense_jetson)
