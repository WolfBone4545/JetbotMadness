import utils
import cv2


def detect_aruco(image):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    arucos, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)

    if ids is not None:
        return {
            "aruco": arucos,
            "id": ids
        }


def process_aruco(aruco, id):
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
    aruco_id = id

    middle_point_x = int(bottomLeft[0] + abs(bottomLeft[0] - bottomRight[0]) / 2)
    middle_point_y = int(bottomLeft[1] - abs(bottomLeft[1] - topLeft[1]) / 2)

    return (middle_point_x, middle_point_y), aruco_id, aruco_dist


def detect_arucos(image):
    aruco_list = []
    aruco_dict = detect_aruco(image)
    for i, aruco in enumerate(aruco_dict["aruco"]):
        aruco_id = aruco_dict["id"][i]

        middle_point, object_id, distance = process_aruco(aruco, aruco_id)
        aruco_list.append([middle_point, object_id, distance])


def aruco_detection(image):
    aruco_list = detect_arucos(image)
    print(aruco_list)

    cv2.imshow("test", image)
    cv2.waitKey(1)


if __name__ == "__main__":
    utils.run_camera_with_callback(aruco_detection)
