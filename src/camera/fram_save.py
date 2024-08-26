import cv2
import numpy as np
import utils

i = 0

def frame_saver(image):
    global i
    cv2.imshow("yaw", image)
    if cv2.waitKey(1) == ord("pline_follower"):
        cv2.imwrite(f"images/img_{i:02d}.png", image)
        print("Done")
        i += 1


### EXAMPLE USAGE ###
if __name__ == "__main__":
    utils.run_camera_with_callback(frame_saver)
