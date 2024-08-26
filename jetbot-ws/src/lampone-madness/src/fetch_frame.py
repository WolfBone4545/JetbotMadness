import skimage
import numpy as np
import cv2
import utils
import rospy
import cv_bridge
from sensor_msgs.msg import Image


class FrameFetcher:

    def __init__(self):
        self.rate = rospy.Rate(0.2)
        self.cv_bridge = cv_bridge.CvBridge()
        self.image_pub = rospy.Publisher("/frame", Image, queue_size=10)

        # Server settings
        self.URL_server = "http://192.168.100.22/image/image.png"

        def publish_frame(image):
            if rospy.is_shutdown():
                quit()
            img_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(img_msg)

        while not rospy.is_shutdown():
            img = skimage.io.imread(self.URL_server, as_gray=False)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            map1, map2 = cv2.fisheye.initUndistortRectifyMap(utils.K, utils.D, np.eye(3), utils.K,
                                                             (img.shape[1], img.shape[0]), cv2.CV_16SC2)
            img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            publish_frame(img)
            self.rate.sleep()


### EXAMPLE USAGE ###
if __name__ == "__main__":
    rospy.init_node("frame_node")
    frame_fetcher = FrameFetcher()
