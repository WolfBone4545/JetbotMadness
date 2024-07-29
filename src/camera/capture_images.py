# Import required modules 
from jetbot import Robot, Camera, bgr8_to_jpeg
import cv2 
import numpy as np 
import os 
import glob 
import time

img_shape = (328*2, 224*2)
i = 0


def capture_frame(value):
    global i

    img = value["new"]

    cv2.imshow('img', img)
    cv2.waitKey(1)
    if 0xFF == ord('d'):
        i += 1
        cv2.imwrite(f"../imgs/img{i}.png", img)
        print("SAVE")
    elif 0xFF == ord('q'):
        camera.unobserve(capture_frame, names='value')
        time.sleep(0.1)
        exit(0)


camera = Camera.instance(width=img_shape[0], height=img_shape[1])
capture_frame({"new": camera.value})
camera.observe(capture_frame, names="value")
