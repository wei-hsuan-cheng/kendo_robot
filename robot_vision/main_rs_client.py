import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import config

from queue import Queue
from collections import deque
import cv2 as cv
import socket

from utils import *
from SocketBuffer import *
from PoseEstimatorClientRS import PoseEstimatorClientRS

def main():
    img_h = 480
    img_w = 640
    fps = 30

    host = "10.42.0.1"
    port = config.PORT

    pec = PoseEstimatorClientRS(host, port)

    try:
        while True:
            pose = pec.Recv()

            if cv.waitKey(50) & 0xff == ord("q"):
                break
    except Exception as e:
        print(f"exception message = {e}")

if __name__ == "__main__":
    main()
