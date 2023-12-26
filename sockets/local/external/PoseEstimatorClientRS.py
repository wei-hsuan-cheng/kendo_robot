import os

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import config
import cv2 as cv
import socket

from SocketBuffer import *

class PoseEstimatorClientRS:
    def __init__(self, addr, port):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((addr, port))

        self.sb_byte = SocketBufferByte(self.s)
        self.sb_arr = SocketBufferArr(self.sb_byte)

    def Recv(self):
        self.sb_byte.Send(int(0).to_bytes(1, "little"))

        # Received data.
        score = self.sb_arr.Recv(None)
        pose = self.sb_arr.Recv(None)

        return score, pose

def main():
    s = socket.socket()
    host = socket.gethostname()
    port = config.PORT
    s.connect((host, port))

    sb_byte = SocketBufferByte(s)
    sb_img = SocketBufferImage(sb_byte)

    rgb_in = cv.VideoCapture(f"{DIR}/anno_v.mp4")

    while True:
        ret, rgb_frame = rgb_in.read()

        if not ret:
            break

        print(f"send")
        sb_img.Send(rgb_frame)

    print(f"EOF")

    s.close()

if __name__ == "__main__":
    main()
