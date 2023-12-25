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

        score = self.sb_arr.Recv(None)
        pose = self.sb_arr.Recv(None)

        return score, pose
