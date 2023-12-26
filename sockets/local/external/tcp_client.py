#import os
#import sys
#DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")
import config
#from queue import Queue
#from collections import deque
import cv2 as cv
#import socket
from utils import *
from SocketBuffer import *
from PoseEstimatorClientRS import PoseEstimatorClientRS

import ws_server as wsServer
import json
import time


async def clientThread(stopQueue):
    # img_h = 480
    # img_w = 640
    # fps = 30
    host = "10.42.0.1"
    # port = config.PORT
    port = 8340
    while stopQueue.empty(): 
        try:
            print("[TCP] Wait server...")
            pec = PoseEstimatorClientRS(host, port)
            while stopQueue.empty(): 
                print("[TCP] Wait recv...")
                time.sleep(0.07)
                score, pose = pec.Recv()
                message = json.dumps({
                    "score": score.tolist(), # 0.02
                    "pose": pose.tolist()
                })
                if await wsServer.sendToClient(message):
                   # print("[WS] Sent %s"%message)
                   print("[WS] Sent")
                   # pass
                else:
                    print("[WS] Attempted send, no client")
        except DisconnectionException:
            print("[TCP] Disconnected by server")
            return
        except Exception as e:
            print(f"exception message = {e}")
