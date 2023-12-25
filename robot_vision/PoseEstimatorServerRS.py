import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import config
import cv2 as cv
import socket
import threading

from Server import *
from PoseEstimator import PoseEstimator
from SocketBuffer import *

class PoseEstimatorServerRS:
    def __init__(self, host, port):
        self.server = Server(host, port, self.SessionFunc)
        self.pe = PoseEstimator()

        self.prev_updated_time = None
        self.score = None
        self.pose = None

        self.lock = threading.Lock()

    def Start(self):
        self.server.Start()

    def Stop(self):
        self.server.Stop()

    def UpdatePose(self, score, pose):
        self.lock.acquire()

        score = np.array([score], np.float32)
        pose = np.array(pose, np.float32)

        try:
            if self.prev_updated_time is None:
                self.prev_updated_time = time.time()
                self.score = score
                self.pose = pose
                self.lock.release()
                return
        except Exception as e:
            print(f"exception when self.pose = pose.copy(): {e}")

        try:
            cur_time = time.time()
            delta_time = cur_time - self.prev_updated_time

            self.prev_updated_time = cur_time
            self.score = score

            lam = config.EXP_LP_RATIO ** delta_time
            max_kp_delta_norm = config.MAX_KEYPOINT_VELOCITY * delta_time

            for k in range(17):
                cur_kp = self.pose[k].copy()
                nxt_kp = pose[k]

                '''
                velocity = np.linalg.norm(cur_kp - nxt_kp) / delta_time

                lam = config.EXP_LP_RATIO ** velocity
                self.pose[k] = cur_kp * lam + nxt_kp * (1 - lam)
                '''


                kp_delta = (cur_kp * lam + nxt_kp * (1 - lam)) - cur_kp
                kp_delta_norm = float(np.linalg.norm(kp_delta))
                kp_delta *= (
                    max_kp_delta_norm * np.tanh(kp_delta_norm / max_kp_delta_norm) / kp_delta_norm)

                self.pose[k] = self.pose[k] + kp_delta

        except Exception as e:
            print(f"exception when self.score = np.array([score], np.float32): {e}")

        self.lock.release()

    def SessionFunc(self, _, conn, addr, end_callback):
        sb_byte = SocketBufferByte(conn)
        sb_arr = SocketBufferArr(sb_byte)

        while self.server.is_active():
            try:
                sb_byte.Recv(1, None)
            except Exception as e:
                print(f"exception when sb_byte.Recv(1, None): {e}")
                break

            try:
                self.lock.acquire()

                sb_arr.Send(self.score)
                sb_arr.Send(self.pose)

                self.lock.release()
            except Exception as e:
                print(f"exception when sb_arr.Send(self.pose.copy()): {e}")
                break

        end_callback()
