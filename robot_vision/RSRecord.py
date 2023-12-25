import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import numpy as np
import cv2 as cv
import time

import pyrealsense2 as rs

def main():
    H = 480
    W = 640
    FPS = 30

    rs_pipe = rs.pipeline()
    rs_cfg  = rs.config()

    rs_cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    rs_cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)

    rs_pipe.start(rs_cfg)

    color_frames = list()
    depth_frames = list()

    while True:
        frame = rs_pipe.wait_for_frames()

        color_frame = frame.get_color_frame()
        depth_frame = frame.get_depth_frame()

        color_frame = np.asanyarray(color_frame.get_data())
        depth_frame = np.asanyarray(depth_frame.get_data())

        color_frames.append(color_frame)
        depth_frames.append(depth_frame)

        if cv.waitKey(1) & 0xff == ord("q"):
            break

    time_stamp = int(time.time() * 1000)

    np.save(f"{DIR}/color_frames_{time_stamp}", color_frames)
    np.save(f"{DIR}/depth_frames_{time_stamp}", depth_frames)

if __name__ == "__main__":
    main()
