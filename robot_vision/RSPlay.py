import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import numpy as np
import cv2 as cv
import time

from utils import *

def main():
    time_stamp = "1702440919172"

    color_filename = f"{DIR}/color_frames_{time_stamp}.npy"
    depth_filename = f"{DIR}/depth_frames_{time_stamp}.npy"

    color_frames = NPLoad(color_filename)
    depth_frames = NPLoad(depth_filename)

    img_h = 480
    img_w = 640
    fps = 30

    load_saved_poses = True

    poses_filename = f"{DIR}/poses_{time_stamp}.pickle"
    scores_filename = f"{DIR}/scores_{time_stamp}.pickle"

    poses = PickleLoad(poses_filename)
    scores = PickleLoad(scores_filename)

    poses_iter = iter(poses)
    scores_iter = iter(scores)

    prev_frame_time = None

    for color_frame, depth_frame in zip(color_frames, depth_frames):
        prev_frame_time = time.time()

        depth_cm = cv.applyColorMap(cv.convertScaleAbs(depth_frame,
                                    alpha = 0.5), cv.COLORMAP_JET)

        color_img = FrameToImage(color_frame)

        cur_poses = next(poses_iter)
        cur_scores = next(scores_iter)

        anno_img = AnnoPoses(color_img, cur_poses, cur_scores)

        cv.imshow("color_frame", color_frame)
        cv.imshow("depth_frame", depth_cm)
        cv.imshow("anno_frame", ImageToFrame(anno_img))

        poses.append(cur_poses.copy())
        scores.append(cur_scores.copy())

        delay = max(1, int((1 / fps - time.time() + prev_frame_time) * 1000))

        if cv.waitKey(delay) & 0xff == ord("q"):
            break

if __name__ == "__main__":
    main()
