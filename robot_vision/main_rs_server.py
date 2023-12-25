import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import config

from queue import Queue
from collections import deque
import cv2 as cv
import socket

from utils import *
from Server import *
from SocketBuffer import *
from PoseEstimator import PoseEstimator
from PoseEstimatorServerRS import PoseEstimatorServerRS

import pyrealsense2 as rs

def Sampling(m, i, j, ks):
    half_ks = ks // 2

    val = list()

    m_h, m_w = m.shape[:2]

    for di in range(-half_ks, half_ks+1):
        for dj in range(-half_ks, half_ks+1):
            i_ = i + di
            j_ = j + dj

            if 0 <= i_ and i_ < m_h and 0 <= j_ and j_ < m_w:
                val.append(m[i_, j_])

    val.sort()
    l = len(val)
    val = val[half_ks:half_ks]

def main():
    img_h = 480
    img_w = 640
    fps = 30
    m_to_mm = 1000
    mm_to_m = 1 / 1000

    pe = PoseEstimator()

    rs_pipe = rs.pipeline()
    rs_cfg  = rs.config()
    rs_align = rs.align(rs.stream.depth)

    rs_cfg.enable_stream(rs.stream.color, img_w, img_h, rs.format.bgr8, fps)
    rs_cfg.enable_stream(rs.stream.depth, img_w, img_h, rs.format.z16, fps)

    rs_profile = rs_pipe.start(rs_cfg)

    depth_sensor = rs_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    print(f"depth_scale = {depth_scale}")

    camera_params = NPLoad(f"{DIR}/camera_params.npy").item()
    camera_mat = camera_params["camera_mat"]
    camera_distort = camera_params["camera_distort"]

    align_homo = NPLoad(f"{DIR}/align_homo.npy").astype(np.float32)

    inv_camera_mat = np.linalg.inv(camera_mat)

    T_camera_to_base = NPLoad(f"{DIR}/T_camera_to_base.npy")

    host = config.HOST
    port = 8340

    pes = PoseEstimatorServerRS(host, port)

    pes.Start()

    try:
        while True:
            frame = rs_pipe.wait_for_frames()
            frame = rs_align.process(frame)

            color_frame = frame.get_color_frame()
            depth_frame = frame.get_depth_frame()

            color_frame = np.asanyarray(color_frame.get_data())
            depth_frame = np.asanyarray(depth_frame.get_data())

            depth_cm = cv.applyColorMap(cv.convertScaleAbs(depth_frame,
                                        alpha = 0.5), cv.COLORMAP_JET)

            depth_frame = depth_frame.reshape((img_h, img_w, 1))

            depth_frame = cv.medianBlur(depth_frame, 5)
            # depth_frame = cv.medianBlur(depth_frame, 5)
            # depth_frame = cv.medianBlur(depth_frame, 5)
            # depth_frame = cv.medianBlur(depth_frame, 5)

            depth_frame = depth_frame.reshape((img_h, img_w))

            img = cv.cvtColor(color_frame, cv.COLOR_BGR2RGB)

            poses, scores = pe.Estimate(img)
            target_idx = scores.argmax()
            score = scores[target_idx]
            pose = poses[target_idx]
            # [17, 3]

            keypoint = np.empty((17, 2), dtype=np.int32)
            keypoint[:, 0] = \
                np.round(pose[:, 0] * img_w).astype(np.int32).clip(0, img_w-1)
            keypoint[:, 1] = \
                np.round(pose[:, 1] * img_h).astype(np.int32).clip(0, img_h-1)

            keypoint_3d = np.empty((17, 3), np.float32)

            ds = np.empty((17,), dtype=np.float32)

            depth_offset = config.DEPTH_OFFSET

            for k in range(17):
                kp_w = keypoint[k, 0]
                kp_h = keypoint[k, 1]

                d = m_to_mm * depth_scale * depth_frame[kp_h, kp_w]
                d = max(1 * m_to_mm, min(d, 3 * m_to_mm)) + depth_offset[k]
                ds[k] = d

                x = np.array([[kp_w], [kp_h], [1]], dtype=np.float32)
                x = (inv_camera_mat @ x).reshape(-1)
                x *= d
                x = np.array([x[0], x[1], x[2], 1], dtype=np.float32) \
                    .reshape((4, 1))
                x = (T_camera_to_base @ x).reshape(-1)

                keypoint_3d[k, :] = x[:3]

            post_offset = config.POST_OFFSET

            keypoint_3d += post_offset

            pes.UpdatePose(score, keypoint_3d * mm_to_m)

            # print(ds.tolist())

            anno_img = AnnoPoses(img, poses, scores)
            anno_depth_cm = AnnoPoses(depth_cm, poses, scores)

            ShowImage("anno_img", anno_img)
            ShowImage("anno_depth_cm", anno_depth_cm)

            if cv.waitKey(1) & 0xff == ord("q"):
                break
    except Exception as e:
        print(f"exception message = {e}")

    pes.Stop()

if __name__ == "__main__":
    main()
