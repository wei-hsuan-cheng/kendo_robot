import numpy as np

LOGO_CAP_DIR = "/media/yu46656/08570A0408570A04/research/LOGO-CAP/logocap"

HOST = "10.42.0.1"
PORT = 8340

EXP_LP_RATIO = 0.5

MAX_KEYPOINT_VELOCITY = 500

DEPTH_OFFSET = np.array([
    150, # nose
    150, # l-eye
    150, # r-eye
    150, # l-ear
    150, # r-ear
    150, # l-shoulder
    150, # r-shoulder
    150, # l-elbow
    150, # r-elbow
    150, # l-wrist
    150, # r-wrist
    150, # l-hip
    150, # r-hip
    150, # l-knee
    150, # r-knee
    150, # l-ankle
    150, # r-ankle
], dtype=np.float32)

# POST_OFFSET = np.array([0, -150, 0], dtype=np.float32)

POST_OFFSET = np.array([
    [0, -200, 0], # nose
    [0, -200, 0], # l-eye
    [0, -200, 0], # r-eye
    [0, -200, 0], # l-ear
    [0, -200, 0], # r-ear
    [0, -200, 0], # l-shoulder
    [0, -200, 0], # r-shoulder
    [0, -200, 0], # l-elbow
    [0, -200, 0], # r-elbow
    [0, -200, 0], # l-wrist
    [0, -200, 0], # r-wrist
    [0, -200, 0], # l-hip
    [0, -200, 0], # r-hip
    [0, -200, 0], # l-knee
    [0, -200, 0], # r-knee
    [0, -200, 0], # l-ankle
    [0, -200, 0], # r-ankle
], dtype=np.float32)
