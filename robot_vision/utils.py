import cv2 as cv
import numpy as np
import pickle

from scipy.spatial.transform import Rotation as scipy_Rotation

class EMPTY_CLASS:
    pass

def GetRotMat(axis, ang):
    axis = np.asarray(axis).reshape(-1)
    assert axis.shape == (3,)

    return scipy_Rotation.from_rotvec(axis * (ang / np.linalg.norm(axis))) \
                         .as_matrix()

def GetHomoTransMat(trans):
    trans = np.asarray(trans).reshape(-1)
    assert trans.shape == (3,)

    ret = np.identity(4, dtype=trans.dtype)
    ret[:3, 3] = trans

    return ret

def GetHomoRotMat(axis, ang):
    axis = np.asarray(axis).reshape(-1)
    assert axis.shape == (3,)

    ret = np.identity(4)
    ret[:3, :3] = GetRotMat(axis, ang)

    return ret

def ImageToFrame(img):
    return cv.cvtColor(img, cv.COLOR_RGB2BGR)

def FrameToImage(frame):
    return cv.cvtColor(frame, cv.COLOR_BGR2RGB)

def ReadImage(path):
    return FrameToImage(cv.imread(path))

def WriteImage(path, img):
    cv.imwrite(path, ImageToFrame(img))

def ShowImage(title, img):
    cv.imshow(title, ImageToFrame(img))

def NPSave(filename, data):
    np.save(filename, data, allow_pickle=True)

def NPLoad(filename):
    return np.load(filename, allow_pickle=True)

def PickleSave(filename, data):
    with open(filename, "wb") as f:
        pickle.dump(data, f)

def PickleLoad(filename):
    with open(filename, "rb") as f:
        return pickle.load(f)

keypoint_idx_to_str = [
    "nose",
    "l-eye",
    "r-eye",
    "l-ear",
    "r-ear",
    "l-shoulder",
    "r-shoulder",
    "l-elbow",
    "r-elbow",
    "l-wrist",
    "r-wrist",
    "l-hip",
    "r-hip",
    "l-knee",
    "r-knee",
    "l-ankle",
    "r-ankle",
]

keypoint_str_to_idx = {s:i for i, s in enumerate(keypoint_idx_to_str)}

keypoint_links = [
    ["l-shoulder", "r-shoulder"],
    ["l-hip", "r-hip"],

    ["l-shoulder", "l-hip"],
    ["l-hip", "l-knee"],
    ["l-knee", "l-ankle"],

    ["l-shoulder", "l-elbow"],
    ["l-elbow", "l-wrist"],

    ["l-shoulder", "l-hip"],
    ["l-hip", "l-knee"],
    ["l-knee", "l-ankle"],

    ["r-shoulder", "r-hip"],
    ["r-hip", "r-knee"],
    ["r-knee", "r-ankle"],

    ["r-shoulder", "r-elbow"],
    ["r-elbow", "r-wrist"],

    ["r-shoulder", "r-hip"],
    ["r-hip", "r-knee"],
    ["r-knee", "r-ankle"],
]

def AnnoPoses(img, poses, scores):
    img_h, img_w, _ = img.shape

    img = img.copy()

    for pose, score in zip(poses, scores):
        if score < 0.01:
            continue

        # pose[17, 3] ratio of image
        # pose[k, 0] * img_w for w idx of keypoint k
        # pose[k, 1] * img_h for h idx of keypoint k

        keypoint = np.empty((17, 2), dtype=np.int32)
        keypoint[:, 0] = np.round(pose[:, 0] * img_w).astype(np.int32)
        keypoint[:, 1] = np.round(pose[:, 1] * img_h).astype(np.int32)

        xmin = keypoint[:, 0].min().item()
        ymin = keypoint[:, 1].min().item()
        xmax = keypoint[:, 0].max().item()
        ymax = keypoint[:, 1].max().item()

        # cv.rectangle(out_img, (xmin, ymin), (xmax, ymax), (255, 255, 255), 2)

        for keypoint_link in keypoint_links:
            cv.line(img,
                    keypoint[keypoint_str_to_idx[keypoint_link[0]], :],
                    keypoint[keypoint_str_to_idx[keypoint_link[1]], :],
                    (0, 255, 0), 2)

        for keypoint in pose:
            x = int(round(keypoint[0] * img_w))
            y = int(round(keypoint[1] * img_h))
            cv.circle(img, (x, y), 0, (255, 0, 0), 5)

    return img




