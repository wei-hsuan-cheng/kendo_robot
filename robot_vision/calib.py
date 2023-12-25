import os
import sys

DIR = os.path.dirname(os.path.abspath(__file__)).replace("\\", "/")

import numpy as np
import cv2 as cv
import math
import itertools
import random

from utils import *

GLOBAL = EMPTY_CLASS()

GLOBAL.chessboard = EMPTY_CLASS()

GLOBAL.chessboard.img_h = 480
GLOBAL.chessboard.img_w = 640
GLOBAL.chessboard.pattern_h = 6
GLOBAL.chessboard.pattern_w = 8
GLOBAL.chessboard.grid_len = 25

GLOBAL.RAD_TO_DEG = 180 / math.pi
GLOBAL.DEG_TO_RAD = math.pi / 180

def MakeTFromRt(R, t):
    assert R.shape == (3, 3)
    assert t.shape == (3, 1)

    T = np.identity(4, dtype=R.dtype)
    T[:3, :3] = R
    T[:3, 3] = t[:, 0]

    return T

def PointsToHomoPoints(points):
    # points[P, N]

    P, N = points.shape

    ret = np.empty((P+1, N), dtype=points.dtype)
    ret[:P, :] = points
    ret[P, :] = 1

    return ret

def FindCornerPoints(imgs):
    per_obj_points = np.empty(
        (GLOBAL.chessboard.pattern_h * GLOBAL.chessboard.pattern_w, 3),
        dtype=np.float32)
    per_obj_points[:, :2] = \
        np.mgrid[:GLOBAL.chessboard.pattern_h, :GLOBAL.chessboard.pattern_w] \
        .transpose().reshape((-1, 2)) * GLOBAL.chessboard.grid_len
    per_obj_points[:, 2] = 0

    obj_points = list()
    img_points = list()

    for img in imgs:
        img = img.copy()

        # cv.imshow("img", img)
        # cv.waitKey(0)

        gray = cv.cvtColor(img, cv.COLOR_RGB2GRAY)

        ret, corners = cv.findChessboardCorners(
            gray,
            (GLOBAL.chessboard.pattern_h, GLOBAL.chessboard.pattern_w),
            None)

        if not ret:
            continue

        obj_points.append(per_obj_points)

        corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1),
            (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 40, 0.001)).reshape((-1, 2))

        img = cv.drawChessboardCorners(
            img, (GLOBAL.chessboard.pattern_h, GLOBAL.chessboard.pattern_w),
            corners, ret)

        # cv.imshow("checkerboard img", img)
        # cv.waitKey(0)

        img_points.append(corners)

    obj_points = np.stack(obj_points, axis=0).astype(np.float32)
    img_points = np.stack(img_points, axis=0).astype(np.float32)

    return obj_points, img_points

def CameraCalib(imgs):
    obj_points, img_points = FindCornerPoints(imgs)

    ret, camera_mat, camera_distort, rvecs, tvecs = cv.calibrateCamera(
        obj_points, img_points,
        (GLOBAL.chessboard.img_w, GLOBAL.chessboard.img_h),
        None, None)

    return camera_mat, camera_distort

def GetMKOfAXXB(AR, At, BR, Bt):
    # AR[3, 3]
    # At[3, 1]
    # BR[3, 3]
    # Bt[3, 1]

    assert AR.shape == (3, 3)
    assert At.shape == (3, 1)
    assert BR.shape == (3, 3)
    assert Bt.shape == (3, 1)

    MA = np.zeros((12, 12), dtype=AR.dtype)

    for i, j in itertools.product(range(3), range(4)):
        MA[4*i+j, 0+j] = AR[i, 0]
        MA[4*i+j, 4+j] = AR[i, 1]
        MA[4*i+j, 8+j] = AR[i, 2]

    KA = np.zeros((12, 1), dtype=At.dtype)

    for i in range(3):
        KA[4*i+3] = At[i, 0]

    BT = MakeTFromRt(BR, Bt)

    MB = np.zeros((12, 12), dtype=BR.dtype)

    for i in range(3):
        MB[4*i:4*i+4, 4*i:4*i+4] = BT.transpose()

    '''

    [ AR At ][ XR Xt ] = [ XR Xt ][ BR Bt ]
    [  0  1 ][  0  1 ] = [  0  1 ][  0  1 ]

    MA * X + KA = MB * X

    (MA - MB) * X = -KA

    '''

    return MA - MB, -KA

def RefineToRot(M):
    assert M.shape == (3, 3)
    U, _, Vh = np.linalg.svd(M)
    return U @ Vh

def MyCalicaretHandEye(R_gripper2base, t_gripper2base,
                       R_target2cam, t_target2cam):
    N = R_gripper2base.shape[0]
    dtype = R_gripper2base.dtype

    assert R_gripper2base.shape == (N, 3, 3)
    assert t_gripper2base.shape == (N, 3, 1)
    assert R_target2cam.shape == (N, 3, 3)
    assert t_target2cam.shape == (N, 3, 1)

    T_gripper2base = np.empty((N, 4, 4), dtype=dtype)
    T_target2cam = np.empty((N, 4, 4), dtype=dtype)

    for i in range(N):
        T_gripper2base[i] = MakeTFromRt(R_gripper2base[i], t_gripper2base[i])
        T_target2cam[i] = MakeTFromRt(R_target2cam[i], t_target2cam[i])

    pairs = list(itertools.combinations(range(N), 2))
    clusters = list(itertools.combinations(pairs, len(pairs)))

    As = np.empty((N, N, 4, 4), dtype=dtype)
    Bs = np.empty((N, N, 4, 4), dtype=dtype)

    for a, b in pairs:
        As[a, b] = np.linalg.inv(T_gripper2base[b]) @ T_gripper2base[a]
        Bs[a, b] = T_target2cam[b] @ np.linalg.inv(T_target2cam[a])

    best_num_of_inliers = 0
    best_X = None

    test_num = len(clusters)

    # for test_i in range(test_num):
        # p1, p2 = random.choice(pair_pairs)

    def Solve(ps):
        cur_As = list(As[p[0], p[1]] for p in ps)
        cur_Bs = list(Bs[p[0], p[1]] for p in ps)

        Ms = list()
        Ks = list()

        for A, B in zip(cur_As, cur_Bs):
            M, K = GetMKOfAXXB(A[:3, :3], A[:3, 3:], B[:3, :3], B[:3, 3:])

            Ms.append(M)
            Ks.append(K)

        M = np.stack(Ms).reshape((-1, 12))
        K = np.stack(Ks).reshape((-1, 1))

        X = np.concatenate([
            np.linalg.lstsq(M, K, rcond=None)[0].reshape((3, 4)),
            np.array([[0, 0, 0, 1]])], axis=0)

        X[:3, :3] = RefineToRot(X[:3, :3])

        return X

    def IsInlier(err):
        return err < 50

    for ps in clusters:
        X = Solve(ps)

        num_of_inliers = 0

        for a, b in pairs:
            err = ((As[a, b] @ X - X @ Bs[a, b])**2)[:3, :].mean()**0.5

            if IsInlier(err):
                num_of_inliers += 1

        if best_num_of_inliers < num_of_inliers:
            best_num_of_inliers = num_of_inliers
            best_X = X

    print(f"best_X = {best_X}")

    inliers = list()

    for a, b in pairs:
        err = ((As[a, b] @ X - X @ Bs[a, b])**2)[:3, :].mean()**0.5

        if IsInlier(err):
            inliers.append((a, b))

    best_X = Solve(inliers)

    return best_X[:3, :3], best_X[:3, 3:]

def HandEyeCalib(camera_mat, camera_distort, Ts_base_to_gripper, imgs):
    obj_points, img_points = FindCornerPoints(imgs)

    Ts_obj_to_camera = list()

    for T_base_to_gripper, img, cur_obj_points, cur_img_points \
    in zip(Ts_base_to_gripper, imgs, obj_points, img_points):
        success, rvec, tvec = cv.solvePnP(
            objectPoints=cur_obj_points,
            imagePoints=cur_img_points[:, None, :],
            cameraMatrix=camera_mat,
            distCoeffs=camera_distort,
            flags=cv.SOLVEPNP_EPNP)

        T = np.identity(4)
        T[:3, :3] = cv.Rodrigues(rvec)[0].reshape((3, 3))
        T[:3, 3] = tvec.reshape(-1)

        Ts_obj_to_camera.append(T)

    Ts_obj_to_camera = np.stack(Ts_obj_to_camera, axis=0)

    if True:
        R, t = cv.calibrateHandEye(
            R_gripper2base=Ts_base_to_gripper[:, :3, :3],
            t_gripper2base=Ts_base_to_gripper[:, :3, 3:],
            R_target2cam=Ts_obj_to_camera[:, :3, :3],
            t_target2cam=Ts_obj_to_camera[:, :3, 3:],
            # method=cv.CALIB_HAND_EYE_TSAI
            # method=cv.CALIB_HAND_EYE_PARK
            method=cv.CALIB_HAND_EYE_HORAUD
            # method=cv.CALIB_HAND_EYE_ANDREFF
            # method=cv.CALIB_HAND_EYE_DANIILIDIS
        )
    else:
        R, t = MyCalicaretHandEye(
            R_gripper2base=Ts_base_to_gripper[:, :3, :3],
            t_gripper2base=Ts_base_to_gripper[:, :3, 3:],
            R_target2cam=Ts_obj_to_camera[:, :3, :3],
            t_target2cam=Ts_obj_to_camera[:, :3, 3:],
        )

    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(-1)

    T_camera_to_base = T

    return T_camera_to_base

def main():
    target_Cpose = [
        (227.0, -412.0, 177.0, 90.0, 0.0, -60.0),
        (181.47, -594.01, 224.79, 89.11, -8.35, -34.42),
        (186.79, -422.13, 203.07, 100.85, -0.1, -80.82),
        (330.47, -314.68, 252.24, 104.03, -3.55, -56.43),
        (278.93, -454.98, 315.46, 89.62, 15.66, -29.09),
        (219.3, -389.65, 190.19, 94.28, -25.89, -80.85),
        (318.99, -509.51, 68.04, 94.08, -12.93, -48.74),
        (282.31, -387.05, 91.58, 105.81, -10.51, -93.1),
        (285.95, -500.72, 204.98, 87.42, 11.14, -39.65),
        (256.91, -506.04, 174.18, 107.86, -25.41, -73.66),
    ]

    img_num = len(target_Cpose)

    Ts_base_to_gripper = list()

    imgs = list()

    for i in range(img_num):
        if i in [5, 9]:
            continue

        imgs.append(ReadImage(f"{DIR}/hand_eye_calib_images/img_{i}.png"))

        pose = target_Cpose[i]

        T = np.identity(4)
        T = GetHomoRotMat([1, 0, 0], pose[3] * GLOBAL.DEG_TO_RAD) @ T
        T = GetHomoRotMat([0, 1, 0], pose[4] * GLOBAL.DEG_TO_RAD) @ T
        T = GetHomoRotMat([0, 0, 1], pose[5] * GLOBAL.DEG_TO_RAD) @ T
        T = GetHomoTransMat(pose[0:3]) @ T

        T = np.linalg.inv(T)

        Ts_base_to_gripper.append(T.copy())

    Ts_base_to_gripper = np.stack(Ts_base_to_gripper, axis=0)

    camera_mat, camera_distort = None, None

    if False:
        camera_mat, camera_distort = CameraCalib(imgs)

        NPSave(f"{DIR}/camera_params", {"camera_mat": camera_mat,
                                    "camera_distort": camera_distort,})
    else:
        camera_params = NPLoad(f"{DIR}/camera_params.npy").item()
        camera_mat = camera_params["camera_mat"]
        camera_distort = camera_params["camera_distort"]

    print(f"camera_mat =\n{camera_mat}")
    print(f"camera_distort =\n{camera_distort}")

    T_camera_to_base = HandEyeCalib(
        camera_mat, camera_distort,
        Ts_base_to_gripper,
        imgs)

    print(f"T_camera_to_base =\n{T_camera_to_base}")

    NPSave(f"{DIR}/T_camera_to_base.npy", T_camera_to_base)

def main2():
    XR = np.random.rand(3, 3)
    Xt = np.random.rand(3, 1)
    X = MakeTFromRt(XR, Xt)

    N = 3

    ARs = [np.random.rand(3, 3) for _ in range(N)]
    Ats = [np.random.rand(3, 1) for _ in range(N)]
    As = [MakeTFromRt(AR, At) for AR, At in zip(ARs, Ats)]

    # A X = X B
    Bs = [np.linalg.inv(X) @ A @ X for A in As]
    BRs = [B[:3, :3] for B in Bs]
    Bts = [B[:3, 3:] for B in Bs]

    Ms = list()
    Ks = list()

    for AR, At, BR, Bt in zip(ARs, Ats, BRs, Bts):
        M, K = GetMKOfAXXB(AR, At, BR, Bt)
        # M[12, 12]
        # K[12, 1]

        Ms.append(M)
        Ks.append(K)

    stack_M = np.stack(Ms).reshape((-1, 12))
    stack_K = np.stack(Ks).reshape((-1, 1))

    re_X = np.linalg.lstsq(stack_M, stack_K, rcond=None)[0].reshape((3, 4))

    re_X = MakeTFromRt(re_X[:, :3], re_X[:, 3:])

    print(f"X=\n{X}")
    print(f"re_X=\n{re_X}")

def main3():
    N = 5

    pairs = itertools.combinations(range(N), 2)

    print(list(pairs))

if __name__ == "__main__":
    main()
