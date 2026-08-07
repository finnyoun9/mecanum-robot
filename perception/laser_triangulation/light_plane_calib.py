"""光平面标定：棋盘格法求激光平面在相机坐标系下的方程 aX+bY+cZ+d=0."""
import cv2
import numpy as np
import json
from pathlib import Path
from laser_extractor import extract_laser_line, Method


def calibrate_light_plane(
    image_paths: list[str],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    chessboard_size: tuple[int, int] = (9, 6),
    square_size: float = 25.0,  # mm
    laser_threshold: int = 80,
) -> np.ndarray:
    """
    标定光平面。

    Returns:
        plane: np.ndarray of shape (4,) — [a, b, c, d] where aX+bY+cZ+d=0
    """
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    all_laser_pts_3d = []

    for img_path in image_paths:
        img = cv2.imread(img_path)
        if img is None:
            print(f"WARNING: Cannot read {img_path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if not ret:
            print(f"WARNING: Chessboard not found in {img_path}")
            continue

        # 亚像素精化
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 解棋盘格位姿 (PnP)
        ret_pnp, rvec, tvec = cv2.solvePnP(objp, corners_sub, camera_matrix, dist_coeffs)
        if not ret_pnp:
            continue

        # 提取激光线
        laser = extract_laser_line(img, method=Method.CENTROID, threshold=laser_threshold)

        if len(laser.points) < 10:
            continue

        # 反投影：光线方向 = inv(K) * [u, v, 1]^T
        pts_2d = laser.points.reshape(-1, 1, 2).astype(np.float32)
        pts_norm = cv2.undistortPoints(pts_2d, camera_matrix, dist_coeffs)
        pts_norm = pts_norm.reshape(-1, 2)
        rays_h = np.hstack([pts_norm, np.ones((len(pts_norm), 1))])  # (N, 3)

        # 棋盘格平面方程：n_cam·X + d_cam = 0
        R, _ = cv2.Rodrigues(rvec)
        n_cam = R[:, 2]
        d_cam = float(np.dot(n_cam, tvec.flatten()))

        # 光线-平面求交
        for ray in rays_h:
            denom = np.dot(n_cam, ray)
            if abs(denom) < 1e-8:
                continue
            t = -d_cam / denom
            if t <= 0:
                continue
            pt_3d = t * ray
            all_laser_pts_3d.append(pt_3d)

    if len(all_laser_pts_3d) < 10:
        raise RuntimeError(f"Not enough 3D laser points: {len(all_laser_pts_3d)}")

    pts = np.array(all_laser_pts_3d)

    # SVD 拟合平面
    centroid = pts.mean(axis=0)
    pts_centered = pts - centroid
    _, _, Vt = np.linalg.svd(pts_centered)
    normal = Vt[2]
    d_param = -np.dot(normal, centroid)

    # 确保法向量朝向相机
    if normal[2] < 0:
        normal = -normal
        d_param = -d_param

    plane = np.array([normal[0], normal[1], normal[2], d_param])
    print(f"Light plane: {plane[0]:.6f}X + {plane[1]:.6f}Y + {plane[2]:.6f}Z + {plane[3]:.6f} = 0")
    print(f"  Fitted from {len(pts)} 3D points across {len(image_paths)} images")

    return plane


def save_calibration(plane: np.ndarray, path: str) -> None:
    """保存光平面标定结果."""
    data = {
        "plane": plane.tolist(),
        "equation": f"{plane[0]:.6f}X + {plane[1]:.6f}Y + {plane[2]:.6f}Z + {plane[3]:.6f} = 0"
    }
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"Saved: {path}")


def load_calibration(path: str) -> np.ndarray:
    """加载光平面标定结果."""
    with open(path) as f:
        data = json.load(f)
    return np.array(data["plane"])
