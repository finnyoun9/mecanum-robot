"""三角测量：激光像素坐标 → 相机 3D 坐标."""
import numpy as np
import cv2
from laser_extractor import LaserLine


def triangulate(
    laser_line: LaserLine,
    camera_matrix: np.ndarray,
    light_plane: np.ndarray,  # [a, b, c, d]
) -> np.ndarray:
    """
    将激光线像素坐标转换为相机坐标系下的 3D 点。

    Args:
        laser_line: 提取到的激光线
        camera_matrix: 3x3 相机内参矩阵
        light_plane: (4,) 光平面参数 aX+bY+cZ+d=0

    Returns:
        points_3d: (N, 3) 相机坐标系下的 3D 点
    """
    n = light_plane[:3]  # 平面法向量
    d_param = light_plane[3]

    # 像素 → 归一化坐标 → 光线方向
    pts_2d = laser_line.points.reshape(-1, 1, 2).astype(np.float32)
    pts_norm = cv2.undistortPoints(pts_2d, camera_matrix, None)
    pts_norm = pts_norm.reshape(-1, 2)

    # 齐次方向向量
    rays = np.hstack([pts_norm, np.ones((len(pts_norm), 1))])

    # 光线与平面求交: t = -d / (n·ray)
    denom = rays @ n
    valid = np.abs(denom) > 1e-8
    t = np.full(len(rays), np.nan)
    t[valid] = -d_param / denom[valid]

    # 只保留正 t（前方的点）
    front = t > 0
    t = t[front]
    rays_valid = rays[front]

    points_3d = rays_valid * t[:, np.newaxis]

    return points_3d
