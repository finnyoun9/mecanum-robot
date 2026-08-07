"""激光线中心提取：灰度重心法 + 高斯拟合法 + Steger 算法."""
import cv2
import numpy as np
from dataclasses import dataclass
from enum import Enum, auto


class Method(Enum):
    CENTROID = auto()   # 灰度重心法
    GAUSSIAN = auto()   # 高斯拟合法
    STEGER = auto()     # Steger 亚像素


@dataclass
class LaserLine:
    """提取到的激光线."""
    points: np.ndarray       # (N, 2) 亚像素坐标 (col, row)
    method: Method
    image_shape: tuple[int, int]


def extract_laser_line(
    image: np.ndarray,
    method: Method = Method.CENTROID,
    roi: tuple[int, int, int, int] | None = None,
    threshold: int = 50,
) -> LaserLine:
    """
    从图像中提取激光线中心。

    Args:
        image: 灰度图或 BGR 图
        method: 提取方法
        roi: (x, y, w, h) 感兴趣区域，None = 全图
        threshold: 亮度阈值，低于此值的像素忽略

    Returns:
        LaserLine 对象
    """
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()

    if roi is not None:
        x, y, w, h = roi
        gray = gray[y:y+h, x:x+w]
        offset_x, offset_y = x, y
    else:
        offset_x, offset_y = 0, 0

    if method == Method.CENTROID:
        points = _centroid_method(gray, threshold)
    elif method == Method.GAUSSIAN:
        points = _gaussian_method(gray, threshold)
    elif method == Method.STEGER:
        points = _steger_method(gray, threshold)
    else:
        raise ValueError(f"Unknown method: {method}")

    if len(points) > 0:
        points[:, 0] += offset_x
        points[:, 1] += offset_y

    return LaserLine(points=points, method=method, image_shape=image.shape[:2])


def _centroid_method(gray: np.ndarray, threshold: int) -> np.ndarray:
    """灰度重心法：对每一列，以灰度为权重计算行方向的质心."""
    h, w = gray.shape
    points = []

    for col in range(w):
        column = gray[:, col].astype(np.float64)
        mask = column > threshold
        if not np.any(mask):
            continue

        intensities = column[mask]
        rows = np.arange(h, dtype=np.float64)[mask]
        centroid_row = np.sum(rows * intensities) / np.sum(intensities)
        points.append([float(col), float(centroid_row)])

    return np.array(points, dtype=np.float64)


def _gaussian_method(gray: np.ndarray, threshold: int) -> np.ndarray:
    """高斯拟合法：对每列灰度剖面做高斯拟合求峰值."""
    from scipy.optimize import curve_fit

    def gaussian(x, a, mu, sigma, b):
        return a * np.exp(-(x - mu)**2 / (2 * sigma**2)) + b

    h, w = gray.shape
    points = []
    rows = np.arange(h, dtype=np.float64)

    for col in range(w):
        column = gray[:, col].astype(np.float64)
        peak_idx = np.argmax(column)
        if column[peak_idx] < threshold:
            continue

        # 取峰值附近 ±10 像素做拟合
        lo = max(0, peak_idx - 10)
        hi = min(h, peak_idx + 10)
        x_data = rows[lo:hi]
        y_data = column[lo:hi]

        try:
            p0 = [column[peak_idx], float(peak_idx), 2.0, 0.0]
            popt, _ = curve_fit(gaussian, x_data, y_data, p0=p0, maxfev=2000)
            mu = popt[1]
            if lo <= mu <= hi:
                points.append([float(col), float(mu)])
        except Exception:
            # 拟合失败时回退到灰度重心
            mask = column > threshold
            if np.any(mask):
                r = np.sum(rows[mask] * column[mask]) / np.sum(column[mask])
                points.append([float(col), float(r)])

    return np.array(points, dtype=np.float64)


def _steger_method(gray: np.ndarray, threshold: int) -> np.ndarray:
    """Steger 算法：Hessian 矩阵求激光条纹亚像素中心."""
    h, w = gray.shape
    points = []

    # 计算图像偏导数 (用 Sobel)
    Ix = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    Iy = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    Ixx = cv2.Sobel(Ix, cv2.CV_64F, 1, 0, ksize=3)
    Ixy = cv2.Sobel(Ix, cv2.CV_64F, 0, 1, ksize=3)
    Iyy = cv2.Sobel(Iy, cv2.CV_64F, 0, 1, ksize=3)

    for row in range(1, h - 1):
        for col in range(1, w - 1):
            if gray[row, col] < threshold:
                continue

            # Hessian 矩阵
            H = np.array([[Ixx[row, col], Ixy[row, col]],
                          [Ixy[row, col], Iyy[row, col]]])
            eigvals, eigvecs = np.linalg.eigh(H)

            # 负特征值且绝对值较大 = 条纹法线方向
            if eigvals[0] >= 0:
                continue

            nx, ny = eigvecs[:, 0]  # 法线方向

            # 亚像素偏移
            denom = Ixx[row, col] * nx * nx + 2 * Ixy[row, col] * nx * ny + Iyy[row, col] * ny * ny
            if abs(denom) < 1e-6:
                continue

            t = -(Ix[row, col] * nx + Iy[row, col] * ny) / denom

            if abs(t) <= 0.5:
                px = col + t * nx
                py = row + t * ny
                if 0 <= px < w and 0 <= py < h:
                    points.append([float(px), float(py)])

    return np.array(points, dtype=np.float64)


def draw_laser_line(image: np.ndarray, laser: LaserLine,
                    color: tuple = (0, 255, 0)) -> np.ndarray:
    """在图像上绘制提取到的激光线."""
    vis = image.copy()
    for pt in laser.points.astype(np.int32):
        cv2.circle(vis, tuple(pt), 1, color, -1)
    return vis
