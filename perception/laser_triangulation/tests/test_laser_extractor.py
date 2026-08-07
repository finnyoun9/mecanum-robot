"""测试激光线提取 — 合成激光条纹图像."""
import cv2
import numpy as np
from laser_extractor import extract_laser_line, draw_laser_line, Method


def make_synthetic_laser(h: int = 480, w: int = 640) -> np.ndarray:
    """生成含高斯激光条纹的合成图像."""
    img = np.zeros((h, w), dtype=np.uint8)

    # 模拟弯折的激光线: y = h/2 + 50*sin(x/100)
    for col in range(w):
        center = h // 2 + int(50 * np.sin(col / 100))
        for row in range(max(0, center - 10), min(h, center + 10)):
            intensity = int(255 * np.exp(-(row - center)**2 / (2 * 3.0**2)))
            img[row, col] = np.clip(intensity, 0, 255)

    return img


def test_centroid_on_synthetic():
    """灰度重心法在合成图像上应提取到接近真实的激光线."""
    img = make_synthetic_laser()
    laser = extract_laser_line(img, method=Method.CENTROID, threshold=30)

    assert len(laser.points) > 100, f"Expected >100 points, got {len(laser.points)}"
    # 中间列的行坐标应接近 240
    mid_col = 320
    mid_points = laser.points[np.abs(laser.points[:, 0] - mid_col) < 1]
    if len(mid_points) > 0:
        mid_row = mid_points[0, 1]
        assert abs(mid_row - 240) < 10, f"Expected ~240, got {mid_row}"

    print(f"  ✓ centroid synthetic test passed ({len(laser.points)} points)")


def test_all_methods():
    """三种方法都能在合成图像上提取到点."""
    img = make_synthetic_laser()
    for method in [Method.CENTROID, Method.GAUSSIAN, Method.STEGER]:
        laser = extract_laser_line(img, method=method, threshold=30)
        assert len(laser.points) > 50, f"{method.name}: only {len(laser.points)} points"
        print(f"  ✓ {method.name}: {len(laser.points)} points")


def test_draw_laser_line():
    """draw_laser_line 应返回与输入同尺寸的彩色图."""
    img = make_synthetic_laser()
    laser = extract_laser_line(img, method=Method.CENTROID)
    vis = draw_laser_line(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), laser)
    assert vis.shape == (480, 640, 3)
    print("  ✓ draw_laser_line test passed")


if __name__ == "__main__":
    print("Testing laser_extractor...")
    test_centroid_on_synthetic()
    test_all_methods()
    test_draw_laser_line()
    print("All tests passed ✓")
