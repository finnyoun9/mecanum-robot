"""测试三角测量 — 用已知几何验证."""
import numpy as np
from triangulation import triangulate
from laser_extractor import LaserLine, Method


def test_triangulation_known_plane():
    """光平面 Z=100 时，像素 (320,240) 应反算出 (0, 0, 100)."""
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
    plane = np.array([0.0, 0.0, 1.0, -100.0])

    points = np.array([[320.0, 240.0]], dtype=np.float64)
    laser = LaserLine(points=points, method=Method.CENTROID, image_shape=(480, 640))

    pts_3d = triangulate(laser, K, plane)
    assert len(pts_3d) == 1, f"Expected 1 point, got {len(pts_3d)}"

    expected = np.array([0.0, 0.0, 100.0])
    error = np.linalg.norm(pts_3d[0] - expected)
    assert error < 0.01, f"Expected {expected}, got {pts_3d[0]}, error={error}"
    print(f"  ✓ known plane test passed (error={error:.6f})")


def test_triangulation_off_center():
    """偏离中心的像素应反算出正确的 3D 坐标."""
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
    plane = np.array([0.0, 0.0, 1.0, -200.0])

    # 像素 (420, 240): 右偏 100px → X ≈ 100*200/600 = 33.33mm
    points = np.array([[420.0, 240.0]], dtype=np.float64)
    laser = LaserLine(points=points, method=Method.CENTROID, image_shape=(480, 640))

    pts_3d = triangulate(laser, K, plane)
    assert len(pts_3d) == 1

    assert 32 < pts_3d[0, 0] < 34, f"X should be ~33.33, got {pts_3d[0, 0]}"
    assert abs(pts_3d[0, 2] - 200) < 0.1, f"Z should be 200, got {pts_3d[0, 2]}"
    print(f"  ✓ off-center test passed (3D: {pts_3d[0]})")


def test_triangulation_multiple_points():
    """多条光线应全部正确反算."""
    K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
    plane = np.array([0.0, 0.0, 1.0, -150.0])

    # 一横排像素，应该反算出 X 变化、Z 恒定 = 150
    cols = np.arange(100, 501, 50, dtype=np.float64)
    rows = np.full_like(cols, 240.0)
    points = np.column_stack([cols, rows])
    laser = LaserLine(points=points, method=Method.CENTROID, image_shape=(480, 640))

    pts_3d = triangulate(laser, K, plane)
    assert len(pts_3d) == len(points)

    # 所有点 Z 应接近 150
    assert np.allclose(pts_3d[:, 2], 150.0, atol=0.1)
    # X 应该从左到右递增
    assert np.all(np.diff(pts_3d[:, 0]) > 0)
    print(f"  ✓ multiple points test passed ({len(pts_3d)} points, Z range: {pts_3d[:,2].min():.1f}-{pts_3d[:,2].max():.1f})")


if __name__ == "__main__":
    print("Testing triangulation...")
    test_triangulation_known_plane()
    test_triangulation_off_center()
    test_triangulation_multiple_points()
    print("All tests passed ✓")
