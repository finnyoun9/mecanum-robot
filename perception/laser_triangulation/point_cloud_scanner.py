"""实时扫描：逐帧采集 → 三角测量 → 累积点云 → Open3D 可视化."""
import cv2
import numpy as np
import open3d as o3d
import json
from pathlib import Path
from laser_extractor import extract_laser_line, Method
from triangulation import triangulate
from light_plane_calib import load_calibration


class PointCloudScanner:
    def __init__(self,
                 camera_matrix: np.ndarray,
                 light_plane: np.ndarray,
                 camera_id: int = 0,
                 laser_threshold: int = 80,
                 method: Method = Method.CENTROID):
        self.K = camera_matrix
        self.plane = light_plane
        self.threshold = laser_threshold
        self.method = method

        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.accumulated_cloud: list[np.ndarray] = []
        self.total_frames = 0
        self.total_points = 0

    def scan_one_frame(self) -> tuple[np.ndarray | None, np.ndarray]:
        """采集一帧，返回 (3D点云, 可视化帧)."""
        ret, frame = self.cap.read()
        if not ret:
            return None, np.zeros((480, 640, 3), dtype=np.uint8)

        laser = extract_laser_line(frame, method=self.method, threshold=self.threshold)

        if len(laser.points) < 5:
            vis = frame.copy()
            cv2.putText(vis, "No laser detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return None, vis

        pts_3d = triangulate(laser, self.K, self.plane)
        self.accumulated_cloud.append(pts_3d)
        self.total_frames += 1
        self.total_points += len(pts_3d)

        # 可视化
        vis = frame.copy()
        for pt in laser.points.astype(np.int32):
            cv2.circle(vis, tuple(pt), 1, (0, 255, 0), -1)
        cv2.putText(vis, f"Frames: {self.total_frames} | Points: {self.total_points}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        return pts_3d, vis

    def get_full_cloud(self) -> np.ndarray:
        """返回累积的全部点云."""
        if not self.accumulated_cloud:
            return np.zeros((0, 3))
        return np.vstack(self.accumulated_cloud)

    def visualize(self):
        """Open3D 可视化累积点云."""
        cloud = self.get_full_cloud()
        if len(cloud) == 0:
            print("No points to visualize")
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)

        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50)

        o3d.visualization.draw_geometries([pcd, coord],
                                           window_name="Laser Scan Point Cloud",
                                           width=1024, height=768)

    def save_cloud(self, path: str):
        """保存点云为 PLY 文件."""
        cloud = self.get_full_cloud()
        if len(cloud) == 0:
            print("No points to save")
            return

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)
        o3d.utility.write_point_cloud(path, pcd)
        print(f"Saved {len(cloud)} points to {path}")

    def close(self):
        self.cap.release()


def main():
    """交互式扫描主程序."""
    import argparse
    parser = argparse.ArgumentParser(description="Laser triangulation point cloud scanner")
    parser.add_argument("--calib", required=True, help="Path to light plane calibration JSON")
    parser.add_argument("--camera-matrix", help="Camera matrix JSON (default: use calib_result.npz)")
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--threshold", type=int, default=80)
    parser.add_argument("--output", default="scan.ply", help="Output PLY file")
    args = parser.parse_args()

    # 加载光平面
    plane = load_calibration(args.calib)

    # 加载相机内参
    if args.camera_matrix:
        with open(args.camera_matrix) as f:
            K = np.array(json.load(f)["camera_matrix"])
    else:
        calib_path = Path(__file__).resolve().parent.parent / "calib_result.npz"
        if calib_path.exists():
            calib = np.load(str(calib_path))
            K = calib["mtx"]
        else:
            # Fallback: use default intrinsics
            print("WARNING: No camera matrix found, using default (fx=fy=600, cx=320, cy=240)")
            K = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)

    scanner = PointCloudScanner(K, plane,
                                 camera_id=args.camera_id,
                                 laser_threshold=args.threshold)

    print("Scanning... Move the object slowly through the laser line.")
    print("  SPACE = capture frame")
    print("  V     = open 3D viewer")
    print("  S     = save PLY file")
    print("  [/]   = threshold ±10")
    print("  Q     = quit")
    print(f"Frames: 0 | Points: 0")

    while True:
        ret, frame = scanner.cap.read()
        if not ret:
            break

        vis = frame.copy()
        cv2.putText(vis, f"Frames: {scanner.total_frames} | Points: {scanner.total_points}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(vis, "SPACE=scan V=view S=save Q=quit",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("Scanner", vis)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):
            pts_3d, _ = scanner.scan_one_frame()
            if pts_3d is not None:
                print(f"  Frame {scanner.total_frames}: {len(pts_3d)} points")
        elif key == ord('v'):
            print("Opening 3D viewer...")
            scanner.visualize()
        elif key == ord('s'):
            scanner.save_cloud(args.output)
        elif key == ord('['):
            scanner.threshold = max(10, scanner.threshold - 10)
            print(f"  threshold: {scanner.threshold}")
        elif key == ord(']'):
            scanner.threshold = min(255, scanner.threshold + 10)
            print(f"  threshold: {scanner.threshold}")
        elif key == ord('q'):
            break

    scanner.close()
    cv2.destroyAllWindows()
    print(f"Done. Total: {scanner.total_frames} frames, {scanner.total_points} points")


if __name__ == "__main__":
    main()
