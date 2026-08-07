"""实时摄像头预览 + 激光线提取叠加."""
import cv2
import sys
from laser_extractor import extract_laser_line, draw_laser_line, Method


def main(camera_id: int = 0):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"Cannot open camera {camera_id}")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    method = Method.CENTROID
    threshold = 80

    print("Controls: q=quit, 1=centroid, 2=gaussian, 3=steger, [/]=threshold±")
    print(f"Method: {method.name}, threshold: {threshold}")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        try:
            laser = extract_laser_line(frame, method=method, threshold=threshold)
            vis = draw_laser_line(frame, laser)
            cv2.putText(vis, f"Method: {method.name} | Thresh: {threshold} | Points: {len(laser.points)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        except Exception as e:
            vis = frame.copy()
            cv2.putText(vis, f"Error: {e}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)

        cv2.imshow("Laser Line Preview", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('1'):
            method = Method.CENTROID
            print(f"  → Method: CENTROID")
        elif key == ord('2'):
            method = Method.GAUSSIAN
            print(f"  → Method: GAUSSIAN")
        elif key == ord('3'):
            method = Method.STEGER
            print(f"  → Method: STEGER")
        elif key == ord('['):
            threshold = max(10, threshold - 10)
            print(f"  → threshold: {threshold}")
        elif key == ord(']'):
            threshold = min(255, threshold + 10)
            print(f"  → threshold: {threshold}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Live laser line preview")
    parser.add_argument("--camera-id", type=int, default=0)
    args = parser.parse_args()
    main(args.camera_id)
