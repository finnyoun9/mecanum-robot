"""
Phase 1 — 相机标定

把打印好的棋盘格对着摄像头，缓慢旋转/倾斜不同角度。
每成功检测到棋盘格会自动截图（左上角计数），攒够 15 张后自动开始标定。
按 q 退出，按 s 强制截图。
"""
import cv2
import numpy as np
import glob
import os

PATTERN = (9, 6)   # OpenCV 标准棋盘格 9x6 内角点
SAVE_DIR = "code/calib_images"
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")
    exit(1)

print(f"棋盘格内角点: {PATTERN[0]}x{PATTERN[1]}")
print("把棋盘格对着摄像头，变换角度和距离")
print("检测到棋盘格时会自动截图（左上角计数），攒够 15 张自动开始标定")
print("按 s 强制截图，按 q 退出\n")

count = 0
while count < 15:
    ok, frame = cap.read()
    if not ok:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, PATTERN, None)

    display = frame.copy()
    if found:
        # 亚像素精细化
        corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(display, PATTERN, corners_sub, found)

        # 每 30 帧自动截一张（避免重复）
        if count == 0 or (cv2.getTickCount() / cv2.getTickFrequency()) - last_save > 1.5:
            count += 1
            path = f"{SAVE_DIR}/calib_{count:02d}.jpg"
            cv2.imwrite(path, frame)
            last_save = cv2.getTickCount() / cv2.getTickFrequency()
            print(f"已保存 {count}/15: {path}")

    cv2.putText(display, f"Saved: {count}/15", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0) if found else (0, 0, 255), 2)
    cv2.putText(display, "Press q to quit, s to save manually", (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.imshow("Camera Calibration - Show checkerboard", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        cap.release()
        cv2.destroyAllWindows()
        exit(0)
    elif key == ord("s") and found:
        count += 1
        path = f"{SAVE_DIR}/calib_{count:02d}.jpg"
        cv2.imwrite(path, frame)
        last_save = cv2.getTickCount() / cv2.getTickFrequency()
        print(f"已保存 {count}/15: {path}")

cap.release()
cv2.destroyAllWindows()

# ===== 标定 =====
print("\n开始标定...")
objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2)
# 假设每个格子 25mm
objp *= 25

objpoints, imgpoints = [], []
images = sorted(glob.glob(f"{SAVE_DIR}/calib_*.jpg"))
print(f"读取 {len(images)} 张图片")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, PATTERN, None)
    if found:
        corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        objpoints.append(objp)
        imgpoints.append(corners_sub)
        print(f"  {fname}: 检测成功")
    else:
        print(f"  {fname}: 检测失败，跳过")

if len(objpoints) < 10:
    print(f"有效图片不足 ({len(objpoints)} 张)，至少需要 10 张。请重新运行。")
    exit(1)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print(f"\n标定完成！重投影误差: {ret:.4f}（越小越好，<0.5 为优秀）")
print(f"内参矩阵 K:\n{mtx}")
print(f"畸变系数 [k1,k2,p1,p2,k3]:\n{dist}")

# ===== 实时畸变矫正预览 =====
print("\n切换到畸变矫正预览，按 q 退出")
cap = cv2.VideoCapture(0)
while True:
    ok, frame = cap.read()
    if not ok:
        break

    h, w = frame.shape[:2]
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    undistorted = cv2.undistort(frame, mtx, dist, None, new_mtx)

    # 并排显示：原始 vs 矫正
    side_by_side = np.hstack([frame, undistorted])
    cv2.putText(side_by_side, "Original", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(side_by_side, "Undistorted", (w + 20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("Distortion Correction", side_by_side)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

# 保存标定结果
np.savez("code/calib_result.npz", mtx=mtx, dist=dist)
print("标定结果已保存到 code/calib_result.npz")
