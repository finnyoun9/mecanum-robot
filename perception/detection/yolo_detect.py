"""
Phase 2 — YOLO 目标检测（onnxruntime）

首次运行自动下载 yolov8n.onnx（~6MB），之后直接用摄像头实时推理。
支持 80 类 COCO 物体。
"""
import cv2
import numpy as np
import onnxruntime as ort
from pathlib import Path
import subprocess, sys

MODEL_PATH = Path("code/yolov8n.onnx")
CLASSES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush",
]

# 下载模型
if not MODEL_PATH.exists():
    print("下载 yolov8n.onnx（~6MB）...")
    url = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.onnx"
    r = subprocess.run(
        ["curl", "-L", "-k", "--connect-timeout", "30", "--max-time", "120",
         "-o", str(MODEL_PATH), url],
        capture_output=True, text=True)
    if r.returncode != 0 or MODEL_PATH.stat().st_size < 100000:
        print("下载失败，请手动下载 yolov8n.onnx 放到 code/ 目录下")
        sys.exit(1)
    print("下载完成")

# 加载 onnxruntime
print("加载模型...")
session = ort.InferenceSession(str(MODEL_PATH), providers=["CPUExecutionProvider"])
input_name = session.get_inputs()[0].name
print("模型就绪，按 q 退出\n")

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")
    sys.exit(1)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    h, w = frame.shape[:2]

    # 预处理：resize 到 640x640，归一化 [0,1]
    img = cv2.resize(frame, (640, 640))
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))  # HWC → CHW
    img = np.expand_dims(img, 0)        # (1, 3, 640, 640)

    # 推理
    outputs = session.run(None, {input_name: img})[0]  # (1, 84, 8400)
    outputs = outputs[0].T  # (8400, 84)

    # 解析输出
    boxes_list, scores_list, class_ids = [], [], []
    for row in outputs:
        cx, cy, bw_val, bh_val = row[:4]
        probs = row[4:]
        class_id = int(np.argmax(probs))
        conf = float(probs[class_id])
        if conf < 0.5:
            continue

        x1 = int((cx - bw_val / 2) * w / 640)
        y1 = int((cy - bh_val / 2) * h / 640)
        x2 = int((cx + bw_val / 2) * w / 640)
        y2 = int((cy + bh_val / 2) * h / 640)

        boxes_list.append([x1, y1, x2 - x1, y2 - y1])
        scores_list.append(conf)
        class_ids.append(class_id)

    # NMS 去重
    indices = cv2.dnn.NMSBoxes(boxes_list, scores_list, 0.4, 0.5)

    for i in indices:
        x1, y1, bw_val, bh_val = boxes_list[i]
        x2, y2 = x1 + bw_val, y1 + bh_val
        label = f"{CLASSES[class_ids[i]]} {scores_list[i]:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.putText(frame, f"Detected: {len(indices)} objects", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("Phase 2 - YOLO Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
