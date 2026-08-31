#!/usr/bin/env python3
"""Headless-friendly YOLOv8 ONNX detection for IMX219 CSI or USB cameras."""
import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import socket
import time
from typing import Iterable

import numpy as np


MODEL_PATH = Path(__file__).with_name("yolo_world.onnx")
INPUT_SIZE = 640
# Open-vocabulary set baked into yolo_world.onnx at export time (see
# edge-ai-lab/benchmark_yolo_world.py). Order must match the classes passed
# to model.set_classes() during export.
CLASSES = [
    "person", "chair", "table", "sofa", "door",
    "box", "shoe", "trash can", "cable", "pet",
]


@dataclass(frozen=True)
class Detection:
    class_id: int
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def label(self) -> str:
        return CLASSES[self.class_id]


def detection_message(frame_id: int, frame_width: int, frame_height: int,
                      detections: list[Detection]) -> bytes:
    """Return the versioned, transport-neutral detection packet.

    This intentionally uses only the Python standard library: the CSI camera
    stays on Raspberry Pi OS while ROS 2 runs in the project's Docker image.
    The ROS bridge republishes this packet as a ROS topic on the same host.
    """
    message = {
        "schema": "mcr.perception.detections.v1",
        "frame_id": frame_id,
        "stamp_ns": time.time_ns(),
        "width": frame_width,
        "height": frame_height,
        "detections": [
            {
                "class_id": item.class_id,
                "label": item.label,
                "confidence": round(item.confidence, 4),
                "bbox": {"x1": item.x1, "y1": item.y1,
                         "x2": item.x2, "y2": item.y2},
            }
            for item in detections
        ],
    }
    return json.dumps(message, ensure_ascii=True, separators=(",", ":")).encode("utf-8")


def preprocess(frame_rgb: np.ndarray) -> np.ndarray:
    """Resize RGB image to YOLO input and produce NCHW float32 tensor."""
    if frame_rgb.ndim != 3 or frame_rgb.shape[2] < 3:
        raise ValueError("camera frame must contain at least three colour channels")
    image = frame_rgb[:, :, :3]
    if image.shape[:2] != (INPUT_SIZE, INPUT_SIZE):
        try:
            import cv2
            image = cv2.resize(image, (INPUT_SIZE, INPUT_SIZE))
        except ImportError:
            # Keeps decode/preprocess tests runnable off-target; Pi uses OpenCV above.
            row = np.linspace(0, image.shape[0] - 1, INPUT_SIZE).astype(np.intp)
            column = np.linspace(0, image.shape[1] - 1, INPUT_SIZE).astype(np.intp)
            image = image[row][:, column]
    return np.ascontiguousarray(image.transpose(2, 0, 1), dtype=np.float32)[None] / 255.0


def _iou(box: Detection, other: Detection) -> float:
    left, top = max(box.x1, other.x1), max(box.y1, other.y1)
    right, bottom = min(box.x2, other.x2), min(box.y2, other.y2)
    intersection = max(0, right - left) * max(0, bottom - top)
    union = ((box.x2 - box.x1) * (box.y2 - box.y1) + (other.x2 - other.x1) * (other.y2 - other.y1) - intersection)
    return intersection / union if union else 0.0


def _nms(candidates: Iterable[Detection], threshold: float) -> list[Detection]:
    """Class-aware greedy NMS, independent of OpenCV for testability."""
    kept: list[Detection] = []
    for candidate in sorted(candidates, key=lambda item: item.confidence, reverse=True):
        if all(candidate.class_id != chosen.class_id or _iou(candidate, chosen) <= threshold for chosen in kept):
            kept.append(candidate)
    return kept


def decode_predictions(output: np.ndarray, frame_width: int, frame_height: int, confidence_threshold: float, nms_threshold: float) -> list[Detection]:
    """Convert YOLOv8 (1, 84, 8400) output to original-frame detections."""
    if output.ndim != 3 or output.shape[0] != 1 or output.shape[1] < 5:
        raise ValueError(f"unexpected YOLO output shape: {output.shape}")
    candidates: list[Detection] = []
    for row in output[0].T:
        class_id = int(np.argmax(row[4:]))
        score = float(row[4 + class_id])
        if score < confidence_threshold:
            continue
        center_x, center_y, width, height = row[:4]
        x1 = int((center_x - width / 2) * frame_width / INPUT_SIZE)
        y1 = int((center_y - height / 2) * frame_height / INPUT_SIZE)
        x2 = int((center_x + width / 2) * frame_width / INPUT_SIZE)
        y2 = int((center_y + height / 2) * frame_height / INPUT_SIZE)
        candidates.append(Detection(class_id, score, max(0, x1), max(0, y1), min(frame_width - 1, x2), min(frame_height - 1, y2)))
    return _nms(candidates, nms_threshold)


def open_camera(kind: str, width: int, height: int, device: int):
    """Return (read_rgb, close) for a CSI IMX219 or a conventional USB camera."""
    if kind == "csi":
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError("CSI mode needs Picamera2; install python3-picamera2 on the Pi host") from error
        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (width, height), "format": "RGB888"}))
        camera.start()
        return lambda: camera.capture_array("main")[:, :, :3], camera.close

    import cv2
    camera = cv2.VideoCapture(device)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"cannot open USB camera device {device}")

    def read_usb_rgb() -> np.ndarray:
        ok, frame_bgr = camera.read()
        if not ok:
            raise RuntimeError("USB camera failed to return a frame")
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    return read_usb_rgb, camera.release


def draw_detections(frame_rgb: np.ndarray, detections: list[Detection]) -> np.ndarray:
    """Create an annotated BGR frame for optional local desktop display."""
    import cv2
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    for item in detections:
        cv2.rectangle(frame_bgr, (item.x1, item.y1), (item.x2, item.y2), (0, 255, 0), 2)
        cv2.putText(frame_bgr, f"{item.label} {item.confidence:.2f}", (item.x1, max(16, item.y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    return frame_bgr


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", choices=("csi", "usb"), default="csi")
    parser.add_argument("--device", type=int, default=0, help="USB VideoCapture device index")
    parser.add_argument("--model", type=Path, default=MODEL_PATH)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--confidence", type=float, default=0.50)
    parser.add_argument("--nms", type=float, default=0.50)
    parser.add_argument("--display", action="store_true", help="show an OpenCV window (desktop only)")
    parser.add_argument("--max-frames", type=int, default=0, help="0 runs until Ctrl-C; useful for smoke tests")
    parser.add_argument("--print-every", type=int, default=10, help="headless status interval in frames")
    parser.add_argument("--ros-udp-port", type=int, default=0,
                        help="send result JSON to the local ROS bridge; 0 disables it")
    parser.add_argument("--ros-udp-host", default="127.0.0.1",
                        help="ROS bridge host; default stays local to the Pi")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.model.is_file():
        raise SystemExit(f"model not found: {args.model}; place yolov8n.onnx in perception/detection/")
    if (args.width <= 0 or args.height <= 0 or args.max_frames < 0
            or args.print_every <= 0 or not 0 <= args.ros_udp_port <= 65535):
        raise SystemExit("width, height and print-every must be positive; max-frames must be non-negative")
    try:
        import onnxruntime as ort
    except ImportError as error:
        raise SystemExit("onnxruntime is required: pip install onnxruntime") from error
    if args.display:
        import cv2

    session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    read_frame, close_camera = open_camera(args.camera, args.width, args.height, args.device)
    print(f"YOLO ready: camera={args.camera} {args.width}x{args.height}, headless={not args.display}")
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.ros_udp_port else None
    if udp_socket:
        print(f"ROS bridge output: udp://{args.ros_udp_host}:{args.ros_udp_port}")
    frame_count, started = 0, time.monotonic()
    try:
        while args.max_frames == 0 or frame_count < args.max_frames:
            frame_rgb = read_frame()
            detections = decode_predictions(session.run(None, {input_name: preprocess(frame_rgb)})[0], frame_rgb.shape[1], frame_rgb.shape[0], args.confidence, args.nms)
            frame_count += 1
            if udp_socket:
                udp_socket.sendto(detection_message(frame_count, frame_rgb.shape[1], frame_rgb.shape[0], detections),
                                  (args.ros_udp_host, args.ros_udp_port))
            if frame_count % args.print_every == 0 or frame_count == 1:
                labels = ", ".join(f"{item.label}:{item.confidence:.2f}" for item in detections[:4]) or "none"
                print(f"frame={frame_count} detections={len(detections)} [{labels}]")
            if args.display:
                cv2.imshow("Mecanum Robot YOLO", draw_detections(frame_rgb, detections))
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        close_camera()
        if udp_socket:
            udp_socket.close()
        if args.display:
            cv2.destroyAllWindows()
    elapsed = max(time.monotonic() - started, 1e-9)
    print(f"stopped: frames={frame_count} fps={frame_count / elapsed:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
