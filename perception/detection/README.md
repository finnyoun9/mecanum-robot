# IMX219 / YOLOv8n detection

This is the standalone M5 visual-perception entry point. It supports the
Raspberry Pi IMX219 CSI camera through Picamera2 and a USB camera through
OpenCV. It is designed to run headless; an OpenCV window is opt-in.

On the Raspberry Pi host (not inside Docker), install Picamera2 and ONNX Runtime:

```bash
sudo apt install python3-picamera2 python3-opencv
pip install onnxruntime
python3 perception/detection/yolo_detect.py --camera csi --max-frames 20
```

The bounded command is the recommended non-moving hardware smoke test. It
prints detections and a final measured FPS. Run continuously with no
`--max-frames`; stop with `Ctrl-C`.

For a local monitor, add `--display` (press `q` to quit). For a USB webcam:

```bash
python3 perception/detection/yolo_detect.py --camera usb --device 0 --display
```

The default model is `yolov8n.onnx` alongside the script. This tool currently
prints observations only; publishing detections as ROS 2 topics is the next M5
integration step.
