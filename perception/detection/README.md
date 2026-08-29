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

The default model is `yolov8n.onnx` alongside the script.

## ROS 2 bridge

The CSI camera remains on Raspberry Pi OS and ROS 2 remains in Docker. Start
the bridge in the ROS container first:

```bash
cd /ros2_ws
colcon build --packages-select mcr_perception --symlink-install
source install/setup.bash
ros2 launch mcr_perception perception_bridge.launch.py
```

Then start the host detector with a local UDP output:

```bash
PYTHONPATH=/usr/lib/python3/dist-packages /home/pi/yolo-venv/bin/python \
  perception/detection/yolo_detect.py --camera csi --ros-udp-port 12000
```

The bridge republishes the compact JSON packet on `/perception/detections`
(`std_msgs/String`) and reports link health on `/perception/status`. Both the
packet and status contain a `schema` field so consumers can reject incompatible
versions. The UDP listener is bound to `127.0.0.1` by default and is therefore
not exposed to the LAN.

```bash
ros2 topic echo /perception/status
ros2 topic echo --once /perception/detections
```
