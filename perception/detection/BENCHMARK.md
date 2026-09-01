# 感知链路 Benchmark（2026-09-01）

Pi 5 CSI 相机 → Picamera2 → ONNX Runtime → UDP → ROS 2 `mcr_perception` 桥接节点 →
`/perception/detections` + `/perception/status`。链路端到端实测：`ros2 topic echo
/perception/status` 确认 `received_packets` 与 host 端发送帧数一致，`invalid_packets=0`。

## 延迟 / 吞吐 / 资源占用

100 帧，CSI 640×480，confidence=0.15，Raspberry Pi 5（CPU-only, ONNX Runtime CPUExecutionProvider）：

| | YOLOv8n（COCO-80） | YOLO-World（10类开放词表） |
|---|---:|---:|
| 模型大小 | 12.3 MB | 47.8 MB |
| FPS | 5.18 | 1.83 |
| 推理延迟 avg | 157.3 ms | 508.9 ms |
| 推理延迟 P95 | 183.1 ms | 579.6 ms |
| CPU（多核合计占用） | 360% | 491% |
| 峰值常驻内存 | 260.0 MB | 352.5 MB |
| 检出帧数占比 | 90/100 | 95/100 |

**结论**：YOLO-World 在 bench 的 RTX 3060 Ti 上比 YOLOv8n 快（87.5 vs 53.5 FPS，见
edge-ai-lab 记录），但换到 Pi 5 CPU 推理后反而慢 **3.2 倍**（508.9ms vs 157.3ms 平均延迟）。
根因是 YOLO-World 用的 backbone（yolov8s 量级）比 YOLOv8n 重，这个差距在 GPU 算力下被掩盖，
到 CPU 上就成了主导因素。**结论：端侧部署决策不能只看训练/开发机的 GPU benchmark，必须在
真实目标硬件上验证。**

## 发现并修复的 bug

`yolo_detect.py` 原先把 YOLO-World 导出时的 10 类开放词表（`person, chair, table, sofa,
door, box, shoe, trash can, cable, pet`）硬编码为全局 `CLASSES`，但 `decode_predictions`
对任意模型输出通用。换成标准 COCO-80 输出的 `yolov8n.onnx` 后，被验证判定：`class_id`
落在 COCO 索引空间，用错误的 10 类词表取 `label` 会张冠李戴（实测复现：COCO `class_id=9`
是 `traffic light`，被误标成 `pet`）。

修复：按模型文件名（`*world*.onnx` → 开放词表，其余 → 标准 COCO-80）在 `main()` 里选择
正确的类别表，`classes_for_model()`。修复前后用真实摄像头画面（人像）对比验证：
修复前 `person:0.68, pet:0.38`（后者是错标的 traffic light）；修复后
`person:0.75, vase:0.56`（vase 是真实 COCO 标签，误检圆柱状物体是该类模型的常见已知局限，
不是标签错误）。

## 已知问题（未修复）

`draw_detections()`（仅用于本地可视化/demo截图，不影响 `/perception/detections` 发布的
JSON 数据）疑似有色彩通道问题：Picamera2 `format="RGB888"` 配置下 `capture_array()`
返回的内存布局实际是 BGR（Picamera2 已知行为），但代码全程当作真 RGB 处理，`draw_detections`
里再做一次 `COLOR_RGB2BGR` 转换，导致保存的标注图像肤色发蓝。不影响检测精度和
ROS 2 topic 数据本身，只影响可视化输出的色彩正确性，可视化用的 demo 图暂不能直接展示。

## 复现

```bash
cd perception/detection
PYTHONPATH=/usr/lib/python3/dist-packages /home/pi/yolo-venv/bin/python \
  benchmark_detect.py --models yolov8n.onnx yolo_world.onnx \
  --camera csi --frames 100 --confidence 0.15 --output benchmark_report.json
```

完整逐帧数据见同目录 `benchmark_report.json`。
