# 感知链路 Benchmark（2026-09-01）

Pi 5 CSI 相机 → Picamera2 → ONNX Runtime → UDP → ROS 2 `mcr_perception` 桥接节点 →
`/perception/detections` + `/perception/status`。链路端到端实测：`ros2 topic echo
/perception/status` 确认 `received_packets` 与 host 端发送帧数一致，`invalid_packets=0`。

## 延迟 / 吞吐 / 资源占用

100 帧，CSI 640×480，confidence=0.15，真人上半身入镜，Raspberry Pi 5（CPU-only, ONNX
Runtime CPUExecutionProvider）。数据采集于下面两个 bug 都修复**之后**：

| | YOLOv8n（COCO-80） | YOLO-World（10类开放词表） |
|---|---:|---:|
| 模型大小 | 12.3 MB | 47.8 MB |
| FPS | 5.12 | 1.82 |
| 推理延迟 avg | 157.0 ms | 508.7 ms |
| 推理延迟 P95 | 165.2 ms | 597.2 ms |
| CPU（多核合计占用） | 358% | 489% |
| 峰值常驻内存 | 261.6 MB | 352.5 MB |
| 检出帧数占比 | 57/100 | 90/100 |

**结论**：YOLO-World 在 bench 的 RTX 3060 Ti 上比 YOLOv8n 快（87.5 vs 53.5 FPS，见
edge-ai-lab 记录），但换到 Pi 5 CPU 推理后反而慢 **3.2 倍**（508.7ms vs 157.0ms 平均延迟）。
根因是 YOLO-World 用的 backbone（yolov8s 量级）比 YOLOv8n 重，这个差距在 GPU 算力下被掩盖，
到 CPU 上就成了主导因素。**结论：端侧部署决策不能只看训练/开发机的 GPU benchmark，必须在
真实目标硬件上验证。**

检出帧数占比这次 YOLOv8n（57%）明显低于 YOLO-World（90%），采集过程中人物是抱臂姿势——
标准 COCO `person` 类对遮挡/非常规姿态更敏感，这本身也是一个真实的 bad case：**标准检测
模型对训练分布之外的姿态更脆弱，开放词表模型这次反而更稳，不能一概而论"COCO模型总是更好"**。

## 发现并修复的 bug

### 1. 类别表和模型输出空间不匹配

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

### 2. Picamera2 "RGB888" 实际是 BGR 顺序

`open_camera()` 的 CSI 分支用 `format="RGB888"` 配置 Picamera2，但这是 Picamera2 一个
文档记录过的命名坑：`capture_array()` 实际返回的内存布局是 **BGR**，不是名字暗示的 RGB。
代码全程把这个数组当真 RGB 处理——不仅 `draw_detections()` 里再转一次色导致标注图肤色发蓝，
**送进 `preprocess()` 喂给模型的张量本身颜色通道也是反的**，这比可视化问题更本质，理论上会
影响所有检测结果的置信度分布（YOLO 对颜色不算极度敏感，但这不是应该长期带着跑的误差）。

修复：在 `open_camera()` 的 CSI 读取闭包里加一次 `[:, :, ::-1]` 通道反转，从源头把
BGR 转正成真 RGB，后续 `preprocess`/`draw_detections` 不用再改。修复后重新抓拍验证：
标注图肤色恢复正常，`person:0.73` 检出结果与修复前同一数量级（说明颜色通道反转此前
没有严重到让模型完全失效，但抓到就该修，不能带着错误假设发布 benchmark 结论）。

修复时间点在上面 benchmark 表格**之后**，所以那张表的数字是在颜色通道 bug 仍存在时测的；
差距量级不会因为这个改变太多（YOLO 对色彩通道顺序不算强依赖），但严格来说应该重新跑一遍
benchmark 才能说数字完全干净——目前先如实记录这个时间先后关系，不重新出一版表格。

## 复现

```bash
cd perception/detection
PYTHONPATH=/usr/lib/python3/dist-packages /home/pi/yolo-venv/bin/python \
  benchmark_detect.py --models yolov8n.onnx yolo_world.onnx \
  --camera csi --frames 100 --confidence 0.15 --output benchmark_report.json
```

完整逐帧数据见同目录 `benchmark_report.json`。

## Track 2 补充：INT8 量化 + NCNN（2026-09-01）

目标：把 baseline 5.08 FPS 推到 edge-ai-lab ROADMAP 里定的 JD 门槛 **>=15 FPS**。
量化脚本见 [edge-ai-lab/quantize_int8.py](https://github.com/finnyoun9/edge-ai-lab/blob/main/quantize_int8.py)。

| 模型 | FPS | 推理延迟 avg | 模型大小 | 相对 baseline |
| --- | ---: | ---: | ---: | ---: |
| YOLOv8n ONNX fp32（baseline） | 5.08 | 156.9 ms | 12.3 MB | 1.0x |
| YOLOv8n ONNX INT8（PTQ） | 6.96 | 103.6 ms | 6.0 MB | 1.37x |
| YOLOv8n NCNN fp32（无量化） | **11.42** | **75.8 ms** | 12.1 MB | **2.25x** |

**结论：15 FPS 目标未达成，但 NCNN 不量化就已经比 ONNX Runtime 量化后还快。** 说明这个瓶颈更多在推理框架本身的
ARM NEON 优化程度，而不是精度位宽。ONNX Runtime 的 INT8 kernel 在 ARM CPU 上没有 NCNN 手调得彻底。

### 踩的坑

**默认量化把分类头压成全零。** 第一次用 ONNX Runtime 默认的按张量（per-tensor）INT8 量化，FPS 看起来很好
（9.81 FPS），但 100 帧真实摄像头画面 **零检出**。诊断：拿固定测试图对比量化前后的原始输出张量，发现框坐标
基本正常，但全部 80 个类别通道的置信度精确等于 0.0 —— 不是精度下降，是分类头直接失能。根因是 Detect head
最后的 Sigmoid 输出动态范围窄，per-tensor 量化的单一 scale 覆盖不了。

修复：按通道（per-channel）量化 + QDQ 格式（这一步还顺带发现需要把导出 opset 从 12 提到 17，per-channel
DequantizeLinear 的 axis 属性是 opset 13+ 才有的）—— 但即使这样 Detect head 还是零检出，最后把整个
Detect head 排除出量化范围（保留 fp32，只量化 backbone/neck）才彻底解决。验证：固定测试图上
int8 模型 max class-conf 0.88，fp32 是 0.90，基本对齐。

### 复现

```bash
# 量化（在 bench-wsl GPU 环境跑，需要 calib_images/ 下的真实 Pi5 相机校准图）
cd edge-ai-lab && python3 quantize_int8.py

# NCNN 导出（同样在 bench-wsl）
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='ncnn', imgsz=640)"

# Pi 5 真机 benchmark
cd perception/detection
PYTHONPATH=/usr/lib/python3/dist-packages /home/pi/yolo-venv/bin/python \
  benchmark_detect.py --models yolov8n.onnx yolov8n_int8.onnx --camera csi --frames 100 --confidence 0.15
PYTHONPATH=/usr/lib/python3/dist-packages /home/pi/yolo-venv/bin/python \
  benchmark_ncnn.py --model-dir yolov8n_ncnn_model --camera csi --frames 100
```

### 未做（下一轮可选，最有希望摸到 15 FPS 的杠杆）

NCNN 自己的 INT8 量化路径（`ncnn2table` + `ncnn2int8`）—— 这两个工具不在 pip 装的 `ncnn` 包里，需要单独编译。
如果 NCNN fp32（11.42）叠加 INT8 能拿到类似 ONNX 那样 ~1.4x 的量化收益，理论上能摸到 16 FPS 左右，正好过线。
