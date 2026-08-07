# Phase C2: 线激光三角测量 — 3D 扫描

## 硬件需求

- USB 摄像头（任何分辨率，640×480 即可）
- 650nm 一字线激光模组 USB（~15-50 元）
- 棋盘格标定板（打印 A4 纸，9×6 格，25mm 格距）

## 文件结构

```
c2_laser_triangulation/
├── README.md
├── __init__.py
├── laser_extractor.py          # 激光线中心提取（3 种方法）
├── light_plane_calib.py        # 光平面标定
├── triangulation.py            # 像素 → 3D 三角测量
├── point_cloud_scanner.py      # 实时扫描 + 点云可视化
├── live_preview.py             # 实时激光线预览
├── test_laser_extractor.py     # 单元测试
├── test_triangulation.py       # 单元测试
└── calibration_data/           # 标定数据存储
```

## 使用流程

### 1. 环境准备

```bash
cd robot-vacuum-lab
source venv/bin/activate
```

### 2. 激光线预览

对准激光和摄像头，打开预览调整参数：

```bash
cd code/perception/c2_laser_triangulation
python live_preview.py
```

按 `[` `]` 调阈值，按 `1` `2` `3` 切换提取算法。

### 3. 光平面标定

1. 准备标定板（打印棋盘格，贴平整）
2. 让激光线投射在标定板上
3. 拍 5-10 张不同角度的照片，放入 `calibration_data/`
4. 运行标定程序：

```python
import numpy as np
from light_plane_calib import calibrate_light_plane, save_calibration

# 加载相机内参（从 Phase C1 标定结果）
calib = np.load('../calib_result.npz')
K = calib['mtx']
dist = calib['dist']

# 标定光平面
images = ['calibration_data/calib_01.jpg', 'calibration_data/calib_02.jpg', ...]
plane = calibrate_light_plane(images, K, dist)
save_calibration(plane, 'calibration_data/light_plane.json')
```

### 4. 扫描

```bash
python point_cloud_scanner.py --calib calibration_data/light_plane.json
```

- **空格** — 采集一帧
- 缓慢移动物体穿过激光线
- **V** — 打开 Open3D 3D 点云查看
- **S** — 保存为 PLY 文件
- **Q** — 退出

## 原理

```
        摄像头
         /
        /  激光线投射到物体表面
       /
      /_____ 激光发射器
     /
    /  物体（杯子/手机）
   /
桌面
```

- **灰度重心法**：对每一列像素，以灰度值为权重计算行方向质心
- **光平面标定**：棋盘格 PnP 求外参 → 激光像素反投影到棋盘格平面 → SVD 拟合光平面
- **三角测量**：光线方向 = K⁻¹ × 像素，光平面 aX+bY+cZ+d=0，求交点

## 验证标准

> **扫描手机/杯子，3D 轮廓可见，高度误差 < 1mm（30cm 近场）**

## 运行测试

```bash
python test_laser_extractor.py
python test_triangulation.py
```
