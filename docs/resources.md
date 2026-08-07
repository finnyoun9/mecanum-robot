# 学习资源汇总 — 从零构建扫地机器人

---

## Track A: STM32 嵌入式

### 课程与教程

| 资源 | 说明 |
|------|------|
| [江科大 STM32 入门教程](https://www.bilibili.com/video/BV1th411z7sn/) | GPIO/PWM/编码器/UART/I2C/ADC 全覆盖，用户正在跟学 |
| [江科大平衡车项目](https://www.bilibili.com/video/BV1oD4y1X7qH/) | 平衡车开源项目，MPU6050+PID，用户已有全套硬件 |
| [硬石科技 STM32 教程](https://www.bilibili.com/video/BV1CJ411R7jL/) | HAL 库开发，电机控制部分讲得好 |
| [野火 STM32 教程](https://www.bilibili.com/video/BV1Lx411Z7Qa/) | 经典系列，外设讲解细致 |

### 工具

| 工具 | 用途 |
|------|------|
| Keil MDK | STM32 标准 IDE（Windows） |
| STM32CubeMX | 外设初始化代码生成 |
| 串口调试助手 | UART 通信调试 |
| 逻辑分析仪（~30 元） | 通信协议分析 |

### 参考书

| 书名 | 说明 |
|------|------|
| 《STM32 HAL 库开发实战指南》 | 野火出版，入门首选 |
| 《PID 控制器——从原理到实现》 | 通俗理解 PID 调参 |

---

## Track B: ROS2 + 导航

### ROS2 核心

| 资源 | 说明 |
|------|------|
| [ROS2 Humble 官方文档](https://docs.ros.org/en/humble/Tutorials.html) | 入门必读，Python 节点教程 |
| [ROS2 Control 文档](https://control.ros.org/master/index.html) | 机器人控制框架 |
| [鱼香 ROS 中文教程](https://fishros.org/) | 中文 ROS2 教程，一键安装脚本 |

### SLAM / 导航

| 资源 | 说明 |
|------|------|
| [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | ROS2 上的 2D SLAM |
| [Cartographer](https://google-cartographer-ros.readthedocs.io/) | Google 的方案，效果更好 |
| [Nav2 文档](https://navigation.ros.org/) | ROS2 导航栈 |
| [Cyrill Stachniss SLAM 课程](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) | SLAM 理论，YouTube 上最系统 |
| 《Probabilistic Robotics》(Thrun) | SLAM 和状态估计圣经 |

### Linux 开发板

树莓派 5 目前价格偏高（~650 元），性价比更好的替代方案：

| 方案 | 价格 | 说明 |
|------|------|------|
| Orange Pi 3B (RK3566, 4GB) | ~220 元 | 推荐，Ubuntu + ROS2 够用 |
| Radxa Rock 3A (RK3568, 4GB) | ~250-350 元 | 性能略强于 Pi 4B |
| 二手 Mini PC (x86) | ~300-500 元 | 原生 x86，最省事 |
| 树莓派 5 8GB | ~650 元 | 原方案，当前性价比低 |

| 资源 | 说明 |
|------|------|
| [Orange Pi 官方文档](http://www.orangepi.org/) | 硬件、系统烧录 |
| [Armbian](https://www.armbian.com/) | 多板型通用 Ubuntu 镜像 |
| [Raspberry Pi Imager](https://www.raspberrypi.com/software/) | 烧录工具（树莓派专用）|
| Ubuntu 22.04 ARM64 / x86 镜像 | 选对应架构的镜像烧录 |

---

## Track C: 感知（传感器 + 视觉）

### 书籍

| 书名 | 作者 | 适合阶段 | 说明 |
|------|------|---------|------|
| 《机器视觉》 | 张广军 | C2 | 结构光/线激光/双目最扎实的中文教材 |
| 《计算机视觉：算法与应用》 | Szeliski | C3,C5-C6 | 领域圣经 |
| 《Multiple View Geometry》 | Hartley & Zisserman | C6 | 多视角几何参考，难度较高 |
| 《学习 OpenCV 3》 | Kaehler & Bradski | C0-C2 | OpenCV 工具书 |

### 深度学习

| 书名 | 说明 |
|------|------|
| 《动手学深度学习》(李沐) | 中文有代码，CNN 基础 |
| [CS231n](http://cs231n.stanford.edu/) | Stanford 深度学习视觉课 |

### 线激光 / 结构光（C2）

| 资源 | 说明 |
|------|------|
| [First Principles of Computer Vision](https://www.youtube.com/@firstprinciplesofcomputerv3258) | YouTube，激光三角测量讲解 |
| [Open3D 点云教程](http://www.open3d.org/docs/latest/tutorial/) | 3D 可视化 |
| OpenCV `cv2.projectPoints()` `cv2.triangulatePoints()` | 投影与三角测量 API |

### LDS 激光雷达（C4）

| 资源 | 说明 |
|------|------|
| [YDLIDAR 开发手册](https://www.ydlidar.com/) | Python SDK, ROS 驱动 |
| [RPLIDAR SDK](https://github.com/Slamtec/rplidar_sdk) | 思岚官方 SDK |
| [Hector SLAM 论文](https://wiki.ros.org/hector_slam) | scan-to-map 匹配 |

### ToF / 红外（C5）

| 资源 | 说明 |
|------|------|
| [VL53L1X Datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf) | ST 官方数据手册 |
| [ST VL53L0X API 文档](https://www.st.com/resource/en/user_manual/dm00280486.pdf) | 寄存器级驱动参考 |

### YOLO / AI 检测（C3）

| 资源 | 说明 |
|------|------|
| [Ultralytics YOLO 文档](https://docs.ultralytics.com/) | 训练、导出 ONNX |
| [NanoDet](https://github.com/RangiLyu/nanodet) | 移动端轻量检测器 |
| [Roboflow](https://roboflow.com/) | 数据标注平台 |

---

## Track D: 机械与硬件

### 底盘与电机

| 资源 | 说明 |
|------|------|
| 淘宝"差速底盘 机器人" | 带编码器电机 + 安装孔位，入门首选 |
| 咸鱼"扫地机 拆机 底盘" | 二手扫地机拆机件，性价比高 |
| TB6612 / L298N 电机驱动 | 淘宝搜型号即可 |
| [直流电机 PID 控制](https://www.embedded.com/pid-control-for-dc-motors/) | 理论参考 |

### 供电

| 资源 | 说明 |
|------|------|
| 淘宝"12V 锂电池 机器人" | 选带保护板的 |
| LM2596 / MINI560 降压模块 | 12V→5V 给 Pi 供电 |

### 传感器硬件

| 用途 | 型号/关键词 | 价格 |
|------|-----------|------|
| 线激光 | 淘宝"650nm 一字线激光 USB" | ~15-50 元 |
| LDS 雷达 | 淘宝"YDLIDAR X4" | ~300 元 |
| ToF 测距 | 淘宝"VL53L1X 模块" | ~30 元 |
| 红外避障 | 淘宝"红外避障模块" | ~2 元/对 |
| 碰撞检测 | 微动开关 | ~1 元 |
| 滤光片 | 650nm 带通滤光片 | ~10 元 |

---

## 扫地机行业参考

### 主流产品传感器配置

| 产品 | LDS | 线激光 | RGB 相机 | 红外悬崖 | ToF |
|------|-----|--------|----------|----------|-----|
| Roborock S8 | 有 | 有 | 有 | 有 | - |
| Roborock Q Revo | 有 | - | - | 有 | - |
| Dreame X40 | 有 | 有 | 有 | 有 | - |
| iRobot Roomba j7+ | - | - | 有 | 有 | - |
| Ecovacs X2 | 有 | 有 | 有 | 有 | - |

### 关键供应商

| 品牌 | 产品 | 说明 |
|------|------|------|
| 思岚 SLAMTEC | LDS 雷达、SLAM 模组 | 国内扫地机雷达主要供应商 |
| 欢创 Camsense | 三角法雷达 | 多家扫地机厂商供应商 |
| ST意法半导体 | VL53L0X/VL53L1X ToF | 扫地机悬崖传感器常用 |
| 奥比中光 | 深度相机 | 3D 结构光方案 |

### 拆解参考

| 资源 | 说明 |
|------|------|
| B 站"Roborock 拆解" | 看内部传感器布局 |
| B 站"扫地机拆机评测" | 大量拆解视频 |
| iFixit 扫地机拆解 | 看 iRobot Roomba 结构 |
| [Roborock 技术博客](https://blog.roborock.com/) | 官方技术介绍 |

---

## 关键工具与库

| 工具 | 用途 | 轨道 |
|------|------|------|
| OpenCV | 图像处理、相机标定、激光线提取 | C |
| NumPy | 数组计算基础 | C |
| Open3D | 点云处理与可视化 | C |
| Matplotlib | 数据可视化 | C |
| PySerial | 串口通信（LDS 雷达）| C |
| smbus2 | I2C 通信（VL53L0X）| C |
| ONNX Runtime | AI 模型推理部署 | C |
| PyTorch | 深度学习训练 | C |
| Ultralytics | YOLO 训练 | C |
| ROS2 Humble | 机器人框架 | B |
| rclpy | ROS2 Python 客户端库 | B |
| SLAM Toolbox | 2D SLAM | B |
| Nav2 | 导航栈 | B |
| Keil MDK | STM32 IDE | A |
| STM32CubeMX | 外设初始化代码 | A |
| 烧录工具 (Armbian/Imager) | 烧录系统 | B |

---

## 中文社区

| 资源 | 说明 |
|------|------|
| [鱼香 ROS](https://fishros.org/) | 中文 ROS/ROS2 学习社区 |
| [自动驾驶之心](https://www.zdjszx.com/) | 感知方向中文社区 |
| 知乎"扫地机器人"话题 | 产品测评、技术讨论 |
| B 站"STM32 平衡车" | 大量项目实操视频 |
| 52pojie 扫地机逆向 | 协议分析和破解 |
| [OpenRobotics 社区](https://discourse.ros.org/) | ROS 官方论坛 |

## 云 GPU 平台

| 平台 | 说明 |
|------|------|
| [AutoDL](https://www.autodl.com/) | 国内按小时租，性价比高 |
| [恒源云](https://www.gpushare.com/) | 国内 GPU 租用 |
| Google Colab | 免费 T4，轻量实验够用 |
