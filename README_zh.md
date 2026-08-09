# 麦克纳姆轮全向移动机器人 (MCR)

**基于 FreeRTOS + ROS2 的多层架构自主移动机器人**

面向嵌入式 MCU+RTOS 岗位的全栈机器人项目。覆盖完整技术栈：裸机驱动 → 实时操作系统 → 通信协议 → ROS2 → SLAM/导航 → 计算机视觉 → 3D 感知。

## 系统架构

```
Raspberry Pi 5 (Ubuntu 24.04 + ROS2 Jazzy)
  ├── SLAM 建图 (slam_toolbox)
  ├── Nav2 全向导航
  ├── ros2_control + 麦克纳姆轮运动学
  ├── 感知层 (YOLO 检测、相机标定、激光三角法 3D 扫描)
  └── 串口通信协议 (UART)
          │
STM32 (FreeRTOS + HAL)
  ├── 4 路 PID 速度闭环 (100 Hz)
  ├── 4 路正交编码器读数
  ├── ToF 紧急避障刹车
  ├── IMU 姿态解算 (Mahony 滤波器)
  └── DMA 串口通信
```

## 目录结构

```
mecanum-robot/
├── firmware/                          # STM32 FreeRTOS 固件
│   └── Core/
│       ├── Inc/                       # 头文件 (pid, motor, encoder, robot_control)
│       └── Src/                       # 实现 + main.c (FreeRTOS 5 任务)
├── shared/                            # 通信协议 (Pi 和 STM32 共享)
│   ├── protocol.h                     # 帧格式定义
│   └── protocol.c                     # CRC16 + 编解码
├── ros2_ws/                           # ROS2 Jazzy 工作空间
│   └── src/
│       ├── mcr_bringup/               # 硬件接口 + 串口协议 + 运动学
│       ├── mcr_description/           # URDF 模型 (4 麦克纳姆轮 + 传感器)
│       └── mcr_navigation/            # Nav2 + SLAM 配置
├── perception/                        # 计算机视觉 & 3D 扫描
│   ├── laser_triangulation/           # 激光线 3D 扫描 (<1mm @ 30cm)
│   ├── detection/                     # YOLOv8 ONNX 目标检测
│   └── camera/                        # 相机标定 (棋盘格)
└── docs/
    ├── resources.md                   # 硬件选型与供应商参考
    └── ros2-guide.md                  # ROS2 学习资源整理
```

## 快速开始

### 1. 硬件清单

| 组件 | 型号 | 数量 |
|------|------|------|
| 主控 | Raspberry Pi 5 (8GB) | 1 |
| MCU | STM32 (FreeRTOS) | 1 |
| 底盘 | 4WD 麦克纳姆轮底盘 | 1 |
| 电机 | JGA25-370 编码器减速电机 | 4 |
| 电机驱动 | TB6612FNG 双路 H 桥 | 2 |
| 激光雷达 | LD19 / LD06 360° | 1 |
| IMU | MPU6050 | 1 |
| ToF 测距 | VL53L0X / VL53L1X | 1 |
| 相机 | USB 摄像头 (或 OAK-D Lite) | 1 |
| 电源 | 3S 锂电池 + 降压模块 | 1 套 |

### 2. 树莓派 5 环境搭建

```bash
# 用 Raspberry Pi Imager 烧录 Ubuntu Server 24.04 (arm64)
# 启动后执行：
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# 编译 ROS2 工作空间
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. STM32 固件

1. 用 CubeMX 配置外设：UART、TIM 编码器模式 ×4、TIM PWM ×4、I2C (IMU)
2. 开启 FreeRTOS (CMSIS_V2 接口)
3. 将 `firmware/Core/` 下的文件复制到生成的工程中
4. 根据实际接线修改 `motor.c` 中的引脚映射
5. 编译烧录

### 4. 启动

```bash
# 终端 1：启动机器人底层驱动
ros2 launch mcr_bringup robot.launch.py

# 终端 2：启动 SLAM + 导航
ros2 launch mcr_navigation navigation.launch.py

# 终端 3：键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5. 感知模块

```bash
pip install opencv-python numpy open3d onnxruntime

# 相机标定
python perception/camera/calibration.py

# YOLO 目标检测
python perception/detection/yolo_detect.py

# 激光三角法 3D 扫描
cd perception/laser_triangulation
python point_cloud_scanner.py
```

## 核心技术展示

- **FreeRTOS**：5 任务实时调度（Ctrl/Comm/Sensor/Monitor/Idle），优先级管理，栈溢出监控
- **PID 控制**：4 路独立速度闭环 (100Hz)，增量式算法 + 抗积分饱和
- **UART/DMA 通信**：自定义二进制协议（双字节帧同步 + CRC16-MODBUS + 序号校验）
- **麦克纳姆轮运动学**：正/逆运动学解算，支持全向移动（横移/斜走/原地旋转）
- **ros2_control**：自定义 `SystemInterface` 硬件接口，桥接串口 ↔ ROS2 控制器
- **Nav2 导航**：全向路径规划 (DWB 局部规划器 + SmacPlannerHybrid 全局规划器)
- **SLAM**：slam_toolbox 在线异步建图（激光 + IMU + 轮式里程计融合）
- **激光三角法**：线激光 + 相机实现 3D 点云扫描，精度 <1mm
- **YOLO 推理**：ONNX Runtime 边缘端目标检测

## 相关项目

- **[stm32-balance-car](https://github.com/finnyoun9)** — STM32 平衡小车（基于江科大自平衡小车教程）。展示更多 MCU 技能：MPU6050 DMP、互补滤波、串级 PID（直立/速度/转向环）、NRF24L01 无线遥控。

## 实施路线

详见 [项目计划](https://github.com/finnyoun9/mecanum-robot/blob/main/docs/../plans/mcu-rtos-ros2-scalable-crescent.md)

1. **Phase 1** — 硬件搭建与裸机电机验证
2. **Phase 2** — FreeRTOS 多任务 + 4 路 PID 闭环
3. **Phase 3** — Pi ↔ STM32 自定义通信协议
4. **Phase 4** — ROS2 驱动层 (ros2_control + 麦克纳姆轮运动学)
5. **Phase 5** — SLAM 建图 + Nav2 自主导航
6. **Phase 6** — 视觉增强（Apriltag 定位 + 物体跟踪）

## License

MIT
