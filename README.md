# Mecanum Mobile Robot (MCR) / 麦克纳姆轮全向移动机器人

## Multi-tier RTOS + ROS2 Autonomous Mobile Robot with Mecanum Wheels / 基于 FreeRTOS + ROS2 的多层架构自主移动机器人

A full-stack embedded robotics project targeting **MCU+RTOS** job roles.
Covers the complete stack: bare-metal drivers → real-time OS → communication protocols → ROS2 → SLAM/navigation → computer vision → mobile manipulation.

面向嵌入式 **MCU+RTOS** 岗位的全栈机器人项目。覆盖完整技术栈:裸机驱动 → 实时操作系统 → 通信协议 → ROS2 → SLAM/导航 → 计算机视觉 → 移动操作。

## Mobile Manipulator Extension / 移动操作扩展

The project now includes a **LeArm 6-DOF manipulator with an STM32 controller**. The arm is developed as an independent subsystem under [`manipulator/`](manipulator/) and will be integrated with the existing Raspberry Pi 5 + STM32 mecanum base through ROS 2.

项目新增 **LeArm 六自由度机械臂 + STM32 核心模组**。机械臂作为 [`manipulator/`](manipulator/) 独立子系统开发，后续通过 ROS 2 与现有 Raspberry Pi 5 + STM32 麦克纳姆底盘联动。

- 装配散件并记录结构、回差、关节限位和线束设计
- 实现总线舵机通信、UART/DMA、FreeRTOS 调度、看门狗和故障处理
- 接入自定义 `ros2_control` hardware interface 和 MoveIt 2
- 完成导航、AprilTag 定位、抓取、运输和放置演示

---

## Progress / 开发进度

- **ROS2 Jazzy 开发环境** ✅ — Ubuntu 24.04 (arm64) Docker 容器跑在树莓派 5 上,`colcon build` 全绿,3 个功能包、8 项测试通过
- **ros2_control 硬件接口** ✅ — 自定义 `SystemInterface` 已移植到 Jazzy 4.x API(`on_init` / `on_activate` / `on_deactivate`),以共享库插件加载
- **URDF 模型** ✅ — 4 麦克纳姆轮 + 传感器模型,已通过 xacro 展开并在 RViz2 中渲染(见下方截图)
- **串口二进制协议** ✅ — 自定义协议 + CRC16-MODBUS 校验,Pi 与 STM32 共享的 C 库
- **麦克纳姆轮运动学** ✅ — 正/逆运动学解算 + 单元测试(gtest,固件内也有 C 版)
- **NRF24L01 无线遥控** ⚠️ — 实物收发、K1/K9、250 ms 失联停车和低速全向控制曾完成真机验收；旧 Blue Pill 已因复位节点异常换新板（SWD/烧录已验），车端 NRF24L01+ 模块电源短路已下线、新模块 SPI 回读仍未建立，待更换/复验(见 [docs/remote_control.md](docs/remote_control.md))
- **ROS2 协议闭环** ✅ — `/cmd_vel` → `mecanum_drive_controller` → 自定义硬件接口 → STM32 UART 模拟器全链路打通；0.3 m/s 指令在协议模拟中得到 0.302 m/s 里程计，尚不是真实底盘结果
- **STM32 速度闭环** ✅ — 单轮 PI 和独立四轮闭环目标已完成空载与低速落地基础验收；完整 `firmware_arch_main()` 的 UART/I2C MSP 和长期带载测试仍未完成
- **四轮开环真机控制** ✅ — 四个电机经 TB6612 已完成 6 V 空载正反转与方向一致性验证；正式供电方案见 [docs/power-system.md](docs/power-system.md)
- **SIL 软件在环测试** ✅ — 固件编译为 Linux 原生可执行文件，Mock HAL + FreeRTOS 调度模拟器在 CI 中验证命令解析、PWM、编码器累积和里程计数据链；不替代真机 PID 性能测试(见 [docs/resume-highlights.md](docs/resume-highlights.md))
- **LD06 激光雷达** ✅ — 经 CH340 USB-TTL 在 Pi 5 上完成 230400 波特率原始帧、ROS 2 `/scan` 约 10 Hz 和二维 `PointCloud2` 实测；已以 submodule 接入 bringup（`robot.launch.py` + URDF `laser_link` TF，GCC-13 本地 patch）
- **IMX219 + YOLOv8n** ✅ — CSI 相机 640×480 连续采集超过 30 FPS；Pi 5 CPU 上 ONNX Runtime 实测约 6 FPS，显示器/人体目标可识别；当前为独立验证，尚未封装成 ROS 2 节点
- **Nav2 + SLAM** 🚧 — 配置骨架已就位，LD06 `/scan` 已独立验证，待接入 bringup 并与真实里程计、TF 联调

> 当前最重要的缺口是 **M4：闭合 Pi↔STM32 真机 UART/里程计链路**——`rtos_drive` FreeRTOS 目标已上板，洪流测试暴露双向帧错误（波特率/电气层待查，Pi 侧已实测排除）和错误风暴下的 lockup（待复现取证）；车端 NRF24 模块仍待更换。具体证据边界见 [项目状态](docs/project-status.md)。

> 后续开发请先读 [docs/project-status.md](docs/project-status.md)，从当前 M4/Pi 侧集成缺口继续推进，不要把 SIL、协议模拟器或独立传感器测试写成整车闭环结果。

> RViz2 渲染效果(无头 Xvfb 截图,1280×800):
> - 静止渲染:[docs/screenshots/rviz2.png](docs/screenshots/rviz2.png)
> - 麦克纳姆方形轨迹(边 0.45 m,含横向平移,机器人运动中):[docs/screenshots/rviz2_mecanum_square_moving.png](docs/screenshots/rviz2_mecanum_square_moving.png)
> - 方形轨迹完成:[docs/screenshots/rviz2_mecanum_square_complete.png](docs/screenshots/rviz2_mecanum_square_complete.png)

## Development Environment / 开发环境

| 层 | 说明 |
| --- | --- |
| 开发机 | macOS (Apple Silicon),用于编辑代码与 git 管理 |
| 机器人主机 | Raspberry Pi 5 (8GB),原装 **Raspberry Pi OS (Debian 12 Bookworm)**,不重刷系统 |
| ROS2 | Jazzy 运行在 **Ubuntu 24.04 Docker 容器** 内 (`mcr-ros2:jazzy`),arm64 原生速度,无 QEMU 模拟 |
| GUI 工具 | `mcr-ros2:jazzy-gui` 附加 rviz2、robot_state_publisher、Xvfb、ImageMagick,可无显示器渲染截图 |
| 构建工具 | colcon / ament_cmake / CMake,共享协议库用 `BUILD_SHARED_LIBS` 便于 pluginlib 加载 |
| 固件工具链 | STM32CubeMX + FreeRTOS (CMSIS_V2) + arm-none-eabi GCC |
| 网络 | 容器用 `--network host` 保证 DDS 组播;Pi 上 mihomo 代理 (127.0.0.1:7890) 在监听时传给 docker build 加速 |

为什么 Pi 不装 Ubuntu:主机保留原装 Raspberry Pi OS,ROS2 跑在 Ubuntu 24.04 容器里,天然 arm64 原生速度,无需模拟。摄像头 (CSI) 留在宿主机原生运行,不进容器 —— 详见 [`docker/README.md`](docker/README.md)。

## Architecture / 系统架构

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
  ├── NRF24L01 无线遥控 (全向控制)
  ├── ToF 紧急避障刹车
  ├── IMU 姿态解算 (Mahony 滤波器)
  └── DMA/IDLE 串口通信（代码路径已具备，CubeMX 与真机待接入）
```

## Directory Structure / 目录结构

```
mecanum-robot/
├── firmware/                          # STM32 FreeRTOS 固件
│   ├── Core/
│   │   ├── Inc/                       # 头文件 (pid, motor, encoder, robot_control)
│   │   ├── Src/                       # 实现 + main.c (FreeRTOS 任务)
│   │   ├── SIL/                       # SIL 软件在环测试 (★★★ 简历亮点)
│   │   │   ├── sil_main.c             #   测试入口,闭环验证 PID + 编码器 + 协议
│   │   │   └── mocks/                 #   Mock HAL (GPIO/TIM/UART/I2C/FreeRTOS)
│   │   └── CMakeLists.txt             # SIL 编译脚本
│   └── remote_controller/             # NRF24L01 无线遥控器固件 (江协科技)
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
    ├── ros2-guide.md                  # ROS2 学习资源整理
    ├── ros2-learning.md               # ROS2 学习笔记
    ├── remote_control.md              # NRF24L01 无线遥控方案 (协议/接线/映射)
    ├── resume-highlights.md           # ★ 简历亮点与面试准备 (SIL/协议/RTOS/PID)
    └── screenshots/                   # 界面截图 (rviz2 渲染)
```

## Getting Started / 快速开始

### 1. Hardware / 硬件清单

| 组件 Component | 型号 Model | 数量 Qty |
| --- | --- | --- |
| 主控 SBC | Raspberry Pi 5 (8GB) | 1 |
| MCU | STM32 (FreeRTOS) | 1 |
| 底盘 Chassis | 4WD 麦克纳姆轮底盘 | 1 |
| 电机 Motors | JGA25-370 编码器减速电机 | 4 |
| 电机驱动 Driver | TB6612FNG 双路 H 桥 | 2 |
| 激光雷达 LiDAR | LD06 360° + CH340 USB-TTL | 1 |
| IMU | MPU6050 | 1 |
| ToF 测距 | VL53L0X（当前驱动不兼容 VL53L1X） | 1 |
| 相机 Camera | IMX219 8MP (CSI 排线) | 1 |
| 无线遥控 Remote | NRF24L01+ 收发模块 + 江协科技手柄 | 2 |
| 电源 Power | 3S 锂电池 + 降压模块 | 1 套 |

当前实装的电源树、降压模块分工和首次上电检查见 [供电方案](docs/power-system.md)。

### 2. Pi 5 Setup / 树莓派 5 环境搭建

树莓派用原装 **Raspberry Pi OS (Debian 12)**,ROS2 跑在 Ubuntu 24.04 的
Docker 容器里,无需重刷系统。摄像头留在宿主机原生跑,不进容器。

```bash
git clone https://github.com/finnyoun9/mecanum-robot.git
cd mecanum-robot

# 首次运行自动构建 ROS2 Jazzy 镜像,然后进入容器 shell
./docker/run.sh

# 容器内:
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. STM32 Firmware / 固件

1. CubeMX 配置外设:UART、TIM 编码器模式 ×4、TIM PWM ×4、I2C (IMU)
2. 开启 FreeRTOS (CMSIS_V2 接口)
3. 将 `firmware/Core/` 下文件复制到生成的工程
4. 按实际接线修改 `motor.c` 引脚映射
5. 编译烧录

### 4. Launch / 启动

```bash
# 终端 1:启动机器人底层驱动
ros2 launch mcr_bringup robot.launch.py

# 终端 2:启动 SLAM + 导航
ros2 launch mcr_navigation navigation.launch.py

# 终端 3:键盘遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5. Perception / 感知模块

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

> 当前 `yolo_detect.py` 仍是通用 OpenCV `VideoCapture`/桌面显示版本；IMX219 真机验证使用宿主机 `Picamera2 + ONNX Runtime`，正式 CSI/headless 入口尚待合入，不能直接把上面的命令当作 Pi 5 已验收启动方式。

## Key Skills / 核心技术展示

- **SIL 软件在环测试**:Mock HAL 层 + FreeRTOS 轮询调度模拟，固件编译为 Linux 原生可执行文件，CI 自动验证控制数据链 (★★★)
- **FreeRTOS**:5 任务实时调度,优先级管理,栈溢出监控
- **PID 控制**:4 路位置式速度 PID 代码与 SIL 数据链验证 (100 Hz)，真机优先按 PI 调参
- **UART/DMA 通信**:自定义二进制协议与 DMA/IDLE 接收代码，真机 CubeMX/时序待验证
- **麦克纳姆轮运动学**:正/逆运动学,支持全向移动(横移 / 斜走 / 原地旋转)
- **ros2_control**:自定义 `SystemInterface` 硬件接口,桥接串口 ↔ ROS2 控制器
- **Nav2 导航**:全向路径规划 (DWB 局部规划器 + SmacPlannerHybrid 全局规划器)
- **SLAM**:slam_toolbox 在线异步建图(激光 + IMU + 轮式里程计融合)
- **激光三角法**:线激光 + 相机 3D 点云扫描代码，30 cm 下 <1 mm 是待实测目标
- **YOLO 推理**:ONNX Runtime 边缘端目标检测

## Roadmap / 实施路线

1. **Phase 1** — 硬件搭建与裸机电机验证
2. **Phase 2** — FreeRTOS 多任务 + 4 路 PID 闭环
3. **Phase 3** — Pi ↔ STM32 自定义通信协议
4. **Phase 4** — ROS2 驱动层 (ros2_control + 麦克纳姆轮运动学)
5. **Phase 5** — SLAM 建图 + Nav2 自主导航
6. **Phase 6** — 视觉增强(Apriltag 定位 + 物体跟踪)

## Related Projects / 相关项目

- **[stm32-pid-balancer](https://github.com/finnyoun9/stm32-pid-balancer)** — PID 控制实验台。当前有通用 PID、host test 和单电机 m01 固件，自平衡真机尚未完成。

## License / 许可证

MIT
