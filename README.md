# Mecanum Mobile Robot (MCR)

**Multi-Tier RTOS + ROS2 Autonomous Mobile Robot with Mecanum Wheels**

A full-stack embedded robotics project targeting MCU+RTOS job roles.
Covers the complete stack: bare-metal drivers → real-time OS → communication protocols → ROS2 → SLAM/navigation → computer vision → 3D perception.

## Architecture

```
Raspberry Pi 5 (Ubuntu 24.04 + ROS2 Jazzy)
  ├── SLAM (slam_toolbox)
  ├── Nav2 (omnidirectional navigation)
  ├── ros2_control + Mecanum IK
  ├── Perception (YOLO, camera calibration, laser triangulation 3D scanner)
  └── Serial Protocol (UART)
          │
STM32 (FreeRTOS + HAL)
  ├── 4x PID speed loops (100 Hz)
  ├── 4x quadrature encoder readers
  ├── ToF emergency brake
  ├── IMU sensor fusion (Mahony filter)
  └── DMA UART communication
```

## Directory Structure

```
mecanum-robot/
├── firmware/                          # STM32 FreeRTOS firmware
│   └── Core/
│       ├── Inc/                       # Headers (pid, motor, encoder, robot_control)
│       └── Src/                       # Implementation + main.c (FreeRTOS tasks)
├── shared/                            # Binary protocol shared by Pi & STM32
│   ├── protocol.h
│   └── protocol.c
├── ros2_ws/                           # ROS2 Jazzy workspace
│   └── src/
│       ├── mcr_bringup/               # Hardware interface + serial + kinematics
│       ├── mcr_description/           # URDF model (4 mecanum wheels + sensors)
│       └── mcr_navigation/            # Nav2 + SLAM configuration
├── perception/                        # Computer vision & 3D scanning
│   ├── laser_triangulation/           # Laser-line 3D scanner (<1mm @ 30cm)
│   ├── detection/                     # YOLOv8 ONNX object detection
│   └── camera/                        # Camera calibration (chessboard)
└── docs/
    ├── resources.md                   # Hardware vendor & sensor reference
    └── ros2-guide.md                  # ROS2 learning resources (Chinese)
```

## Getting Started

### 1. Hardware

| Component | Model | Qty |
|-----------|-------|-----|
| SBC | Raspberry Pi 5 (8GB) | 1 |
| MCU | STM32 (FreeRTOS) | 1 |
| Chassis | 4WD Mecanum wheel chassis | 1 |
| Motors | JGA25-370 DC gear motor w/ encoder | 4 |
| Motor Driver | TB6612FNG dual H-bridge | 2 |
| LiDAR | LD19 / LD06 360° | 1 |
| IMU | MPU9250 / ICM-20948 | 1 |
| ToF | VL53L0X / VL53L1X | 1 |
| Camera | USB camera (or OAK-D Lite) | 1 |
| Battery | 3S LiPo + buck converter | 1 set |

### 2. Pi 5 Setup

```bash
# Flash Ubuntu Server 24.04 (arm64) with Raspberry Pi Imager
# Boot, then:
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Build
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. STM32 Firmware

1. Open CubeMX, configure peripherals (UART, TIM encoder ×4, TIM PWM ×4, I2C for IMU)
2. Generate code with FreeRTOS (CMSIS_V2)
3. Copy `firmware/Core/` files into generated project
4. Wire `motor.c` pin mappings to match your board
5. Build and flash

### 4. Launch

```bash
# Terminal 1: Robot bringup
ros2 launch mcr_bringup robot.launch.py

# Terminal 2: SLAM + Navigation
ros2 launch mcr_navigation navigation.launch.py

# Terminal 3: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5. Perception

```bash
pip install opencv-python numpy open3d onnxruntime

# Camera calibration
python perception/camera/calibration.py

# YOLO object detection
python perception/detection/yolo_detect.py

# Laser triangulation 3D scanner
cd perception/laser_triangulation
python point_cloud_scanner.py
```

## Key Skills Demonstrated

- **FreeRTOS**: 5-task real-time system, priority scheduling, stack watermark monitoring
- **PID Control**: 4 independent velocity loops at 100 Hz with anti-windup
- **UART/DMA**: Custom binary protocol (CRC16-MODBUS, frame sync, sequence checking)
- **Mecanum Kinematics**: Forward/inverse kinematics for omnidirectional motion
- **ros2_control**: Custom `SystemInterface` bridging UART serial ↔ ROS2 controller manager
- **Nav2**: Omnidirectional path planning (DWB + SmacPlannerHybrid)
- **SLAM**: slam_toolbox with laser + IMU + wheel odometry fusion
- **Laser Triangulation**: 3D point cloud scanning via line-laser + camera, <1mm precision
- **YOLO Inference**: ONNX Runtime object detection on edge device

## Related Projects

- **[stm32-balance-car](https://github.com/finnyoun9)** — STM32 self-balancing robot (based on 江科大课程). Demonstrates additional MCU skills: MPU6050 DMP, complementary filtering, cascaded PID (angle/speed/turn loops), NRF24L01 wireless control.

## License

MIT
