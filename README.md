# Mecanum Mobile Robot (MCR)

**Multi-Tier RTOS + ROS2 Autonomous Mobile Robot with Mecanum Wheels**

A full-stack embedded robotics project targeting MCU+RTOS job roles.

## Architecture

```
Raspberry Pi 5 (Ubuntu 24.04 + ROS2 Jazzy)
  ├── SLAM (slam_toolbox)
  ├── Nav2 (omnidirectional navigation)
  ├── ros2_control + Mecanum IK
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
├── firmware/               # STM32 FreeRTOS firmware
│   └── Core/
│       ├── Inc/            # Headers (pid, motor, encoder, robot_control)
│       └── Src/            # Implementation + main.c (FreeRTOS tasks)
├── shared/                 # Protocol definition (shared by Pi & STM32)
│   ├── protocol.h
│   └── protocol.c
├── ros2_ws/                # ROS2 workspace
│   └── src/
│       ├── mcr_bringup/    # Hardware interface + serial + kinematics
│       ├── mcr_description/# URDF model + robot description
│       └── mcr_navigation/ # Nav2 + SLAM configuration
└── docs/
```

## Getting Started

### 1. Hardware

See [plan](./docs/../plans/mcu-rtos-ros2-scalable-crescent.md) for the BOM.

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

## Key Skills Demonstrated

- **FreeRTOS**: 5-task real-time system, priority scheduling, stack monitoring
- **PID Control**: 4 independent velocity loops at 100 Hz
- **UART/DMA**: Custom binary protocol with CRC16, frame sync, sequence checking
- **Mecanum Kinematics**: Forward/inverse kinematics for omnidirectional motion
- **ros2_control**: Custom SystemInterface bridging serial ↔ ROS2
- **Nav2**: Omnidirectional path planning (DWB + SmacPlannerHybrid)
- **SLAM**: slam_toolbox with laser + IMU + odometry fusion

## License

MIT
