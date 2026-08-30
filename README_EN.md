<div align="center">

# MCR · Mecanum Autonomous Mobile Robot

**STM32F103 + FreeRTOS + Raspberry Pi 5 + ROS 2 Jazzy**

An integrated robotics platform spanning real-time motor control, wireless teleoperation, a custom serial protocol, `ros2_control`, sensor fusion, SLAM, and edge vision.

[![中文](https://img.shields.io/badge/Language-简体中文-334155?style=for-the-badge)](README.md)
[![English](https://img.shields.io/badge/Language-English-2563eb?style=for-the-badge)](README_EN.md)

[![Firmware Tests](https://github.com/finnyoun9/mecanum-robot/actions/workflows/firmware-tests.yml/badge.svg)](https://github.com/finnyoun9/mecanum-robot/actions/workflows/firmware-tests.yml)
[![ROS 2 Build](https://github.com/finnyoun9/mecanum-robot/actions/workflows/ros2-build.yml/badge.svg)](https://github.com/finnyoun9/mecanum-robot/actions/workflows/ros2-build.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)
![FreeRTOS](https://img.shields.io/badge/RTOS-FreeRTOS-16A085)
![STM32](https://img.shields.io/badge/MCU-STM32F103-03234B?logo=stmicroelectronics)
![Raspberry Pi](https://img.shields.io/badge/SBC-Raspberry_Pi_5-A22846?logo=raspberrypi)

<img src="docs/screenshots/robot-hero-2026-08-31.jpg" alt="MCR hardware, top view (2026-08-31, M5 stage)" width="88%">

<sub>Top view · 2026-08-31 · Pi 5 + IMX219 + LD06 + ToF/OLED onboard</sub>

<img src="docs/screenshots/rviz2_mecanum_square_moving.png" alt="MCR executing an omnidirectional square trajectory in RViz2" width="88%">

</div>

> [!IMPORTANT]
> The main track is now **M5: mobile mapping quality**. The chassis, handset, Pi-to-STM32 link, ROS 2 control stack, and first SLAM run are closed. Next: gyro bias, EKF tuning, remapping, and Nav2 validation.

## Navigation

[Overview](#overview) · [Status](#current-status) · [Architecture](#architecture) · [Evidence](#measured-evidence) · [Quick Start](#quick-start) · [Roadmap](#roadmap) · [Documentation](#documentation)

## Overview

MCR is a full-stack robot built around real hardware. An STM32 handles deterministic control and safety, while a Raspberry Pi 5 runs ROS 2, fused localization, SLAM, and perception. A CRC16-protected binary protocol connects both layers.

Verification is a design goal. The same core firmware is exercised by hardware targets, host unit tests, and software-in-the-loop tests. Project records distinguish measured hardware evidence from simulated or inferred results.

### Highlights

- Four independent mecanum wheel speed loops with feedforward, slew limiting, and zero-speed settling
- NRF24L01 omnidirectional handset with e-stop, link-loss stop, and handset/Pi command arbitration
- MPU6050, ToF, OLED, encoders, LD06, and IMX219 sensor integration
- Custom UART binary protocol with CRC16, DMA circular reception, and communication watchdogs
- ROS 2 Jazzy, `ros2_control`, EKF, SLAM Toolbox, and Nav2 configuration
- Firmware SIL, host tests, ROS 2 tests, and GitHub Actions CI

## Current Status

| Subsystem | Status | Evidence boundary |
| --- | --- | --- |
| Chassis and closed-loop control | **Hardware verified** | Omnidirectional motion, feedforward, clean stopping, wheel consistency |
| Wireless control and safety | **Hardware verified** | K1, K9, K10, 250 ms watchdog, handset-priority arbitration |
| Pi-to-STM32 transport | **Closed** | 921600 8N1, bidirectional protocol, ODOM, ROS 2 hardware interface |
| IMU / ToF / OLED | **Hardware verified** | Shared I2C2 bus, continuous ranging, attitude data, status dashboard |
| LD06 LiDAR | **Hardware verified** | `/scan` near 10 Hz with stable static measurements |
| ROS 2 robot stack | **Closed** | `ros2_control`, controllers, EKF, TF, and odometry topics |
| SLAM | **In progress** | Mobile mapping works; map distortion and scan artifacts remain |
| Nav2 autonomous navigation | **Pending validation** | Configuration exists; real goal navigation is not yet accepted |
| IMX219 + YOLOv8n | **Experimental** | About 4.5–6 FPS on Pi 5 CPU; ROS UDP bridge implemented |
| Battery display | **Deferred** | Feature-gated firmware and tests exist; no current rewiring or flash |
| Mobile manipulator | **Early scaffold** | Host protocol and SIL skeleton; outside the chassis critical path |

See the dated [project status and evidence log](docs/project-status.md) for the authoritative boundary.

## Architecture

```mermaid
flowchart TB
    RC[Wireless handset] -->|NRF24L01| MCU
    ENC[Four encoders] --> MCU[STM32F103 · FreeRTOS]
    IMU[MPU6050] --> MCU
    TOF[ToF ranging] --> MCU
    MCU --> OLED[0.96-inch OLED]
    MCU -->|PWM + DIR| DRV[TB6612 ×2]
    DRV --> MOTOR[Four JGA25-370 motors]
    MCU <-->|UART 921600 · CRC16| PI[Raspberry Pi 5]
    LIDAR[LD06] -->|USB-TTL| PI
    CAMERA[IMX219 CSI] --> VISION[Host YOLOv8n]
    VISION -->|UDP| PI
    PI --> CONTROL[ros2_control]
    CONTROL --> EKF[robot_localization EKF]
    EKF --> SLAM[SLAM Toolbox]
    SLAM --> NAV[Nav2]
```

### Responsibility by Layer

| Layer | Technology | Responsibility |
| --- | --- | --- |
| Real-time control | STM32F103, FreeRTOS, C | 100 Hz wheel control, sensors, safety state, wireless control |
| Edge compute | Raspberry Pi 5, Debian 12 | Device access, containers, native CSI inference |
| Robotics middleware | ROS 2 Jazzy, Ubuntu 24.04 Docker | Controllers, TF, fusion, SLAM, navigation, perception bridge |
| Verification | CTest, SIL, GTest, GitHub Actions | Protocol, kinematics, control, drivers, and build regressions |

## Technical Map

A subsystem-grouped index of the technical topics in this project; theory, code location, and rationale for each node are in [Technical Map details](docs/technical-map.md).

```mermaid
flowchart TB
    ROOT((MCR Technical Map))

    subgraph RT[Real-time Control]
        RT1[FreeRTOS 5-task scheduling]
        RT2[PID / PI velocity loop]
        RT3[Timer PWM + EXTI encoder]
        RT4[Mecanum kinematics]
    end

    subgraph COMM[Communication Link]
        C1[UART protocol + CRC16]
        C2[DMA + ring buffer]
        C3[I2C bus]
        C4[SPI + NRF24 wireless]
    end

    subgraph SENSE[Perception & State Estimation]
        S1[Mahony attitude filter]
        S2[ToF ranging]
        S3[EKF state estimation]
        S4[LD06 SLAM mapping]
    end

    subgraph SYS[System & Verification]
        Y1[SIL software-in-the-loop]
        Y2[GitHub Actions CI]
        Y3[Safety state machine / arbitration]
        Y4[Power + ADC divider]
    end

    ROOT --> RT
    ROOT --> COMM
    ROOT --> SENSE
    ROOT --> SYS
```

## Measured Evidence

| Metric | Hardware result |
| --- | --- |
| Wheel spread at 2.5 rad/s | About **1.4%** with a clean transport link |
| STM32 ODOM | About **50 Hz**, zero CRC errors in accepted validation runs |
| ROS 2 odometry | Raw near **100 Hz**, EKF output near **50 Hz** |
| LD06 | `/scan` near **10 Hz**; static pointwise median standard deviation about **8.1 mm** |
| CSI capture | 640×480 RGB near **30 FPS** |
| YOLOv8n | About **4.5–6 FPS** end to end on Pi 5 CPU |
| Firmware regression | CMake/CTest **12/12** passing |

These numbers apply only to their recorded test conditions. They do not imply unmeasured payload, endurance, or final navigation performance.

## Hardware

| Module | Model / solution |
| --- | --- |
| Controller | STM32F103C8T6 Blue Pill |
| Robot computer | Raspberry Pi 5 8GB |
| Chassis | Four-wheel X-pattern mecanum base |
| Motors and drivers | JGA25-370 encoder motors ×4, TB6612FNG ×2 |
| Localization and safety | MPU6050, LD06, I2C ToF module |
| Interaction and control | 0.96-inch I2C OLED, NRF24L01+ handset |
| Vision | IMX219 8MP CSI camera |
| Power | 3S LiPo, separated regulators, protection chain |

See [wiring](docs/wiring.md) and the [3S power architecture](docs/power-system.md) before powering hardware.

## Quick Start

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/finnyoun9/mecanum-robot.git
cd mecanum-robot
```

### 2. Build the ROS 2 Jazzy Environment

The Pi keeps Raspberry Pi OS. ROS 2 runs in a native arm64 Ubuntu 24.04 container, while the CSI camera remains on the host.

```bash
./docker/run.sh --build

# Inside the container
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Build Production Firmware

```bash
cd firmware/Core/HW
make clean
make TARGET=rtos_drive RTOS=1 TOF=1 OLED=1 OLED_CTRL=SH1106
```

Flashing resets real hardware and can trigger the documented I2C reset issue. Read the [hardware target guide](firmware/Core/HW/README.md) before operating the chassis.

### 4. Launch the Robot and Mapping

```bash
# Inside the container: chassis, LiDAR, controllers, and EKF
ros2 launch mcr_bringup robot.launch.py

# Currently recommended isolated mapping mode
ros2 launch mcr_navigation navigation.launch.py nav2:=false rviz:=false
```

### 5. Run Firmware Tests

```bash
cmake -S firmware/Core -B firmware/Core/build
cmake --build firmware/Core/build -j4
ctest --test-dir firmware/Core/build --output-on-failure
```

## Visualization

<details>
<summary><strong>Open hardware photos</strong></summary>

| Top view (2026-08-31) | Chassis wiring detail |
| --- | --- |
| ![MCR top view](docs/screenshots/robot-hero-2026-08-31.jpg) | ![Chassis STM32/motor/IMU wiring](docs/screenshots/robot-wiring-detail.jpg) |

</details>

<details>
<summary><strong>Open the RViz2 validation gallery</strong></summary>

| Robot model | Omnidirectional trajectory | Completed square |
| --- | --- | --- |
| ![](docs/screenshots/rviz2.png) | ![](docs/screenshots/rviz2_mecanum_square_moving.png) | ![](docs/screenshots/rviz2_mecanum_square_complete.png) |

</details>

<details>
<summary><strong>Open current SLAM mapping state (M5 in progress, not a success case)</strong></summary>

Three stages of the same teleop mapping session (2026-08-30), shown as-is with the current scalloped distortion and scan-matching artifacts, for cross-reference with the gyro-bias investigation logged in [project status](docs/project-status.md). This does not represent finished mapping.

| Stage 1 | Stage 2 | Stage 3 |
| --- | --- | --- |
| ![SLAM mapping stage 1, scalloped distortion](docs/screenshots/2026-08-30_slam_map.png) | ![SLAM mapping stage 2, distortion grows with the trajectory](docs/screenshots/2026-08-30_slam_map2.png) | ![SLAM mapping stage 3, scan-matching artifacts more visible](docs/screenshots/2026-08-30_slam_map3.png) |

</details>

## Roadmap

- [x] M1 · Hardware, PWM, direction, and encoder calibration
- [x] M2 · Single/four-wheel speed control, feedforward, and SIL
- [x] M3 · Wireless control, safety state machine, and basic hardware motion
- [x] M4 · Pi-to-STM32 transport, ROS 2 hardware interface, and topic closure
- [ ] M5 · Sensor fusion, mobile mapping quality, and Nav2 (in progress)
- [ ] M6 · Vision-assisted localization, interaction, and long-duration validation

Shortest path forward: **gyro bias calibration → EKF validation → remapping → Nav2 goal navigation → 30-minute stability run**.

## Documentation

| Document | Scope |
| --- | --- |
| [Project status](docs/project-status.md) | Latest progress, hardware evidence, blockers, next steps |
| [Technical map](docs/technical-map.md) | Subsystem-grouped topics: theory, code location, rationale |
| [Wiring guide](docs/wiring.md) | STM32, TB6612, encoder, I2C, and NRF24 pinout |
| [Power system](docs/power-system.md) | 3S battery tree, protection, and failure records |
| [Closed-loop roadmap](docs/hardware-closed-loop-roadmap.md) | Control bring-up, calibration, and acceptance |
| [Wireless control](docs/remote_control.md) | Protocol, keys, wiring, and safety behavior |
| [ROS 2 Docker](docker/README.md) | Pi host, container, devices, and remote RViz2 |

<details>
<summary><strong>Repository layout</strong></summary>

```text
mecanum-robot/
├── firmware/              STM32 chassis, handset, and manipulator firmware
├── shared/                Pi/STM32 protocol and CRC16
├── ros2_ws/src/           bringup, description, navigation, perception
├── perception/            CSI/YOLO, calibration, laser triangulation experiments
├── tools/                 link, encoder, wheel, IMU, and ToF diagnostics
├── docker/                ROS 2 Jazzy on Raspberry Pi OS
└── docs/                  wiring, power, status, roadmaps, evidence
```

</details>

---

<div align="center">

**Evidence first. Hardware first. Safety first.**

[中文文档](README.md) · [Latest Project Status](docs/project-status.md)

</div>
