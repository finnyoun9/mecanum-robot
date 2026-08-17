# Mecanum Mobile Robot / 麦克纳姆轮移动机器人

> **STM32 + FreeRTOS + Raspberry Pi 5 + ROS 2 Jazzy**｜当前处于硬件集成阶段

这是一个面向嵌入式机器人底层控制的学习与工程实践项目。目标是让 Raspberry Pi 5 上的 ROS 2 与 STM32 实时控制器形成可测量的闭环，而不是在未完成底盘验证前宣称已具备导航、视觉或移动操作能力。

## Current evidence / 当前已验证

- Raspberry Pi 5 上的 Ubuntu 24.04 arm64 Docker + ROS 2 Jazzy 开发环境可构建。
- 自定义 `ros2_control` `SystemInterface`、URDF 和麦克纳姆轮正/逆运动学已通过软件构建与测试。
- STM32/Pi 共享的 UART 二进制协议包含帧同步、序列号和 CRC-16/MODBUS 校验。
- STM32 FreeRTOS 固件具备 PID、编码器、协议解析和电机控制框架；Mock HAL + SIL 在 CI 中验证控制数据链。
- RViz2 中的 `/cmd_vel` → 协议模拟器 → `/odom` 是**软件/协议模拟结果**，不是实车里程计。

## Not verified yet / 尚未实机验证

- 底盘尚未组装；没有真实电机、编码器、UART DMA/IDLE 或四轮 PID 闭环结果。
- NRF24L01、IMU、ToF、Nav2、SLAM、视觉和机械臂均不是当前 Demo 范围。
- 本仓库的目标架构与软件骨架保留，但只有下文 MVP 完成后才可写成实机能力。

## MVP: one-wheel speed-control demo / 最小可演示版本

**MVP 的完成定义：** 一块 STM32、一块 TB6612FNG、一个带编码器减速电机，在限流电源下完成正反转、编码器测速、100 Hz 速度 PI 和通信超时急停；保存串口 CSV、示波器/逻辑分析仪截图与一段实拍视频。

这比先做 ROS 2 导航、视觉或机械臂更重要。单轮闭环跑稳后，复制到四轮才有意义。

| 优先级 | 阶段 | 只做什么 | 产出 / 验收 |
| --- | --- | --- |
| P0 | 底盘与电源基线 | 组装底盘；确认电机、减速比、编码器线数、轮径、TB6612 引脚和电源共地 | 接线图、BOM、限流电源下的静态电流记录 |
| P1 | 单电机开环 | STM32 PWM 驱动一块 TB6612；依次测试 `+20% / 0 / -20%` | 每个方向可重复启停；电机堵转或通信丢失时立即停机 |
| P2 | 单轮测速与 PI | 定时器编码器模式，100 Hz 固定周期；先 P 后 PI | 三档阶跃的 CSV：目标速度、实测速度、PWM、周期 jitter；上升时间/超调/稳态误差 |
| P3 | Pi 5 ↔ STM32 | 将现有协议接到真实 UART；Pi 发目标速度，STM32 回传实际轮速 | `/cmd_vel` → 真实 PWM → 编码器 → `/odom` 单轮证据 |
| P4 | 四轮与麦克纳姆运动学 | 一轮一轮复制 P1/P2，再验证前进、横移和原地旋转 | 30 分钟无失控；三种基本运动实拍与日志 |

P0–P2 就是第一个对外 Demo；P3–P4 是第二阶段。**P4 前不接 Nav2、SLAM、相机、ToF、遥控器或机械臂。**

## Safe first hardware session / 第一次上电

1. 只装电源与一块电机驱动，不接 Raspberry Pi；确认电机电源、逻辑电源和 GND 共地。
2. 使用限流电源；先测 3.3 V/电机电源、电流和发热，再连接 STM32。
3. 烧录一个只输出固定低占空比 PWM 的程序；确认急停默认关闭和正反转方向。
4. 最后才接编码器 A/B 相，用逻辑分析仪确认相位、方向与计数倍频。

每次实机结果都记录板卡、固件 commit、仪器、接线、测量值和结论。SIL 是验证控制逻辑的工具，不能代替电气与机械验证。

## Software structure / 软件结构

```text
firmware/       STM32 FreeRTOS 控制框架、SIL 与 host tests
shared/         STM32 / Raspberry Pi 共享 UART 协议库
ros2_ws/        ROS 2 bringup、description、navigation 骨架
manipulator/    独立机械臂探索（不属于当前 MVP）
docs/           硬件路线、接线、软件与测试记录
```

## Development environment / 开发环境

| 层 | 当前方案 |
| --- | --- |
| 开发机 | macOS（编辑、Git、代码审查） |
| 机器人主机 | Raspberry Pi 5，保留 Raspberry Pi OS |
| ROS 2 | Ubuntu 24.04 arm64 Docker 中运行 ROS 2 Jazzy |
| MCU | STM32 + HAL + FreeRTOS（真实引脚映射待 P1 完成） |

ROS 2 和固件的软件入口、SIL 细节及后续记录见 [docs/](docs/)。面试可强调的软件证据与边界见 [docs/resume-highlights.md](docs/resume-highlights.md)。

## Future scope / 后续范围

只有 P4 完成后，才按顺序接入 IMU/ToF、真实 `ros2_control` 里程计、再考虑 SLAM/Nav2。机械臂保持独立子系统，等底盘稳定后再做单舵机 → 六轴 → ROS 2 联动。
