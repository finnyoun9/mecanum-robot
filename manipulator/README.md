# LeArm Mobile Manipulator Lab

面向机器人/智能硬件嵌入式岗位的移动操作机器人项目。硬件主线确定为 **LeArm 散件 + STM32 核心模组**：先理解并验证机械结构，再完成总线舵机控制器，最后接入现有 Raspberry Pi 5 + STM32 麦克纳姆底盘。

## Project positioning

这不是复现厂商视觉、语音 Demo。项目要打通：

`LeArm mechanics -> bus servo protocol -> STM32/FreeRTOS -> host protocol -> ros2_control -> MoveIt 2 -> mobile manipulation`

主要证明：

- STM32 HAL、UART/DMA、FreeRTOS 和看门狗
- 多关节总线通信、标定、软限位及故障恢复
- Linux/MCU 分层控制和可测量的实时性能
- URDF/Xacro、轨迹执行、相机标定与移动抓取
- 使用逻辑分析仪、示波器和测试数据定位真实硬件问题

## Hardware decision

| Item | Decision | Reason |
| --- | --- | --- |
| Arm | LeArm 6-DOF kit | 全金属结构、智能总线舵机、适合上车 |
| Assembly | Loose-parts kit | 先理解连杆、轴系、舵机受力和线束路径 |
| Controller | STM32 module | 与目标岗位的 C/HAL、RTOS、DMA、调试能力最匹配 |
| High-level computer | Existing Raspberry Pi 5 | 运行 ROS 2 Jazzy、MoveIt 2 和视觉，不重复购买主控 |
| Camera | Existing CSI/USB camera | 第一版使用 AprilTag，暂不购买深度相机 |
| Mobile base | Existing `mecanum-robot` | 后期完成底盘与机械臂联动 |

51 模组不作为主线，因为资源和工程生态不足；ESP32 模组适合快速联网和 Arduino Demo，但会稀释当前 STM32/RTOS 求职主线。后续可将 ESP32 作为通信性能对照，不影响主控制器选择。

## Milestones

| Phase | Deliverable | Verification |
| --- | --- | --- |
| 0 | 锁定 SKU 和完整 BOM | 舵机型号、反馈项、协议、电源和源码全部确认 |
| 1 | 机械干装与结构测绘 | 逐轴运动无干涉，记录连杆尺寸、轴向间隙和线束路径 |
| 2 | 单舵机测试台 | 扫描 ID，读回状态，软限位和超时生效 |
| 3 | STM32/FreeRTOS 控制器 | DMA 通信、任务调度、看门狗和故障状态机可验证 |
| 4 | 桌面机械臂 | 6 轴标定，关节控制、急停和重复性测试完成 |
| 5 | ROS 2 接入 | RViz 状态与实机一致，轨迹可执行 |
| 6 | MoveIt 2 | 点到点抓取和碰撞规划生效 |
| 7 | 上车联动 | 底盘到位后完成抓取/放置，并统计成功率 |

## Repository layout

```text
docs/               # 选型、装配、架构、实验计划
firmware/           # STM32 HAL + FreeRTOS 控制器
hardware/           # 尺寸记录、安装板、电源和线束设计
ros2_ws/src/        # description、hardware、bringup、tasks
tools/              # 舵机诊断、标定和数据分析
experiments/        # 配置、原始数据、波形和结论
```

## Definition of done for v1

小车到达工作台前，通过相机定位带 AprilTag 的目标，机械臂抓取后由底盘运送到投放点并释放；连续运行 10 次，公开成功率、通信延迟、关节误差、电流数据和失败复盘。

## Current status

- [x] 项目定位与求职方向分析
- [x] 确定 LeArm 散件 + STM32 核心模组
- [ ] 向卖家确认 SKU、舵机反馈项和源码范围
- [ ] 到货清点与机械干装
- [ ] 单舵机 bring-up
- [ ] STM32/FreeRTOS 控制器
- [ ] ROS 2 / MoveIt 2
- [ ] 与 `mecanum-robot` 联调

## References

- [Hiwonder LeArm AI](https://www.hiwonder.com/products/learm-ai)
- [LeArm documentation](https://wiki.hiwonder.com/projects/LeArm_AI/en/latest/)
- [ros2_control Jazzy](https://control.ros.org/jazzy/)

License: MIT
