# Robot Arm Mobile Manipulator Lab

面向嵌入式岗位的低成本移动操作机器人项目：先完成桌面 6-DOF 机械臂，再与 `mecanum-robot` 的 Raspberry Pi 5 + STM32 底盘联动，最后尝试视觉抓取和模仿学习。

## Why this project

这个项目不是单纯复现套件 Demo。目标是打通一条完整链路：

`bus servo -> hardware driver -> ros2_control -> MoveIt 2 -> camera calibration -> mobile manipulation -> LeRobot`

重点展示：

- 总线舵机通信、关节标定、限位与故障处理
- URDF/Xacro、正逆运动学、轨迹执行
- Raspberry Pi 5、ROS 2 Jazzy 与 STM32 的分层控制
- 机械臂与麦克纳姆底盘的坐标系和任务协同
- 可复现的测试数据、问题记录和演示视频

## Recommended hardware baseline

首选 **SO-101 follower arm（6 × Feetech STS3215）**。第一阶段只买 follower，不急着买 leader。详见 [采购清单](docs/buying-guide.md)。

## Milestones

| Phase | Deliverable | Verification |
| --- | --- | --- |
| 0 | 选型、BOM、风险表 | 每个部件有规格和购买理由 |
| 1 | 单舵机测试台 | 扫描 ID，读位置/温度/电压，限位生效 |
| 2 | 桌面机械臂 | 6 轴标定，关节控制和急停可用 |
| 3 | ROS 2 接入 | RViz 状态与实机一致，轨迹可执行 |
| 4 | MoveIt 2 | 完成点到点抓取，碰撞规划生效 |
| 5 | 上车联动 | 底盘到达目标后，机械臂完成抓取/放置 |
| 6 | 视觉抓取 | AprilTag 或固定物体定位，重复成功率有统计 |
| 7 | 可选 LeRobot | 采集数据，训练并复现实机策略 |

## Repository layout

```text
docs/               # 调研、采购、设计和实验记录
firmware/           # 可选 STM32 安全/IO/末端执行器控制
hardware/           # CAD、安装板、电源和线束设计
ros2_ws/src/        # description、hardware、bringup、tasks
tools/              # 舵机诊断、标定和数据分析脚本
experiments/        # 每次实验的配置、数据、结论
```

## Definition of done for v1

机器人从固定起点出发，到达工作台前，通过相机定位带 AprilTag 的目标，抓取后移动到投放点并释放；连续 10 次统计成功率，README 中附架构图、实机视频、问题复盘和测量数据。

## Current status

- [x] 技术路线和首版 BOM
- [ ] 下单前确认具体商品参数
- [ ] 单舵机 bring-up
- [ ] 机械臂装配与标定
- [ ] ROS 2 / MoveIt 2
- [ ] 与 `mecanum-robot` 联调

## References

- [Hugging Face LeRobot](https://github.com/huggingface/lerobot)
- [SO-ARM100 / SO-101 hardware](https://github.com/TheRobotStudio/SO-ARM100)
- [SO-101 documentation](https://huggingface.co/docs/lerobot/so101)
- [ros2_control Jazzy](https://control.ros.org/jazzy/)

License: MIT
