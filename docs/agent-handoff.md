# Agent 交接：真实底盘闭环

> 基线：2026-08-13。详细步骤见 [hardware-closed-loop-roadmap.md](hardware-closed-loop-roadmap.md)。

## 当前真实状态

- 共享协议、运动学、ROS 2 接口、协议模拟闭环和两套 SIL 已有代码。
- 独立机械臂仓库已通过 subtree 合入 `manipulator/`；后续只维护本仓库。
- `firmware/Core/Src/motor.c` 的四路 TIM/GPIO 仍为 `NULL` 占位，真机速度闭环尚未开始。
- UART DMA/IDLE、NRF24L01、IMU、ToF、Nav2 和机械臂舵机总线都不能按真机完成描述。

## 下一阶段只做三件事

1. M0：确认 MCU、电机驱动、编码器、供电、引脚和安全默认态。
2. M1：完成单轮开环，记录 PWM、rad/s、空载电流、死区和方向。
3. M2：完成单轮速度 PI，输出阶跃 CSV、超调、建立时间、稳态误差和 IAE。

四轮闭环与 Pi 5 HIL 必须等 M2 数据通过后再开始。

## 软件基线与已知问题

- 底盘 SIL 和机械臂 SIL 可构建，但 CMake 没有注册 CTest；当前需直接运行生成的可执行文件。
- 底盘 SIL 有 `xLastWakeTime` 未使用警告。
- 机械臂协议对 packed payload 成员直接取地址，有潜在非对齐访问警告，需要先复制到对齐缓冲区。
- 复用 `stm32-pid-balancer` 前，先修它的 BACKCALC `Kp=0` 除零问题并补测试。

## 简历红线

- `0.302 m/s` 是协议模拟器结果，不是真实底盘。
- 当前只能写“协议/运动学/SIL/CI 已验证，真机速度闭环进行中”。
- 不写“四轮 PID 已上板”“DMA/IDLE 已验证”或“机械臂已完成”。
