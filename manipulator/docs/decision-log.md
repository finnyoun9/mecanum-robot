# Decision log

## 2026-08-12 — Select LeArm loose-parts kit with STM32 controller

### Context

目标是为 2026 年深圳机器人/智能硬件嵌入式岗位补齐多轴执行器总线、RTOS、安全控制和 Linux/MCU 协同能力，同时复用现有 Pi 5 与麦克纳姆底盘。

### Decision

- 机械臂选择 LeArm 六自由度散件。
- 主控选择 STM32 核心模组。
- 第一阶段不购买 ESP32、51、视觉、语音和厂商底盘扩展。
- 先理解机械结构，再进行单舵机通电和固件开发。

### Rationale

- STM32 能直接展示 C、HAL、UART/DMA、中断、FreeRTOS、watchdog 和硬件调试。
- LeArm 智能总线舵机比 PWM 舵机更适合位置反馈、故障诊断和 ROS 2 接入。
- 散件装配可以补足机械结构、轴系、回差、线束和重心经验。
- Pi 5 负责 ROS 2/视觉，避免在 MCU 上堆与实时控制无关的应用功能。

### Risks

- 厂商“开源”范围和舵机反馈能力尚待卖家确认。
- 低成本舵机的回差、刚度和重复定位精度有限。
- 厂商 ROS/视觉示例不能替代自研控制链路。
