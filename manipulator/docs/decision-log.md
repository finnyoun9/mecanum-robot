# Decision log

## 2026-08-13 — Host protocol v1 + arm controller SIL skeleton

### Context

机械臂尚未到货，先把与硬件无关的主机协议和控制器核心确定下来，避免后续反复改线格式。

### Decision

- host protocol 沿用底盘 framing（`SYNC0/SYNC1 + LEN + SEQ + CMD + CRC16-MODBUS`），机械臂命令分配到 `0x40-0x4F`（Pi→arm MCU）/ `0x50-0x5F`（arm MCU→Pi），见 `shared/protocol.h`。
- 关节位置一律使用 radians，与 ros2_control/MoveIt2 一致；到舵机厂商单位（0-1000 或 0.1°）的换算放在 arm STM32 内完成。
- 控制器骨架落在 `firmware/arm_controller/`，结构与架构文档的 `app/`（arm_control、safety_monitor）+ `drivers/`（host_protocol、servo_bus）+ SIL 一致。
- 复用底座固件的 SIL 方法论：mock servo_bus，CI 里确定性跑 FAULT 状态机场景（`firmware-tests.yml` 新增 `sil_arm_controller --ci`）。
- 故障统一策略（骨架版）：**任何 FAULT 入口都断扭矩**，主机在 `CMD_ARM_RESET` 后需重新 `CMD_ARM_TORQUE` 上电。

### Rationale

- 一种 framing、一套 CRC 和单测，Pi 侧解析器与 SIL 基建全部复用，系统只有一个协议模型。
- radians 让 Pi 侧与 MoveIt2 零转换，MCU 负责厂商协议隔离。
- 在硬件到货前就能在 CI 里确定性验证限位、急停、超时、总线故障与恢复。

### Risks / Open

- 厂商舵机协议未确认，`servo_bus` 只有接口和 mock；真实驱动待 Stage 2 抓帧后补。
- `ARM_FAULT_OVER_TEMP / UNDER_VOLT / OVERLOAD / WATCHDOG` 已入协议，但骨架未接线。
- `PROTO_FRAME_OVERHEAD` 的 6→7 偏差已在 `9adae80` 修复；该提交当前仍需推送到远端。
- Windows host 构建仍报告 packed joint 数组地址可能未对齐；上板前改为复制到对齐的本地数组并启用 `-Werror`。

---

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
