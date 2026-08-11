# 简历亮点 / Resume Highlights

本项目可作为嵌入式 MCU+RTOS 岗位简历的核心项目经历。以下是各模块的技术含金量分析和简历写法建议。

---

## 一、SIL 软件在环测试框架 ⭐⭐⭐ (最大亮点)

**技术含金量**：SIL（Software-in-the-Loop）是汽车/航空/工业控制领域的专业测试方法论。应届生/初级工程师简历上出现 SIL 极为罕见，能直接体现"工程化思维"和"测试意识"。

**你做了什么**：
- 设计了一套 Mock HAL 层，用**包含路径优先级**（`-I` 顺序）让 mock 头文件"影子化"真实 STM32 HAL 头文件，固件源码一行不改即可在 Linux 上编译
- Mock FreeRTOS 用轮询调度器模拟多任务：每个 tick 调用每个任务一次，`#ifndef SIL_BUILD` 条件编译移除 `for(;;)` 和阻塞调用
- 编码器物理模型：PWM 占空比 → 车轮转速 → 正交编码器边沿积分（含浮点累加器解决小占空比截断问题）
- 在 mock HAL 内闭环验证了：串口协议帧收发 → CommTask 解析 → `robot_handle_command()` → CtrlTask PID 运算 → PWM 输出 → 编码器累积 → 里程计帧发布
- CI 集成：GitHub Actions 自动编译 + 运行 SIL 测试

**简历写法**：
> 为 STM32 FreeRTOS 固件搭建 **SIL（软件在环）测试框架**：设计 Mock HAL 层（GPIO/TIM/UART/I2C）以包含路径优先级影子化真实外设头文件，固件源码零修改即可编译为 Linux 原生可执行文件；实现编码器物理模型（PWM→转速→正交边沿）和 FreeRTOS 轮询调度模拟器；在 CI 中自动验证 PID 闭环（PWM 输出 / 编码器累积 / 里程计协议帧）。

**英文版**：
> Built a **Software-in-the-Loop (SIL) test framework** for STM32 FreeRTOS firmware: designed a Mock HAL layer (GPIO/TIM/UART/I2C) that shadows real peripheral headers via include path ordering, allowing firmware to compile as a native Linux binary with zero source changes; implemented encoder physics model (PWM→speed→quadrature edges) and FreeRTOS round-robin scheduler simulator; automated PID closed-loop verification (PWM output / encoder accumulation / odometry frames) in CI.

**面试可能追问**：
- 为什么用 include 路径而非条件编译？→ 答：保持固件源码干净，mock 层完全独立
- setjmp/longjmp 方案为什么失败？→ 答：协程共享 C 栈，任务 B 执行会覆盖任务 A 的栈帧
- 编码器模型精度如何？→ 答：简化模型，仅验证闭环逻辑正确性，不替代硬件测试

---

## 二、自定义二进制通信协议 ⭐⭐⭐

**技术含金量**：从零设计通信协议并实现 CRC 校验，体现对数据链路层的理解。

**你做了什么**：
- 帧格式：`[0xA5][0x5A][LEN][SEQ][CMD][PAYLOAD...][CRC16 LE]`
- CRC-16/MODBUS（多项式 0x8005），查表法实现，校验覆盖 `[LEN..PAYLOAD_END]`
- CommTask 内含帧同步状态机（5 状态：WAIT_SYNC0 → WAIT_SYNC1 → READ_HEADER → READ_PAYLOAD → READ_CRC），抗数据错位
- 双向命令：Pi→STM32（`CMD_VEL_CTRL` 四轮速度 / `CMD_EMERGENCY_STOP` / `CMD_PID_TUNE`），STM32→Pi（`CMD_ODOM_FEEDBACK` 编码器+IMU+ToF / `CMD_ACK` / `CMD_ERROR`）
- 协议库 `shared/protocol.h` 同时在 STM32（C）和 Pi（C++ via `extern "C"`）上编译

**简历写法**：
> 设计 **自定义二进制串口通信协议**：双字节帧同步（0xA5 0x5A）+ CRC-16/MODBUS 校验 + 序列号防丢失；CommTask 内状态机解析器抗数据错位；协议库跨平台（STM32 C / Linux C++）共享。

**面试可能追问**：
- 为什么选 CRC-16/MODBUS？→ 答：工业标准，16 位多项式足够短帧，查表法快
- 帧同步丢了怎么恢复？→ 答：状态机在任意状态收到 0xA5 都重新开始同步

---

## 三、FreeRTOS 多任务实时架构 ⭐⭐

**技术含金量**：5 任务优先级分配 + 任务间通信，展现 RTOS 设计能力。

**你做了什么**：
| 任务 | 频率 | 优先级 | 职责 |
|------|------|--------|------|
| CtrlTask | 100 Hz | 4 (最高) | 4 路 PID 速度闭环 + 里程计发布 (50Hz) + 通信超时看门狗 |
| CommTask | 事件驱动 | 3 | DMA 串口接收 + 协议帧解析 + 命令分发 |
| SensorTask | 100 Hz / 20 Hz | 2 | IMU 读数 + Mahony 姿态解算 + ToF 测距 |
| RemoteTask | 20 Hz | 2 | NRF24L01 无线遥控接收 + 急停/使能逻辑 |
| MonitorTask | 1 Hz | 1 (最低) | 栈高水位监控 + 心跳 |

- 任务间通信：`xQueueCreate` / `xSemaphoreCreateBinary` 传递命令与同步
- 通信超时看门狗：100ms 无指令自动急停
- ToF 紧急避障：<10cm 自动刹车

**简历写法**：
> 基于 FreeRTOS 实现 **5 任务实时调度系统**：100Hz PID 控制任务（优先级 4）+ DMA 串口通信任务（优先级 3）+ 传感器融合任务（100Hz IMU + 20Hz ToF）+ 无线遥控任务 + 栈监控任务；任务间通过队列和信号量通信。

---

## 四、PID 速度闭环控制 ⭐⭐

**技术含金量**：经典控制理论在嵌入式上的实现，含抗积分饱和。

**你做了什么**：
- 4 路独立 PID（增量式/位置式），100 Hz 控制频率
- 抗积分饱和（integral windup clamping），积分上限 300
- 默认参数 kP=2.5, kI=0.8, kD=0.05（可在 SIL 中调参验证）
- 支持运行时通过串口 `CMD_PID_TUNE` 调整增益
- 编码器速度反馈：正交编码器 → 边沿计数 → 弧度/秒转换

**简历写法**：
> 实现 **4 路独立 PID 速度闭环控制**（100 Hz），含抗积分饱和和运行时增益调整；通过正交编码器提供速度反馈。

---

## 五、CI/CD 自动化测试 ⭐⭐

**技术含金量**：嵌入式项目做 CI 的不多，体现 DevOps 意识。

**你做了什么**：
- GitHub Actions 自动运行：协议 CRC 单测 / PID 单测 / AHRS 滤波单测 / 麦克纳姆轮运动学单测 / 遥控协议单测 / **SIL 固件闭环测试**
- SIL 步骤：`cmake .. && make -j$(nproc) && ./sil_firmware --ci`
- 每次 push 自动验证固件不退化

**简历写法**：
> 搭建 GitHub Actions **CI 流水线**，每次提交自动运行协议/PID/运动学单元测试 + **SIL 固件软件在环闭环测试**，确保固件变更不引入回归。

---

## 六、其他技术点

| 模块 | 简历关键词 | 适合强调的场景 |
|------|-----------|---------------|
| Mahony AHRS 姿态解算 | 6 轴 IMU 融合，四元数姿态估计 | 面无人机/平衡车岗位 |
| 麦克纳姆轮运动学 | 正/逆运动学解算，全向移动控制 | 面移动机器人岗位 |
| NRF24L01 无线通信 | 2.4GHz SPI 驱动，位脉冲模拟 | 面 IoT/无线通信岗位 |
| ros2_control 硬件接口 | 自定义 SystemInterface 插件 | 面 ROS2 岗位 |
| 编码器驱动 | STM32 TIM 正交编码器模式，16→32 位扩展 | 面电机控制岗位 |
| UART DMA 收发 | IDLE 中断 + 双缓冲，零 CPU 占用 | 面底层驱动岗位 |

---

## 简历项目描述模板

### 精简版（3-4 行，适合空间有限的简历）

> **麦克纳姆轮全向移动机器人** | STM32 + FreeRTOS + ROS2 | 独立开发
> - 搭建 **SIL 软件在环测试框架**：Mock HAL 层使固件以 Linux 原生可执行文件编译，在 CI 中自动验证 PID 闭环
> - 实现 **FreeRTOS 5 任务实时调度**（100Hz PID 控制 + DMA 串口通信 + IMU/ToF 传感器融合 + 无线遥控）
> - 设计 **自定义二进制通信协议**（双字节帧同步 + CRC-16/MODBUS + 状态机解析器），跨平台 C/C++ 共享
> - 构建 **GitHub Actions CI 流水线**，每次提交自动运行单元测试 + SIL 闭环验证

### 详细版（5-6 行，适合项目经历专页）

> **麦克纳姆轮全向移动机器人 — 嵌入式实时控制系统**
> - 为 STM32 FreeRTOS 固件搭建 **SIL 软件在环测试框架**：设计 Mock HAL 层（GPIO/TIM/UART/I2C）以包含路径优先级影子化真实外设头文件，固件源码零修改编译为 Linux 原生可执行文件；实现编码器物理模型（PWM→转速→边沿）和 FreeRTOS 轮询调度模拟器；CI 中自动验证 PID 闭环（PWM 输出/编码器累积/里程计协议帧）
> - 实现 **5 任务 FreeRTOS 实时调度**：CtrlTask（100Hz PID + 看门狗）> CommTask（DMA 串口 + 协议解析）> SensorTask（IMU + ToF）> RemoteTask（NRF24L01 无线）> MonitorTask（栈监控）
> - 设计 **自定义二进制串口协议**：`[SYNC0 SYNC1 LEN SEQ CMD PAYLOAD CRC16]`，CRC-16/MODBUS 校验，5 状态帧同步状态机抗数据错位，协议库跨 STM32（C）和 Raspberry Pi（C++）共享
> - 4 路独立 PID 速度闭环（100 Hz，抗积分饱和），正交编码器反馈，支持运行时增益调整
> - CI/CD：GitHub Actions 自动运行协议/PID/运动学单元测试 + SIL 闭环验证

---

## 面试准备建议

面试官看到 SIL 框架大概率会问：

1. **"SIL 和 HIL 的区别是什么？"** → SIL 纯软件仿真，HIL 接真实硬件。你们目前是 SIL，后续可以在 Pi 上接真实 STM32 变成 HIL。

2. **"Mock 层怎么保证和真实硬件行为一致？"** → 不保证完全一致。SIL 验证的是**控制逻辑正确性**（PID 是否产生输出、协议帧是否正确编码解码），硬件时序/电气特性需要 HIL 或真机测试。

3. **"你用 SIL 发现了什么 bug？"** → 可以讲这次开发中发现的 bug：`encoder_init()` 清零 htim 指针、`huart1` 与 `mock_uart1` 分叉导致里程计帧丢失、`proto_encode` 返回值判断错误 —— 这些在真机上都是难以定位的静默 bug。

4. **"为什么不用 QEMU 模拟 STM32？"** → QEMU 模拟外设寄存器层级，太底层，开发慢。我们的 Mock HAL 在应用层 API 级别模拟，更轻量，更适合验证应用逻辑。

5. **"编码器模型为什么用浮点累加器？"** → 小占空比下每个 tick 产生不到 1 个边沿（如 64/1000 → 0.017 边沿/tick），直接截断为 `int16_t` 永远不积累；浮点累加器保留小数部分，跨 tick 累积后整数部分才写入 `tim->cnt`。
