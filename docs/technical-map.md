# 技术地图 / Technical Map

按子系统分类的技术点速查：每个点的理论基础、在本仓库里的具体体现（文件/模块）、以及为什么这么做。配合 [README 技术地图图示](../README.md#技术地图) 使用；真机数据和证据边界以 [项目状态](project-status.md) 为准。

## 实时控制

| 技术点 | 理论基础 | 项目体现 | 用途 / 为什么这么做 |
| --- | --- | --- | --- |
| FreeRTOS 5 任务调度 | 抢占式优先级调度、时间片轮转 | [`firmware/Core/Src/main.c`](../firmware/Core/Src/main.c)：CtrlTask 100Hz/优先级4、CommTask 事件驱动/3、SensorTask/RemoteTask 2、MonitorTask 1Hz/1 | 控制环需要固定周期和低抖动；通信/传感器不能阻塞控制，用优先级分层保证 |
| PID / PI 速度闭环 | 比例积分控制、抗积分饱和（anti-windup） | [`firmware/Core/Src/pid.c`](../firmware/Core/Src/pid.c)：单轮 `Kp=100,Ki=300`；四轮前馈 + `Kp=15,Ki=35` + 零速消抖 | D 项会放大编码器量化噪声，故用 PI；积分上限按 `out_max/Ki` 推导防止饱和后超调 |
| 定时器复用 + 软件 EXTI 编码器 | 定时器 PWM 通道、外部中断（EXTI） | [`firmware/Core/Src/encoder.c`](../firmware/Core/Src/encoder.c)：TIM2/3/4 占用于四路 PWM 后，编码器改用 GPIO EXTI 双边沿解码，448 edges/圈 | 硬件资源冲突（TIM1 与 NRF24/USART1 冲突）时的降级方案；不是硬件正交编码模式 |
| 麦克纳姆运动学 | 正/逆运动学矩阵 | [`mecanum_kinematics.cpp`](../ros2_ws/src/mcr_bringup/src/mecanum_kinematics.cpp) | `cmd_vel`（vx/vy/w）与四轮目标转速之间互相转换 |

## 通信链路

| 技术点 | 理论基础 | 项目体现 | 用途 / 为什么这么做 |
| --- | --- | --- | --- |
| 自定义 UART 协议 + CRC16 | 帧同步、循环冗余校验 | [`shared/protocol.h`](../shared/protocol.h)：`[0xA5][0x5A][LEN][SEQ][CMD][PAYLOAD][CRC16]`，5 状态帧同步机 | 跨平台（STM32 C / Pi C++）共享一套协议，CRC 检错但不是加密认证 |
| DMA + 环形缓冲区 | DMA 搬运不占 CPU、环形缓冲避免拷贝竞争 | `main.c` 里 DMA 只写 staging buffer，`HAL_UARTEx_RxEventCallback` 搬入软件 ring | 921600 波特率下解析任务和 DMA 不会抢同一块内存；ODOM 约 50Hz，验收 CRC 错误为 0 |
| I2C 总线 | 7 位地址、主从时序、总线共享 | [`mpu6050.c`](../firmware/Core/Src/mpu6050.c) / [`tof_sensor.c`](../firmware/Core/Src/tof_sensor.c) / [`ssd1306.c`](../firmware/Core/Src/ssd1306.c) 共享 I2C2（PB10/PB11） | 一条总线复用多个低速传感器；OLED 丝印 `0x78` 是 8 位写地址，代码用 7 位 `0x3C` |
| SPI + NRF24 无线 | 全双工同步串行、无 ACK 机制 | [`nrf24l01.c`](../firmware/Core/Src/nrf24l01.c)：位翻转模拟 SPI | 2.4GHz 遥控；无 ACK，靠寄存器默认值（`STATUS=0x0E`）探针证明模块真的在响应 |

## 感知与状态估计

| 技术点 | 理论基础 | 项目体现 | 用途 / 为什么这么做 |
| --- | --- | --- | --- |
| Mahony 姿态解算（AHRS） | 互补滤波、四元数姿态表示 | [`ahrs.c`](../firmware/Core/Src/ahrs.c) | 6 轴 IMU（无磁力计）融合出 roll/pitch；yaw 没有绝对基准，会漂 |
| ToF 测距 | 飞行时间测距、I2C 寄存器窗口读取 | [`tof_sensor.c`](../firmware/Core/Src/tof_sensor.c) | 近距离避障/停车；曾因 burst 读取把低字节复制成高字节，改逐字节读取修复 |
| EKF 状态估计 | 扩展卡尔曼滤波、过程/观测协方差 | `ros2_ws/src/mcr_bringup` 的 `robot_localization` 配置 | 融合轮速里程计 + IMU，比单一里程计更平滑；协方差没调好会导致地图扭曲 |
| LD06 SLAM 建图 | 激光扫描匹配、占据栅格地图 | [`ros2_ws/src/ldlidar_stl_ros2`](../ros2_ws/src/ldlidar_stl_ros2) + SLAM Toolbox | 静止扫描稳定（点位标准差中位数约 8.1mm），当前地图扭曲主因是 gyro 零偏与打滑，不是雷达本体 |

## 系统与验证

| 技术点 | 理论基础 | 项目体现 | 用途 / 为什么这么做 |
| --- | --- | --- | --- |
| SIL 软件在环 | Mock/仿真替代硬件做逻辑验证 | [`firmware/Core/SIL/mocks/mock_hal.c`](../firmware/Core/SIL/mocks/mock_hal.c)：用 include 路径优先级影子化真实 HAL 头文件 | 固件源码零修改即可编译成 Linux 可执行文件，在 CI 里跑控制逻辑回归；不验证电气/机械时序 |
| GitHub Actions CI | 持续集成 | [`.github/workflows/`](../.github/workflows) | 每次 push 自动跑协议/PID/运动学/SIL 测试，CTest 12/12 |
| 安全状态机 + 多源仲裁 | 有限状态机、优先级仲裁、超时租约 | `main.c` / [`remote_control.c`](../firmware/Core/Src/remote_control.c)：K9 急停、250ms 通信看门狗、`remote_active` 仲裁 | 手柄和 Pi 同时下发目标速度时避免"最后写入者获胜"导致的失控 |
| 电源与 ADC 分压 | 分压公式、ADC 采样电容限制 | 设计见 [供电方案](power-system.md) | 3S 电池（100kΩ/27kΩ 分压）电量估算；代码与主机测试已完成，未改线烧录 |
