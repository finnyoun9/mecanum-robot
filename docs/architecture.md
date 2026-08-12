# LeArm 系统架构

## 控制分层

```text
Raspberry Pi 5 / ROS 2 Jazzy
  Task state machine / MoveIt 2 / vision
                 |
          framed UART protocol
                 |
LeArm STM32 controller / FreeRTOS
  host_comm -> motion_command -> safety_monitor
                 |
          half-duplex servo bus
                 |
          six smart bus servos

Existing mecanum base:
Pi 5 -> existing UART protocol -> STM32/FreeRTOS -> DC motors
```

Pi 5 负责任务、规划、TF 和视觉；LeArm STM32 负责确定性的关节命令调度、反馈采集和安全状态；舵机内部闭环完成位置控制。机械臂与底盘分别保留独立 MCU，Pi 5 在 ROS 2 层统一编排。

## STM32 firmware boundary

预期模块：

```text
app/
  arm_control       # 目标关节、插值入口、状态机
  safety_monitor    # 限位、超时、过温/欠压、急停
drivers/
  servo_bus         # 帧编码、收发、校验、重试
  host_protocol     # Pi 5 与 MCU 的协议
platform/
  uart_dma
  gpio
  watchdog
```

硬件型号和舵机协议在资料确认前不写死。优先复用厂商原理图和可验证的初始化参数，但核心通信、调度和故障处理要能独立解释和测试。

## FreeRTOS tasks

| Task | Period/trigger | Responsibility |
| --- | --- | --- |
| `servo_bus_task` | RX notification / bus schedule | 处理总线收发，禁止在 ISR 内解析完整协议 |
| `motion_task` | `vTaskDelayUntil`, initial target 100 Hz | 更新关节目标和同步下发 |
| `safety_task` | initial target 50 Hz | 限位、通信超时、供电/温度和故障状态 |
| `host_comm_task` | RX notification | 解析 Pi 指令、回复状态和诊断信息 |
| `telemetry_task` | initial target 10 Hz | 输出调试指标、栈水位和错误计数 |

控制频率是初始目标，必须以实际协议带宽和逻辑分析仪结果校准。UART 接收使用 DMA/IDLE 或短 ISR + task notification；所有收发都必须有超时，不能无限阻塞。

## Safety states

```text
BOOT -> DISCOVERY -> CALIBRATION -> IDLE -> ACTIVE
                                  |        |
                                  +-> FAULT <-+
```

进入 `FAULT` 的最小条件：

- 主机命令超时
- 舵机连续多次无响应或校验失败
- 越过关节软限位
- 卖家协议支持时：过温、欠压或过载
- 急停输入有效
- 看门狗复位原因被检测到

故障策略需要区分 hold position 和 torque-off；具体选择要经过机械臂跌落/夹伤风险评估。

## ROS 2 packages

| Package | Responsibility |
| --- | --- |
| `learm_description` | URDF/Xacro、mesh、joint limits、车载安装位 |
| `learm_hardware` | 自定义 `ros2_control SystemInterface` |
| `learm_moveit_config` | planning group、collision、kinematics |
| `learm_bringup` | controller、TF、camera 和整机 launch |
| `mobile_manipulation_tasks` | dock、detect、pick、place 状态机 |

## Electrical boundary

- 桌面阶段先使用原配电源，不凭宣传图猜额定电压。
- 舵机支路配置保险和急停；Pi 5 保持独立供电，共地但避免舵机大电流走逻辑地细线。
- 测量整机静态、典型动作和短时堵转电流后，再选车载电池与 DC-DC。
- 上车前完成重心和倾覆测试，机械臂伸展时限制底盘加速度。

## Verification targets

- UART/servo bus：控制周期、P99 latency、checksum error、timeout 和 recovery time。
- RTOS：每个任务 stack high-water mark、最小剩余 heap、watchdog 验证。
- 机械：工作空间、关节回差、重复定位误差。
- 系统：连续 10 次抓取成功率和失败分类。
