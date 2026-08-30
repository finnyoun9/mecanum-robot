# 项目状态与协作基线

> **最后更新：2026-08-29。** 改动了真机状态、固件结构或标定常数之后，**请同步更新本文档**——多人或多端并行推进时，过时的基线会让其他开发者基于错误前提做出判断。

## 协作机制

不同开发端之间以 **git 仓库 + 本文档**作为统一协调基线。因此：

- 开工前先 `git pull`，读本文档确认当前基线。
- 完成一段工作后**及时 push**，并更新本文档中被你改变的事实。
- 不确定某个结论是"已验证"还是"假设"时，看下方"证据分级"，不要默认它已验证。

### 硬件是独占资源

**ST-Link + 真车同一时间只能由一个人操作。** 烧录、GDB 读写、电机上电都会互相打断（`st-util` 连接时会复位芯片）。

- **需要真机的任务**：由当前持有硬件的一方独占，另一方不要碰 `firmware/Core/HW/` 的烧录流程。
- **不需要真机的任务**可以并行：ROS 2 侧（`ros2_ws/`）、协议模拟器、主机单元测试、文档、Pi 侧集成。

当前硬件由真机调试端占用。旧车端 Blue Pill 的 `NRST` 故障已由新板替换；新板的 SWD、
Flash 写入和 `remote_pid_drive` 启动已验证。旧板故障的完整实测链仍见
[power-system.md](power-system.md#故障记录车端-blue-pill-复位节点异常2026-08-26)。

### 证据分级

写结论时区分清楚，这是本项目反复踩过的坑：

| 级别 | 含义 | 例子 |
| --- | --- | --- |
| **实测** | 真机测量，有数据 | A 相双边沿解码 448 edges/圈 |
| **SIL/单测验证** | 逻辑正确，非物理验证 | PID 数据链、协议编解码 |
| **推导/估计** | 由其他数据算出，未验证 | SIL 里的 `max_wheel_rps 4.27` |
| **假设** | 没有依据 | —— 不要写进文档 |

两次真实教训：`EDGES_PER_WHEEL_REV` 按厂商典型值推导，错了 6.7 倍；台面限流电源的 6V 被当成电机额定电压，导出一整套错误的供电架构（电机实为 12V 标称）。

---

## 当前真实状态（2026-08-27）

### 旧车端硬件故障（2026-08-26，实测）

- **车端 NRF24L01+ 已下线，待更换。** 只接该模块 `VCC` 与 `GND` 时，车端 3.3 V
  从正常值下跌到 2.x V、继而约 1.x V，PC13 熄灭；模块芯片明显发热，且模块 pin 1
  (`GND`) 与 pin 2 (`VCC`) 被万用表测得导通。该模块在故障排查中不得再接入新板。
  这是模块电源端异常的**实测症状**；它是否也是后续车板故障的最初电气原因尚未证实。
- **车端 Blue Pill 复位节点异常，暂缓维修并等待替换板。** 同一 ST-Link、同一组
  SWD 线可读取手柄板，车板却在 100 kHz 热插拔及复位保持连接时稳定报
  `chipid 0x000`。`BOOT0=3.3 V` 后经 Pi 的 `PA9/PA10` 串口以
  115200/57600/38400/19200/9600 尝试系统 Bootloader，均超时。
  最终在**只保留 Micro-USB 供电**时测得 3.3 V 主轨稳定、`RES/NRST` 对 GND 仅
  0.47 V（按下 Reset 为 0 V、松开仍为 0.47 V）；以 1 kΩ 从 3.3 V 外拉后也仅
  0.68 V。故障结论是复位节点被强力拉低，MCU 无法正常出复位；尚未定位到按键、
  复位电容/上拉网络或 MCU `NRST` 引脚中的具体故障件，**不能写成“MCU 已确认烧毁”**。
- 上述故障发生前已经验收的遥控和 M3 闭环结果仍是历史真机验证记录；新板已完成
  SWD 与烧录自检，但电机、编码器和无线链路仍须逐项复验。

### 新车端板与 NRF24 更换排障（2026-08-27，实测）

- **替换 Blue Pill 可用。** ST-Link V2J27S6 在 1 MHz、4 MHz SWD 下均可稳定识别
  Cortex-M3；目标电压约 3.22 V，设备报告 Flash 128 KiB，读写保护关闭。已编译并烧录
  `remote_pid_drive`（Flash `9196 B`，RAM `2248 B`），OpenOCD `verify` 通过并复位运行。
  该镜像启动时关闭 TB6612 `STBY`，没有收到有效遥控包不会驱动电机。
- **历史尝试（已由 2026-08-29 换模块后复测解除）：旧替换模块未建立 SPI 回读。** 模块 `VCC/GND` 测得 3.3 V，模块旁已并联
  220 µF 电解。通过不初始化任何电机 GPIO 的临时探针两次读取 `STATUS`、`CONFIG`、
  `RF_CH`、`SETUP_AW`，均为 `0x00`；正常模块应返回非零默认/配置值。重新拔插所有连线后
  结果不变。探针完成后已恢复正式 `remote_pid_drive` 镜像并再次 `verify`。
- **示波器证据与寄存器读数一致。** `CSN/PA15` 存在车端访问脉冲，但在 `MISO/PB4` 上看到的是
  RC 式缓慢爬升而非随 SCK 的 0/3.3 V 数字跳变，说明模块未主动驱动 MISO。手柄开机时
  车端 `rx_packets=0`，手柄 `Sig=0` 表示未获得车端 Auto-ACK；这不是蓝牙式“配对码”状态。
  现阶段应继续用万用表确认模块 `MISO` 排针根部至 Blue Pill `PB4` 的实际连通性及排针方向；
  若连通无误，再判定模块本体异常。不能写成“手柄固件已经确认有问题”。
- **接线已复核，与探针假设一致（实测）。** 车端实际接线为
  `CE=PA8`、`CSN=PA15`、`SCK=PB3`、`MISO=PB4`、`MOSI=PB5`、`IRQ` 不接（轮询接收）。
  排查早期一度把 SCK/MOSI 误记为 `PA13/PA14`（实为 SWD 引脚），已纠正；
  后续文档与探针必须以 `PB3/PB4/PB5` 为准。
- **替换板 SW-DP ID 与旧板不同（实测）。** `Core/HW/stlink_stm32f1.cfg` 中写死的
  `CPUTAPID 0x2ba01477` 在新板上不匹配，OpenOCD 因此拒绝连接；新板读到标准
  `0x1ba01477`。已新增 `Core/HW/stlink_new.cfg`（直接 source `interface/stlink.cfg`
  + `target/stm32f1x.cfg`，不锁 TAP ID）用于替换板烧录，实测 `program ... verify reset`
  通过。这是**板间差异**，不是 MCU 故障。
- **`MISO/PB4` 被拉低的位置已缩到模块侧（实测，尚未定性）。** 新增引脚级探针
  `Core/HW/nrf24_spi_probe_main.c`（USART1/PA9 @115200 文本输出 + 全局变量供 GDB 读取，
  含快速与 ~100 kHz 慢速两次 `STATUS` 读取）。模块接 `VCC` 时，`PB4` 配上拉仍读到 0、
  快速读 `STATUS` 仍为 `0x00`；**拔掉模块 `VCC` 后同一固件的 `PB4` 上拉恢复读 1**。
  据此可判定 MCU 侧 `PB4` 与 JTAG remap 正常，问题在模块侧或模块上电后的 MISO 行为，
  但**尚不能断定模块已损坏**——上电即输出低亦可见于部分 SI24R1/带 PA 兼容模块。
- **当前接入模块复测（2026-08-29，实测）排除了“PB4 被硬拉低”。** 以同一无电机
  `nrf24_spi_probe` 重复读取，`MISO` 内部上拉读值为 `1`，但快 SPI 和约 100 kHz 慢 SPI
  的 `STATUS` 均为 `0x00`（正常 NRF24L01+ 复位状态应为 `0x0E`）。因此软件时序过快不是
  原因，MCU 的 PB4 也没有被占用；故障范围为模块本体、模块排针方向，或 `CSN/PA15`、
  `SCK/PB3`、`MOSI/PB5` 任一条的实际连通性。探针后已恢复
  `rtos_drive RTOS=1 TOF=1 OLED=1`，ST-Link verify 通过。
- **替换后的新普通 NRF24L01+ 已通过 SPI 健康检查（2026-08-29，实测）。** 相同探针得到
  `MISO` 上拉为 `1`，快速和约 100 kHz 慢速 `STATUS` 均为正常复位值 `0x0E`。这同时验证
  3.3 V 供电、模块方向、`CE/CSN/SCK/MISO/MOSI` 接线和 STM32 位翻转 SPI；旧模块应继续
  下线。正式 `rtos_drive` 已恢复，但尚待手柄开机后的 Auto-ACK/收包验收。
- **手柄固件已定向重刷（2026-08-29，实测）。** 两只 ST-Link 同时接入时，按序列号
  `37FF71064E57343602361C43`（手柄、V2J27S6）写入
  `firmware/remote_controller/remote-control.bin`，41,184 B，写入校验通过；车端
  `37FF71064E57343623CE1E43` 未被本次操作触及。该结果仅证明烧录完成，仍待手柄 OLED
  启动和与车端的 Auto-ACK/收包验收。
- **历史阻断（已由 2026-08-29 GDB 复测解除）：慢速 SPI 读数曾未取到。** 探针跑到后段时 PC13 观察到持续慢闪，
  与预期的阶段化闪烁不符，怀疑卡在某阶段或存在探针自身逻辑缺陷；本机 `st-info` 缺失、
  `arm-none-eabi-gdb` 不可用（`exit 127`），OpenOCD `mdw` 亦未回读到 stdout，
  因此当时慢速 `STATUS` 值没有可信读数。现已通过 ST-Link/GDB 读取到新模块快/慢
  `STATUS=0x0E`，无需再以该历史症状作为换模块依据。

### Pi 侧传感器（2026-08-27，实测）

- **LD06 + CH340 USB-TTL 链路可用。** 最终接法为 `P5V→Pi 5V`、`GND→USB-TTL GND`、
  `DATA/TX→USB-TTL RXD`、`CTL` 悬空；USB-TTL 的 `TXD/VCC` 不接。适配器枚举为
  `/dev/ttyUSB0`，稳定路径为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
  230400 8N1 连续读取 3 秒得到 53504 bytes、1139 个 `54 2c` 帧头，相邻 1138 个帧间距
  全部为 47 bytes。官方 `ldlidar_stl_ros2` 的 LD06 配置实测 `/scan` 稳定 10.00 Hz，
  量程字段 0.02–25 m；`laser_geometry` 转换后的 `/ld06/points` 也为 10 Hz，抽样一帧
  421 个有效点、frame 为 `base_laser`。这是平面二维点云（z=0），不是 3D 点云。
- **LD06 已正式接入仓库 bringup（2026-08-27）。** `ldlidar_stl_ros2` 以 git submodule
  形式纳入 `ros2_ws/src/`（pinned 到上游 `bf668a8`，HTTPS URL，CI 自动递归 checkout）。
  `robot.launch.py` 的雷达节点已取消注释并改用 LD06 参数（`product_name: LDLiDAR_LD06`、
  `port_name: /dev/ttyUSB0`、`port_baudrate: 230400`、`frame_id: laser_link`）；
  `laser_link`/`laser_joint` 在 `mcr.urdf.xacro` 中已存在（parent `base_link`，z=0.10 m），
  故 `base_footprint → base_link → laser_link` TF 链由 robot_state_publisher 统一发布，
  **不**再起上游 `ld06.launch.py` 自带的 `base_laser` static transform（会与 URDF 冲突）。
- **GCC 13 编译修复走本地 patch。** 上游 `log_module.cpp` 调用 `pthread_mutex_*` 却未
  `#include <pthread.h>`，GCC 13 不再传递性引入，编译报 "not declared in this scope"。
  用 `docker/patches/ldlidar-include-pthread.patch` 在 colcon build 前由
  `docker/apply-patches.sh` 幂等应用（优先 `git apply`，回退 `patch --forward --batch`，
  已应用则跳过、不反向）；CI workflow 安装 `patch` 并在 build 前跑该脚本。
- **CI `colcon test` 已限定到自有包（2026-08-27 修复）。** submodule 引入后，
  `colcon test` 会把上游 `ldlidar_stl_ros2` 的 `ament_uncrustify`/`cpplint` 一起跑，
  实测 `729 tests / 695 failures`，全部来自上游 `ldlidar_driver/**` 与 `src/demo.cpp`
  的既有代码风格，与本仓库改动无关。已改为
  `colcon test --packages-select mcr_bringup mcr_description mcr_navigation`；
  `colcon build` 仍构建全部包。**不要**为了过 CI 去改上游格式。
- **实测验证（2026-08-27，容器内）**：`ldlidar_stl_ros2` + `robot_state_publisher` 起来后，
  `/scan` 实测 **9.77 Hz**（≈10 Hz），`frame_id: laser_link`。GPIO14/15 的
  `/dev/ttyAMA0` 留给 STM32，雷达走 CH340 `/dev/ttyUSB0` 不再抢占。
  **SLAM（`/scan` + TF → `/map`）已跑通**，见下方 M5。

### 堵转保护（2026-08-29，SIL/单测验证，真机未验收）

**动机：此前固件对"被大占空比驱动但不转"的轮子没有任何处理。** `ERR_MOTOR_FAULT`
（`0x03`）在协议里定义了却**从未被任何代码置位**。一旦轮子卡死、线松脱或电机失效，
PI 的积分会冲到 `PID_INTEGRAL_MAX`、输出钉在满占空比硬怼一个不转的电机。
JGA25-370 堵转电流约为空载的 5–8 倍，而 3S 电池无限流、
**采购清单里的 10 A 保险丝至今未装**，这足以烧毁绕组。

2026-08-29 当天这个场景**真实发生过两次**：RR 电机失效时、以及 RR 输出线松脱时
（松脱期间闭环测量恒为 0，固件即在满占空比硬怼）。

- **实现**（`robot_control.c`）：每轮独立累计"输出 ≥ `STALL_DUTY_MIN` 且转速
  < `STALL_SPEED_MAX`"的持续时间，达 `STALL_TRIP_MS` 即**只切断该轮**输出、
  置 `ERR_MOTOR_FAULT`、`pid_reset()`；其余三轮继续正常工作。
- **阈值取自实测**：`STALL_DUTY_MIN=150`、`STALL_SPEED_MAX=1.0 rad/s`、
  `STALL_TRIP_MS=500`。占空比下限高于实测 5–10% 启动死区（5% 完全不转、
  10% 为 2.0 rad/s），而故障 RR 电机在 80% 占空比下仅 0.49 rad/s——两者相差一个
  数量级，判据不含糊。
- ⚠️ **`STALL_DUTY_MIN` 不能设更高**：`PID_*_DEFAULT` 这组占位增益
  （`Kp=2.5/Ki=0.8`）的输出上限是 `Kp·err + Ki·integral_max = 260`，
  阈值设 300 会让保护在默认增益下**静默失效**，却仍向堵转电机灌 26% 占空比。
  Pi 实际下发的 M2 增益（`Kp=100/Ki=300`）上限才是 1000。开发时踩过这个坑。
- ⚠️ **堵转闩锁不被 `robot_emergency_clear()` 释放**，新增
  `robot_clear_motor_fault()` 专门释放。原因：deadman 恢复路径在**每个**有效运动
  指令上都调用 `robot_emergency_clear()`，若共用会导致每次通信抖动后几百毫秒就
  重新驱动坏电机——保护形同虚设。测试 `test_deadman_recovery_does_not_release_stall`
  锁死这个语义。
- ⚠️ **`ERR_*` 是顺序枚举值，不是互斥位**：`ERR_MOTOR_FAULT`（`0x03` = `0b011`）
  与 `ERR_CRC_MISMATCH`（`0b001`）、`ERR_UNKNOWN_CMD`（`0b010`）**位重叠**，
  尽管 `protocol.h` 把该字段注释为 "Bitmask of active errors" 且既有代码在用
  `|=` / `&= ~`。本次实现改为以 `stalled_mask` 为权威闩锁、不对这些值做位运算。
  **这是既有的协议设计隐患，本次未改动协议**，后续若要真当位掩码用需重新分配取值。
- ⚠️ **`dt_ctrl` 不能用来累计时间**：`last_ctrl_ms` 是初值为 0 的 static，
  **首次调用的 dt 等于整个绝对 tick 计数**（实测 1011 ms），会瞬间越过 500 ms 窗口。
  已改为按固定标称 tick 累计（即窗口以控制周期数衡量）。这个既有缺陷同样使
  首次 `encoder_get_speed_rads()` 读数失真。
- **验证**：新增 `firmware/Core/Test/test_stall.c` **8 项全通过**，覆盖单轮触发不影响
  其他三轮、闩锁保持、deadman 不释放、显式清除后恢复、正常启动瞬态不误触发、
  "慢但在转"不算堵转、零指令不算堵转、反向同样触发。
  **CTest 由 6/6 增至 7/7 全通过**；`rtos_drive` 真机目标 `-Wall -Werror` 编译通过。
- ❌ **真机未验收。** 以上全部为主机单测；换上新 RR 电机后需在真车上验证
  （建议先人为堵住一个轮子确认 500 ms 内切断且只切该轮）。

### 新电机驱动板换装后四轮复测（2026-08-29，实测，空载架空 + 3S 电池）

换上同款裸引脚 TB6612 ×2 后用 `tools/drive_check.py --spin` 与临时单轮探针复测。
**链路与三轮驱动正常，RR 单通道确认偏弱且不稳定。**

- **前置只读检查全绿**：`link_check.py` 15 s → 有效帧 769、CRC 失败 0、ODOM 50.2 Hz、
  ACK 15/15；`drive_check.py` 被动相位静止编码器漂移 `(0,0,0,0)`。
  （工具那条 `outside 15-25 Hz` 警告是其默认按 20 Hz probe 提示，`rtos_drive` 设计值
  为 50 Hz，非故障。）
- **✅ 四轮"目标为正 → 计数为正"符号一致性已验证**，这解掉了 M4 留下的待验项。
  四轮同跑 `+3.0 rad/s` 时四路计数均为正，`error_flags=0x00`，未触发编码器故障。
- **四轮同跑一致性（`Kp=100 / Ki=300`，各档 3 s）**：

  | 目标 rad/s | FL | FR | RL | RR | RR 缺口 |
  | --- | --- | --- | --- | --- | --- |
  | 2.0 | 1.89 | 1.90 | 1.77 | **1.29** | −32% |
  | 3.0 | 2.86 | 2.86 | 2.71 | 2.77 | −3% |
  | 4.0 | 3.83 | 3.83 | 3.47 | **2.43** | −37% |
  | 6.0 | 5.74 | 5.75 | 5.74 | **4.22–5.01**（三次） | −13%…−27% |
  | 8.0 | 7.56 | 7.65 | 7.69 | **6.31** | −18% |

- **单轮独占探针（每轮单独给 6.0 rad/s，其余三轮目标 0）**：
  FL `0.96`、FR `0.96`、RL `0.96`、**RR `0.86`**。前三轮一致到小数点后两位；
  串扰计数为 0（唯一例外是当批第一次上电时 FL/FR 各 −57 edges，判为上电抖动）。
- **RR 单跑各档 ratio：** 2.0 → `0.44`、4.0 → `0.85`、6.0 → `0.86`、8.0 → `0.74`；
  **实测转速在 6→8 档间封顶约 5.9 rad/s**，加大目标不再增加输出，而同档其他三轮可达
  7.56–7.69。整条曲线被压低（低速段 ratio 也只有 0.44），是**驱动能力饱和**特征。
- **判据：PI 消不掉这个误差。** `Ki=300` 下 RR 稳态仍差 30%，说明该通道已到控制量上限
  仍达不到目标转速，**不是调参问题**。
- **已排除的原因**：机械阻力（断电手转 RR 与其他三轮手感无差异）、电源限流
  （3S 电池供电，非台面限流电源）、四轮共载压降（RR 单独独占整块板与电池时仍为 0.86）。
- **带不稳定成分**：6.0 档三次连测由 5.01 → 4.70 → 4.22 单调恶化，且 3.0 档曾出现过一次
  正常值（2.77，ratio 0.92）。更像接触不良或器件发热退化，不像固定的元件参数差异。
- ⚠️ **`drive_check.py` 的 `ok` 判据（ratio 0.5–1.5）会放过这个故障**：RR 的 0.61 落在
  容差内，脚本仍报 `RESULT: DRIVE LOOP OK`。读这个脚本的结论时要看表格数值，别只看末行。
- **✅ 已定位：故障在 RR 电机本体，不是驱动板**（2026-08-29 交叉互换实测）。
  把 RL/RR 的**电机输出线对调**（`AO1/AO2` ↔ `BO1/BO2`，编码器四路均未动），
  用开环占空比探针（`Ki=0` + 固定 `Kp` + 远高于可达转速的目标值，不参测通道
  `Kp=Ki=0` 钉零）逐通道测，按**编码器位置**判读：

  | 40% 占空比 | 电机 | 所在通道 | 实测 rad/s |
  | --- | --- | --- | --- |
  | 基准（接线未动） | FL | 板 1 A | **10.02** |
  | 对调后 | RL | 板 2 B | **10.37** |
  | 对调后 | **RR** | 板 2 A | **0.20 ~ 0.29** |

  **慢的跟着 RR 电机跑到了另一个通道**，且该通道驱动 RL 电机时完全正常 → 判定为
  **RR 电机本体故障**。板 2 的两个通道均健康。
- **RR 电机占空比扫描（在已证明健康的板 2 A 通道上）**：40% → 0.29、60% → 0.33、
  80% → 0.49 rad/s。**几乎不随占空比上升**，而同档健康电机为 FL 20.96 / FR 21.08
  （80%）。差 **40 倍以上**。这不是"偏弱"，是基本转不动。
- **探针本身已验证可信**：未动过接线的 FL 在 40% 下测得 10.02 rad/s，与
  [wiring.md](wiring.md) 记录的 `encoder_port_check` 独立实测 10.24–10.38 rad/s
  吻合（差 2%）；不参测的三路计数确实恒为 0。
- **中途发现并解释了此前的漂移**：对调后首次测 A 通道时**四路编码器全为 0**，
  排查为 RR 电机线未插紧（重新插上后即有输出）。这很可能也是此前 6.0 档三次连测
  由 5.01 → 4.70 → 4.22 **单调恶化**、以及 3.0 档偶现正常值（2.77）的原因——
  接触不良在渐进松脱。**因此"RR 偏弱"的早期数据是电机故障与接触不良两者叠加**，
  不能用那批数字去量化电机本身的衰减程度。
- **对调后的接线方向**：RR 电机在 A 通道上、RL 电机在 B 通道上均记为**负计数**
  （线序与原接法相反）。这是对调时线序反接的预期结果，不影响转速大小判读；
  **恢复原接法或换新电机后须复核"正占空比 = 前进"的符号**。
- **下一步**：更换 RR 电机（JGA25-370）。换好后需复做：四轮一致性扫描、
  符号一致性复核、并把接线恢复为 RL→A 通道 / RR→B 通道。
  （2026-08-29 更新：接线已恢复原接法；已临时换装一个 620 RPM 电机过渡，
  匹配电机次日到货 —— 见下节，**该临时电机减速比不同，禁止落地**。）
- **本次未做**：带载（落地）复测、30 分钟连续运行、`lx/ly` 实测替换。以上全部为空载架空。

### 临时换装 620 RPM 电机到 RR（2026-08-29，实测，空载架空）

**结论：接线与新电机均正常，但该电机减速比与其余三轮不同，`EDGES_PER_WHEEL_REV`
对这一路是错的 —— 仅可架空调试，禁止落地。** 匹配的 JGA25-370 次日到货，
因此**刻意不改代码**，以临时电机的错误标定换取一天的调试可用性。

**接线验证（每次只驱动一路，其余编码器严格为 0，`tools/openloop_probe.py`，40% 占空比）：**

| 驱动通道 | 动的编码器 | 实测 rad/s（固件报告） |
| --- | --- | --- |
| ch0 | FL | 10.26 |
| ch1 | FR | 10.02 |
| ch2 | RL | 10.51 |
| ch3 | **RR（620 RPM 新电机）** | **17.11** |

四路配对全部正确（用户已恢复 RL→A / RR→B 原接法），三个健康轮彼此在 ±2.5% 内，
基准可信。链路 50.5 Hz、CRC 0 错、静止零漂移、`error_flags=0x00`。

- **新电机健康**：旧 RR 在 80% 占空比只有 0.49 rad/s，新电机 33.58 —— 相差 68 倍。
- **减速比实测**（手转 5 圈，`tools/encoder_watch.py` 被动读绝对计数）：
  RR 计数变化 **−1020 → 204 边沿/圈**，其余三轮为 448。
  2 倍频下每电机圈 102 边沿 ÷ 11 CPR ≈ **1:9.3**，原电机为 1:20.4，**相差 2.196 倍**。
  同批次 FL 计数严格未变（11282 → 11282），排除串扰。计数为负仅说明手转方向与
  固件"前进"相反，驱动时计正数，符号约定不变。
- ⚠️ **固件把 RR 真实轮速低报 2.196 倍**（`EDGES_PER_WHEEL_REV` 是
  [`encoder.h`](../firmware/Core/Inc/encoder.h) 里的**全局单值、四轮共用**，无逐轮数组）：

| 占空比 | 固件报告 rad/s | **真实轮速 rad/s（推导）** | FL 实测 rad/s |
| --- | --- | --- | --- |
| 20% | 10.24 | **22.5** | 4.71 |
| 40% | 17.11 | **37.6** | 10.26 |
| 60% | 24.83 | **54.5** | 15.78 |
| 80% | 33.58 | **73.7** | 21.26 |

- ❌ **闭环下会失控且无报错**：PI 加占空比直到"报告值 = 目标值"，即**真实转速冲到
  目标的 2.2 倍**，而 `error_flags` 全程干净；里程计同步少算 2.2 倍。
  这正是"编码器对减速比变化是盲的"——读数正常恰恰是最危险的情形。
- **未消解的矛盾（推导估计）**：真实轮速比 FL 快 3.47 倍（80% 档），但按标称
  620 vs 280 RPM 只应快 2.21 倍；反推该电机输出轴在 80% 占空比下为 704 RPM、
  外推满占空比约 880 RPM，比标称 620 高 1.4 倍。可能是标称值记错（用户原话为
  "620rpm 好像"）、手转圈数偏差、或编码器 CPR 不是 11。**因临时电机次日即换下，
  刻意不再追查**；若日后长期使用同款须重测 10 圈并加测 FL 作对照。
- **刻意未做的改动**：改 `EDGES_PER_WHEEL_REV` 为逐轮数组需同步动 C
  （`encoder.h` / `encoder.c`）、C++（`mcr_hardware_interface.cpp`）、
  Python（`drive_check.py` / `stm32_uart_sim.py`）五处，外加
  `tools/check_calibration_constants.sh` 锁死 `448` 字面量的三条正则及相关测试。
  为一个一天寿命的临时电机不值得。
- **换回匹配电机后必做**：复核 RR 计数回到 448（手转 10 圈 ≈ 2240）、
  重做四轮一致性扫描、复核"正占空比 = 前进"符号。
- **那一路 PID 需重调**（若继续用此电机）：低报 2.2 倍等于回路增益翻倍，
  `Kp=100/Ki=300` 很可能振荡。另该电机 20% 占空比即达 10.24 rad/s，
  启动死区远低于原电机；`STALL_SPEED_MAX=1.0` 对它仍安全（差一个数量级）。
- **麦轮 IK 的固有问题**：四轮减速比不一致破坏等价轮假设，带载时该轮扭矩少 2.2 倍，
  **落地必然跑偏，且改标定常量修不了**。

### I2C2 外设 + MPU6050 驱动（2026-08-29，真机验收通过）

**动机：`firmware_arch_main()` 缺 I2C MSP 初始化**（见"未做/未验证"），IMU 与 ToF
两个驱动里所有 I2C 调用都是注释状态。排查后发现**它们引用的 `hi2c1` 从未在项目任何
地方定义过**——没有 `HAL_I2C_Init`、没有 MspInit、没有时钟使能，因此
"取消注释即可"是错的，会直接编译失败。

- **总线选定 I2C2（PB10 SCL / PB11 SDA）**，与 [wiring.md:112](wiring.md#L112) 一致。
  ⚠️ 文档 [wiring.md:255](wiring.md#L255) 另有一处写 IMU 走 I2C1/PB8-PB9，**该方案不可行**：
  PB8 是 RL 电机的 `PWMA`。I2C1 默认脚 PB6/PB7 也与 RL 编码器冲突。
  已核实 PB10/PB11 空闲（`rtos_drive_main.c` 中的 `GPIO_PIN_10` 是 **PA**10 = USART1 RX）。
- **实现**（`rtos_drive_main.c` 新增 `i2c_sensor_init()`）：`GPIO_MODE_AF_OD` 开漏
  （推挽会与总线上拉电阻打架，破坏时钟拉伸与仲裁）、`GPIO_NOPULL`（模块自带上拉）、
  400 kHz、**阻塞模式不挂 NVIC**（刻意避开 FreeRTOS syscall 优先级天花板）。
- **驱动改为句柄注入**（`mpu6050_set_i2c()`），沿用 `motor_set_tim()` 的既有模式：
  驱动不预设自己挂在哪条总线上，SIL 与单测才能注入假总线。
- ⚠️ **每次事务用有界超时 10 ms，不用 `HAL_MAX_DELAY`**：传感器未接或未供电时
  `HAL_MAX_DELAY` 会让 `SensorTask` 永久卡死。`test_mpu6050.c` 断言了这一点。
- ⚠️ **`mpu6050_init()` 此前无条件返回 true**，对着不存在的硬件也"成功"。现在
  WHO_AM_I 不匹配即返回 false 且**在碰配置寄存器之前退出**。
- **读失败返回零向量**而非脏数据：Mahony 有零模保护分支，掉帧退化为纯陀螺积分，
  不会把姿态拽向一个假的重力方向。
- **`SensorTask` 接入真机 target**，当时新增 `HW_IMU_ONLY` 只启用 IMU；ToF 半边先编译掉，
  避免占位驱动把 `ERR_TOF_TIMEOUT` 永久钉在 Pi 看到的 `error_flags` 里。2026-08-29
  VL53L0X 驱动完成后该开关改为构建参数：模块未装保持默认，装好后用 `TOF=1` 启用。
- **顺带修掉一个潜伏的链接顺序 bug**：`-lc -lm` 原在 `LDFLAGS` 里、位于目标文件
  **之前**，而 ld 从左到右解析，库先被扫过时还没有未定义符号、之后不再回头。
  表现为 `sqrtf` 找不到 `__errno`。**该 bug 一直存在但被 `--gc-sections` 掩盖**
  （此前无人引用 AHRS，整段被丢弃，libm 未被需要），`SensorTask` 一启用即暴露。
  改为 `LDLIBS` 置于目标文件之后。
- **验证**：`test_mpu6050` 8 项通过（覆盖六个配置寄存器实际值、WHO_AM_I 拒绝、
  总线 NACK、未注入总线、大端解码含负数、跳过温度字节、读失败零向量、单位换算）；
  CTest 8/8；真机 `-Wall -Werror` 干净链接，text 20860 → **25204 B**
  （+AHRS +libm +I2C HAL）；`nm` 确认 `SensorTask` / `mpu6050_*` /
  `MahonyAHRSupdateIMU` 均在 ELF 中。
- **真机验收通过（2026-08-29）**：MPU6050 按 `3.3V / GND / PB10 SCL /
  PB11 SDA / AD0 GND` 接入后，`rtos_drive` 经 ST-Link 烧录并 verify。Pi 端运行
  `python3 tools/imu_watch.py --duration 15`，收到 **754 个 ODOM 样本（约 50 Hz）**、
  **CRC failure 0、`error_flags=0`**；四元数 norm 始终在 0.95–1.05，静止姿态约
  `roll=-0.6° / pitch=+2.7°`，gyro 有真实噪声而非固件的 identity/全零故障默认值。
  这闭合了 `I2C2 → MPU6050 → Mahony AHRS → robot_update_imu() → UART ODOM → Pi`
  整条数据链。
- 新增 `tools/imu_watch.py` 作为可重复验收工具：只发 heartbeat，不发速度命令，
  车轮保持急停；实时显示 RPY/gyro，并拒绝无 ODOM、CRC 错、非单位四元数以及
  identity quaternion + zero gyro。
- **实测待校准项**：静止时 Y 轴 gyro 约 `+0.045 rad/s`（约 `+2.6°/s`），说明连接和
  采样有效，但零偏偏大。下一步应在启动静止窗口内取均值并扣除 gyro bias，再做手动
  90° 转动和静置漂移量化；当前结果只证明数据链，不代表姿态精度已验收。
- 接线约束仍是：**AD0 必须接地**（驱动按 `0x68` 写死）、**VCC 走 3.3V**，轮询驱动
  不用 INT 脚。当前 400 kHz 已在实物杜邦线下稳定通过，无需降到 100 kHz。

### VL53L0X + SSD1306 驱动模块（2026-08-29，真机闭环通过）

- **VL53L0X 不再是占位实现**：I2C 句柄注入后校验 0xEE model ID，从 NVM 读取 reference
  SPAD 数量/类型并读取 factory SPAD map，加载 STSW-IMG005 v36 默认 tuning table，执行 VHV/phase reference calibration，
  再进入 back-to-back continuous ranging。没有把完整 ST PAL 搬入 F103，只保留避障所需路径。
- `tof_read_mm()` 是非阻塞轮询：SensorTask 20 Hz 读取最新结果，三次连续未 ready/NACK
  才报告 `TOF_TIMEOUT`；raw range status 仅接受 ST 定义的 11（valid），越界或错误保持
  last-known-good，避免用假 0 触发/解除避障逻辑。每次 I2C 事务 5 ms、初始化校准 50 ms
  有界超时，未接模块不会永久卡住任务。
- I2C2 在 `rtos_drive_main.c` 同时注入 MPU6050 与 VL53L0X。默认仍编译
  `HW_IMU_ONLY`；接线后使用 `make TARGET=rtos_drive RTOS=1 TOF=1 flash-stlink`。
- 新增 `tools/tof_watch.py`：只发 heartbeat、不发速度命令，显示 ODOM 中的 mm 和
  `error_flags`；无帧、CRC 错、`TOF_TIMEOUT` 或全程零距离均失败。
- **SSD1306 车端轻量模块**：支持 128×64、地址 0x3C 的 init/clear/full-frame/
  display enable/contrast，所有写入有 5 ms 超时。驱动不内置字体或 1 KiB framebuffer，
  因而仅启用模块不会吃掉 F103 5% SRAM；调用者需要整帧显示时自行提供 1024 B buffer。
  手柄端原有江协 OLED 驱动保持不动，已重新交叉编译通过。
- **验证**：CTest **10/10**；VL53L0X 4 组 host 测试覆盖连续测距配置、错误身份/NACK/
  NULL bus、有效/越界结果和三次缺帧超时；SSD1306 3 组测试覆盖初始化清屏、整帧大小和
  bus failure。`TOF=1` 真机 target 在 `-Wall -Werror` 下为 **text 26956 / data 360 /
  bss 14056 B**，相对 IMU-only 增加约 1.8 KiB flash、16 B RAM。
- ✅ **YB-MVV18 真机闭环通过（并记录 F1 I²C 读取缺陷）。** 模块在 `0x29` 完成型号校验、
  SPAD/VHV/phase 校准和连续测距。初版以一次 12-byte burst 读取 `0x14..0x1F`，在此模块与
  STM32F1 I2C2 组合上会把距离低字节复制为高字节，伪造 `0x0101/0x0202/0x0303`
  （257/514/771 mm）三档；安全探针同时读取证明单字节值连续为如 `0x03C3/0x039F/0x03B6`。
  `tof_read_mm()` 已改为分别读取状态、MSB、LSB，并有模拟该 burst 缺陷的回归测试。修复后 Pi
  端 8 秒收到 401 帧、CRC 0、38 个不同距离值（约 931–962 mm）、`error_flags=0`，闭环验收通过。
  SSD1306 `0x3C` 与 MPU6050 共用 PB10/PB11；VL53L1X、SH1106 不兼容。
- ✅ **OLED 分层仪表盘**：`OLED=1` 顶部标注单位、中部以 2× 5×7 字体突出 ToF 距离，底部只保留
  链路/运行/错误、陀螺仪和四轮命令摘要。单个 1024 B 静态 framebuffer 驻留 BSS，无动态分配，
  以 1 Hz 刷新。
  已以 `stm32-smart-home-ota` 实机验证的 SSD1306 传输格式（7 位 `0x3C`，即模组丝印
  的 8 位写地址 `0x78`）修正显示；每次换页一次性刷新整帧，消除逐页写入的重影。target
  为 **text 29416 / data 360 / bss 15120 B**；CTest 增至 **11/11**。

### NRF24 遥控并入 `rtos_drive`（2026-08-30，真机闭环通过）

- 新车端 NRF24 已先用独立 SPI probe 验证：MISO 上拉正常，快/慢时序读取 `STATUS=0x0E`。
  手柄与车端地址、频道、速率一致；手柄 `Sig=10` 表示最近 10 次发送全部收到车端 Auto-ACK。
- 正式 `rtos_drive` 曾遗留 `HW_MINIMAL_TASKS`，导致 `nrf24l01.c` 虽参与编译却没有创建
  `RemoteTask`。移除该宏后以 `nm` 确认任务进入 ELF，空口收发恢复。
- 修复上电安全锁的两层状态不一致：底层 `motor_init()` 已锁住 PWM，而上层
  `emergency_stop_active` 原为 false，导致有效遥控目标也被底层永久丢弃。现在首次有效遥控
  只解除上电/通信超时锁；K9、ToF 近障和堵转故障仍保持锁存。
- 将 M3 真机验收过的手感移植到生产控制环：电机模型前馈 + `Kp=15/Ki=35`、每 10 ms
  目标斜坡 1 rad/s、回中进入 0.25 rad/s 死区后强制 PWM=0 并清积分；无线看门狗由与
  10 Hz 发包周期竞争的 100 ms 恢复为 250 ms。操作者真机确认新版明显更丝滑，停止后不再
  因高增益追逐编码器量化误差而持续咔哒。
- 新增回归覆盖：遥控解除上电/看门狗锁、不能解除显式急停、默认控制越过启动死区、回中时
  ±1.4 rad/s 编码器量化跳变仍保持 PWM=0、150 ms 合法包间隔不误急停。CTest **11/11**；
  `rtos_drive RTOS=1 TOF=1 OLED=1` 为 **text 31236 / data 360 / bss 15176 B**，ST-Link
  定向烧录小车并 verify 通过。
- RR 已更换为新电机；旧 RR 的弱驱动和异常一致性数据不再代表当前硬件。仍须重新做四轮
  空载/落地一致性扫描后才能建立新基线。

### M5：ROS 2 整车栈闭合 + SLAM 出图（2026-08-28，实测）

- **`ros2_control` 硬件接口已激活并稳定运行。** `MCRSystem` 状态 `active`、四个轮速命令
  接口 `available/claimed`，三个控制器（`mecanum_drive_controller`、
  `joint_state_broadcaster`、`imu_sensor_broadcaster`）全部 `active`。实测
  `/mecanum_drive_controller/odometry` **99.999 Hz**、`/odometry/filtered` **50.001 Hz**、
  `/scan` **~10 Hz**；`odom → base_footprint` TF 由 EKF 稳定发布（静止时全零，符合预期）。
- **修掉两个 ROS 2 侧真 bug（此前每次启动几秒后硬件组件即被停用）：**
  - **`serial_protocol.cpp` 的 B0 挂断实际一直存在。** Linux 的线速存放在 `c_cflag` 的
    `CBAUD` 位域里，而 `open()` 中先 `cfsetospeed()` 再 `tty.c_cflag = CS8|CREAD|CLOCAL`
    这个**赋值**把刚写入的波特率位清成 0，即 **B0（modem 挂断）**。`8b9a977` 只修了
    "用 B 常量替代裸整数"这一半，赋值顺序仍在清掉结果。诊断依据：`TIOCOUTQ` 实测
    内核 TX 队列**恒为 16384（满）**、`write()` 恒返回 `EAGAIN`，而同端口同 termios 的
    Python 写入在 100 Hz 下 `outq=0`、1000 次零失败。已改为**先设标志位、最后设波特率**，
    并 `tcgetattr` 回读校验速率确实生效（`tcsetattr` 可能成功却静默忽略不支持的速率）。
  - **写入背压被当成致命错误。** 端口是 `O_NONBLOCK`，部分写入/`EAGAIN` 属于背压
    （STM32 处于上电安全锁存期间不排水时必然出现），旧代码据此 `return ERROR`，
    直接触发 `controller_manager` 停用整个组件及其所有控制器。现改为：`write_bytes`
    对剩余字节做有限轮询（整帧写出或丢弃），且仅在**连续**失败达 100 次
    （100 Hz 下约 1 s）才升级为 ERROR——彼时 STM32 自身的通信看门狗也会停机。
- **SLAM 已出图（`slam_toolbox`）。** 实测 `/map` 为 **87×62 栅格、分辨率 0.05 m**、
  `frame_id: map`、发布率 **1.000 Hz**（即配置的 `map_update_interval: 1.0`），
  `map → odom` TF 正常，日志出现 `Registering sensor: [Custom Described Lidar]`。
- **根因：`async_slam_toolbox_node` 是 LifecycleNode，却被当普通 `Node` 启动**，
  因此停在 `unconfigured` 不动——只打印 `Node using stack size ...` 后再无输出，
  既不订阅 `/scan` 也不发布 `/map`。**此状态下 `ros2 param get` 会报 "not set"，
  看起来像 YAML 没加载，其实只是节点未 configure 的假象**（YAML 路径与节点名均正确）。
  已改为 `LifecycleNode` 并驱动转换：configure 挂在 `OnProcessStart`
  （裸 `EmitEvent` 会在节点生命周期服务就绪前发出而被静默丢弃）、activate 链在
  `OnStateTransition(configuring → inactive)`（configure 要加载 Ceres 求解器插件，
  Pi 上实测约 10 s，不能用固定 sleep）。注意 `OnStateTransition` 在
  `launch_ros.event_handlers` 而非 `launch.event_handlers`。
- **`navigation.launch.py` 新增 `nav2:=` 与 `rviz:=` 开关。** 建图只需 `/scan` 与
  `odom→base_footprint` TF，拆开后可单独验证 SLAM——这也是目前唯一能跑 SLAM 的路径，
  因为 `nav2_bringup` 用 `PythonExpression` 求值 `slam` 参数，而 `'true'` 不是 Python
  字面量（`True` 才是），带上 Nav2 会抛 `NameError: name 'true' is not defined`。
  **Nav2 本身因此仍未验证。**
- **未验证/须注意**：以上均为**静止**状态下的链路与建图验收，**车未移动**，因此
  没有回环、没有真实位姿轨迹，不能据此判断建图精度或里程计符号是否正确；
  逐轮"向前=正"的符号一致性仍待电机驱动板到货后随带载实测复核。
- **IMX219 CSI 相机可用。** Pi 识别为 3280×2464、10-bit RGGB；640×480 `RGB888`
  连续采集实测 30.56 FPS，静态拍照正常。宿主机以 `Picamera2 + ONNX Runtime 1.23.2`
  运行仓库 `yolov8n.onnx`，20 帧连续测试平均推理 163.8 ms、端到端 6.05 FPS；当前场景
  两个显示器在 20 帧中共得到 38 个 `tv` 检测框，最高置信度 0.741，另一次人体局部检测
  置信度 0.883。该结果证明 Pi 5 CPU 可承担低速视觉检测，不代表已完成 ROS 2 相机节点。
- **CSI/headless 独立入口已合入并通过 Pi 回归。** `perception/detection/yolo_detect.py`
  默认使用 `Picamera2` 的 IMX219 CSI 输入、脚本同目录模型路径和 console 状态输出；
  `--max-frames 20` 可做有界烟雾测试，`--camera usb` 保留 USB/OpenCV 回退，`--display`
  只在有桌面时启用。解析/NMS 已有无相机单元测试。M5 已新增 `mcr_perception`：宿主机检测
  进程通过 loopback UDP 发送版本化 JSON，Docker 内桥接节点发布 `/perception/detections`
  和 `/perception/status`；CSI 仍不进入容器。2026-08-29 在 Pi 5 + IMX219 上以该正式入口
  完成 20 帧 headless 连续测试，端到端 4.52 FPS、无异常退出；测试画面没有超过阈值的目标，
  因而正常报告 0 个检测。

### M4：Pi↔STM32 真机链路 bring-up（2026-08-28，**链路已闭合**）

- **`rtos_drive` 固件目标已就绪（SIL/编译验证，尚未真机验收）。** `make TARGET=rtos_drive RTOS=1`
  把 SIL 验证过的完整 Core/Src 应用搬上真板：CommTask（USART1 PA9/PA10 @921600，DMA1 ch4 TX /
  ch5 RX）、CtrlTask（100 Hz 四轮速度 PI）、50 Hz 里程计；启动即安全锁存急停，收到有效
  `CMD_VEL_CTRL` 且链路活着才解除。配套改动：
  - RX DMA 改 **CIRCULAR**，中断里按 CNDTR 派生"新写入区间"排水进软件 ring，消除了在
    RxEvent 回调里重 arm 与 BUSY_RX 竞争、跨 staging 边界丢帧的旧缺陷；
  - `HAL_UART_ErrorCallback` 增加 `comm_uart_errors` 计数，先清 ORE/FE 标志再重 arm RX
    （F1 HAL 出错路径不读 DR，不先清标志会重入风暴）；信号量/通知对象在 UART NVIC 使能前
    创建，避免复位时 Pi 已在发字节导致 `xSemaphoreGiveFromISR(NULL)` 断言；
  - 修复零负载帧（心跳/ACK）解析：此前 `exp_len==0` 时状态机永远到不了 CRC 状态，真机上
    还会持续越界写 `frame_buf`；SIL 新增心跳 ACK 回归；
  - deadman 语义：通信看门狗/上电导致的急停可被首个有效运动指令解除，显式急停和 ToF 停车
    不自动解除；
  - 新增 `motor_hold` 接线诊断目标（四轮固定占空比常转，供万用表探 TB6612 信号）。
- **ROS 2 侧修复一个真 bug：`serial_protocol.cpp` 波特率设置。** 旧代码把裸波特率整数传给
  `cfsetospeed/cfsetispeed`，与 CBAUD 掩码后得到 `B0`——内核把 B0 当作 modem 挂断，端口停在
  挂起态，输出缓冲填满后写失败（表现为节点起量几秒后 EAGAIN/ENOTTY、硬件组件去激活）。
  已改为 `B<speed>` 常量映射并检查 `tcsetattr` 返回值。
- **Pi 侧真机检查工具（stdlib，在 Pi 宿主机跑、不进容器）：** `tools/link_check.py`
  （链路/心跳/ODOM 流）、`tools/encoder_watch.py`（手转编码器看计数）、
  `tools/drive_check.py`（端到端驱动，`--spin` 会真动车，先架空底盘）、
  `tools/uart_baud_sweep.py`（只听不发，多波特率扫 A5 5A 帧，判断 STM32 实际波特率）。
- **洪流测试实测（2026-08-27 深夜）：链路双向不通 + 最终 lockup。**
  - 烧录曾有一次"擦了没写进"的失败，芯片在擦空 flash 上跑 `0xFFFFFFFF` 指令，表现为
    PC=0xFFFFFFFE/MSP=0xFFFFFFD8——**那是空 flash 不是固件 HardFault**，重新烧录 verify 通过。
  - ttest4 以 100 Hz 发 23 字节合法帧头帧（CRC 字段为 0，协议层必丢）18 秒：
    **`comm_uart_errors = 45772`（≈每个接收字节都报错，帧错误级）**；Pi 侧 `IGNPAR` 把坏字节
    直接丢弃，只收到 ~50 B/s（正常 50 Hz ODOM 应 ~1350 B/s）——这是**双向对称的波特率/电气
    不匹配**特征，不是单向 TX 停滞；约 14 s 后 Pi 发送也规律卡死（对端不读，内核 TX 缓冲填满
    后 EAGAIN）。
  - 测试末期芯片进入 **lockup**：PC=0xFFFFFFFE、MSP=0xFFFFFFD8、CFSR=UNDEFINSTR+IACCVIOL，
    即异常栈/向量取指失败（fault-on-fault），说明错误风暴路径里存在栈/堆破坏。这是
    **独立于链路故障的真 bug**；错误回调、DMA abort/重 arm 顺序、解析器边界已逐行静态审查
    未发现越界，需下次复现时先抓 CFSR/HFSR/BFAR/PSP/ICSR 再复位定位。
- **Pi 侧已实测洗清：** ttyAMA0（RP1 PL011，物理 `0x1f00030000`）参考时钟 **50 MHz**；
  921600 实配 IBRD=3/FBRD=25 → **921,659 baud（+0.06%）**，LCR_H=0x70（8N1+FIFO），
  CR 中 UARTEN/TXE/RXE 均使能；ttyAMA0 上无 console/getty 占用。
- **✅ 已恢复并闭合（2026-08-28 实测）。** 上面"待恢复/下一步"的 ①②③ 均已完成：
  - ① 物理重插 ST-Link 后 `st-info --probe` 正常：`chipid 0x410`、flash `131072`、
    sram `20480`、V2J27S6（序列号 `...02361C43`）。**连测三次读数完全一致**（按项目判据即健康），
    与故障板的 `chipid 0x000` 完全不同。
  - ② Pi↔STM32 三线万用表已复核导通：`pin8↔PA10`、`pin10↔PA9`、共地。固件侧引脚配置见
    `rtos_drive_main.c:238-246`（PA9 = `GPIO_MODE_AF_PP`，PA10 = `GPIO_MODE_INPUT`）。
  - ③ **芯片已自 lockup 恢复**：PC=`0x08002f06`→`0x08002f58`（在 flash 正常代码区且推进）、
    PSP=`0x20001ce0`、xPSR=`0x61000000`（Thread 模式）、目标电压 3.228 V。
  - **时钟已终判正常**：CFGR=`0x0038040A` → SW=PLL、PLLSRC=HSI/2、PLLMUL=×16 →
    **SYSCLK=PCLK2=64 MHz**；BRR=`0x45`（mant=4/frac=5，USARTDIV=4.3125）→
    **927,536 baud**，相对 921600 偏 **+0.64%**，在 UART 容限内。
  - ⚠️ **修正本文档此前记错的"健康值"**：原记 CFGR `0x001C040A` / BRR `0x00000457` 解出来是
    **36 MHz / 32,403 baud**，与项目"HSI+PLL @ 64 MHz"基线自相矛盾，那组数是错的。
    正确健康值为 **CFGR `0x0038040A` / BRR `0x45`**。
  - **波特率扫描（Pi 侧只听不发）**：921600 得 `8558 B/3s`（2853 B/s）、**A5 5A 帧头 155 个**、
    仅 2 个 `0xFF`，标记 CLEAN；460800/230400/115200 均 0 个帧头。
  - **双向链路实测 OK**：`tools/link_check.py` 12 秒 → **有效帧 615、CRC 失败 0**、
    ODOM **50.2 Hz**（`rtos_drive` 设计值 50 Hz，工具默认按 20 Hz 的 probe 目标提示，
    故那条 `outside 15-25 Hz` 警告不是故障）、心跳 **ACK 12/12**、`RESULT: LINK OK`。
  - **此前"每字节帧错误 45772"的真因是两个已修 bug，不是硬件**：ROS 2 侧
    `serial_protocol.cpp` 的 B0 挂断（`8b9a977` 已修）+ 芯片当时停在 lockup。
- **四路编码器已确认全部有信号（2026-08-28，手转实测）。** 经 ODOM 帧读取累计计数，
  手动带动底盘后四轮累计绝对位移：FL 365、FR 397、RL 2129、RR 1768，量级与手转幅度相符；
  FL 正反向均出现过（-989 / +256），软件 EXTI 解码工作正常。
  **未做**：逐轮受控转动以核验"向前=正"的符号一致性（历史上已验过并为统一符号在 FL 线束上
  对调过 A/B），留待电机驱动板到货后随带载实测一并复核。
- **仍未解决（独立于链路的真 bug）：** 洪流测试末期的 Cortex-M3 lockup
  （PC=0xFFFFFFFE、MSP=0xFFFFFFD8、CFSR=UNDEFINSTR+IACCVIOL）尚未定位；
  下次复现时 **halt 后先抓 CFSR/HFSR/BFAR/PSP/ICSR 再复位**。
  SIL 已新增 RX ring 填满、噪声排空、合法 heartbeat 恢复 ACK 的回归，证明当前可模拟的
  ring 溢出/解析路径会丢计数但不会卡死；该测试不覆盖真机 DMA 错误中断重入。


### 已实测（真机）

- **主控** STM32F103C8T6（Blue Pill，**无 HSE 晶振**，HSI+PLL @ 64 MHz）。
- **引脚映射**已确认并验证，见 [wiring.md](wiring.md)。
- **四路电机开环驱动**：方向、PWM、STBY 全部验证。正占空比 = 前进（方向引脚按 (AIN2, AIN1) 传入修正）。
- **四路编码器**：软件 EXTI 解码，计数与方向验证；FL 编码器 A/B 已在线束上对调以统一符号。
- **标定常数**：`EDGES_PER_WHEEL_REV = 448`。其 1 倍频实测基准为 224 edges/圈，
  A 相双边沿解码后精确翻倍为 448；轮径 60 mm / 半径 0.030 m。
- **占空比→转速曲线（台面供电）**：20% 以上线性，斜率 ~0.986 edges/(duty·s)；启动死区 5%~10%；四轮一致性 2.8%；满占空比外推 ~4.27 rev/s ≈ 0.81 m/s。数据表见 [hardware-closed-loop-roadmap.md](hardware-closed-loop-roadmap.md)。
- **占空比→转速曲线（3S 电池供电，2026-08-25 实测）**：电池 **11.47 V**（实测记录，这个数以前一直没记）、空载抬起、四轮完整数据。
  - 比台面供电**整体快约 4.2%**（推导），死区不变（5% 不转 / 10% 起转），线性度保持。
  - **外推**满占空比 ~4.45 rev/s ≈ 0.84 m/s（外推值，非实测）。
  - ✅ **M2 的 `Kp=100 / Ki=300` 经此确认不需要因换电池重调**；仍未验证落地带载。
  - ⚠️ **四轮一致性变宽到 4.9%，RR 最慢** —— M3 不要默认四路共用一组参数。
    （**2026-08-29 补注**：RR 偏慢现已定位为该轮电机本体故障，见上方"新电机驱动板
    换装后四轮复测"。这条 4.9% 因此不能当作正常的轮间差异基线，换电机后须重测。）
- **NRF24 遥控链路（2026-08-25 实测）**：车端 `remote_drive` 目标 + 手柄 `remote_controller` 固件均已烧录。
  - 25 秒收到 **249 包 = 9.96 Hz**（手柄标称 10 Hz），**丢包基本为零**。
  - **手柄断电重启的间歇性失控已定位并修复**：该完整手柄会在部分启动时未能切到
    HSE+PLL，而停在 8 MHz HSI。旧固件把 TIM1 预分频硬编码为 71，导致本应每 100 ms
    发一次的包变成约每 900 ms 一次；`Sig:10` 只表示这些稀疏包都收到了 ACK，不能代表
    车端持续有包。现在 TIM1 按 RCC 实际 PCLK2 计算预分频（8 MHz 用 PSC=7，72 MHz 用
    PSC=71），并将中断共享标志声明为 `volatile`。正式镜像已读回校验；车端重置后约 3 秒
    收到 42 个有效包，恢复约 10 Hz。仍需在手柄实际断电重启后复核连续驾驶手感。
  - **安全默认已实测**：不按 K1 时四轮占空比恒为 0，电机不动。
  - **连续控制已实测（2026-08-25，瓷砖地面）**：K1 按一次后可连续操纵；前进、
    后退、左右横移、左/右原地旋转均按手柄方向工作。底盘后轮已换为标准 **X 型**
    滚轮布局（STM32 端为车头）。
  - **代码级诊断已真机复验**：原车端 100 ms 超时与实测
    9.96 Hz（平均 100.4 ms）发送周期竞争，会在正常相邻包之间清掉 K1 锁存，
    正是“动一下就停、每次都要重按 K1”的直接原因；改为 250 ms 后已消失。
    左右两个摇杆的 ADC 符号也已分别反转以符合“左拨向左”的直觉，并有主机回归测试。
  - **安全功能已实测**：K9 急停有效；手柄断电/失联后自动停车有效。
    固件阈值为 250 ms；本次为功能验证，未以仪器测量精确停机时间。
- **M3 四轮速度闭环（2026-08-25，实测）**：独立的 `remote_pid_drive` 目标以 100 Hz
  运行四个速度 PI。最终采用前馈加 `Kp=15 / Ki=35 / Kd=0`、每周期目标斜坡 1 rad/s，
  回中时清零 PWM 和积分，避免静摩擦区的咔嗒抖动。空载抬起时已核对四种基本动作：
  前进目标 `{+6.20,+6.20,+6.20,+6.20}`、实测 `{+5.61,+5.61,+5.61,+5.61}`；后退、
  左横移和逆时针旋转也均为目标符号与实际轮向一致，且未触发编码器故障。随后在瓷砖地面以
  约 10% 摇杆量测试前进、左横移和左转，操作者观察为低速连续、平滑且可响应小输入。
  K9 急停和手柄断电失联也已用该镜像复验；复验后遥测为 `rx_packets=747`、
  `control_ticks=5612`、四轮 PWM 均为 0、`drive_active=0`、`failsafe_hits=1`、
  `encoder_fault=0`，与安全停车一致。
  这是基础功能验收，不代表已完成带载精度或长期可靠性验收。

### 已通过 SIL / 单元测试（非真机）

- 共享协议（CRC16-MODBUS、帧同步）、麦克纳姆运动学、Mahony AHRS、遥控映射、PID 数据链。
- CTest 10/10 通过（含 `test_stall`、`test_mpu6050`、`test_tof_sensor`、`test_ssd1306`）；`firmware/arm_controller` SIL 24/24 通过。
- UART DMA/IDLE 代码路径已实现（staging buffer 与软件 ring 分离），**真机未验证**。
- UART 模拟器轮速对象模型使用台面供电测得曲线的推导上限 4.27 rev/s，以及由
  5% 不转、10% 时 0.32 rev/s 推导的保守启动阈值；它不是电池供电或带载实测模型。
  模拟器轮半径已同步为实测 0.030 m。该静态模型的主机测试已通过，ROS 容器
  `verify_sil.sh` 尚待具备 Docker 的环境复核。

### 未做 / 未验证

- **遥控开环验收已完成**：K1 锁存、前后/横移/旋转、K9 急停和手柄断电失联停车均已真机验证；精确失联停车延迟尚未仪器测量。
- 带载（落地）定量转速、堵转电流、阶跃响应指标。**M2 已完成**（`Kp=100 / Ki=300`，空载，台面与电池供电下均已验证）。
- **M3 四轮闭环**：基础动作、K9 急停和 250 ms 失联停车已验收；尚未做 30 分钟连续运行、
  量化的落地带载速度/跟踪误差。~~四轮电池供电扫描的差异为 4.9%，RR
  最慢，后续若出现偏航应以分轮记录为依据调参。~~
  **2026-08-29 更正**：RR 偏慢已定位为电机本体故障（非需要分轮调参），
  换电机后须重测四轮一致性基线。
- `lx = 0.10 m`（半轴距）和 `ly = 0.12 m`（半轮距）仅为默认估计值，均未实测；
  应量前后轮、左右轮的轴中心距后各除以 2，再替换 ROS 2 与模拟器中的默认值。
- `firmware_arch_main()` **不能在真机跑**：缺 UART 的 MSP 初始化；`motor.c` 引脚映射目前由各 HW target 在初始化时传入，不是静态表。
  `rtos_drive` target 已在进入它之前完成 UART DMA 和 I2C2 MSP 初始化；I2C2/MPU6050
  已于 2026-08-29 真机验收（见上文）。
- **IMU 已上真机并闭合到 Pi ODOM 帧**，但 gyro 零偏校准和动态姿态精度仍未验收。
  ToF、Nav2 尚未上真机；ToF 驱动已完成并通过 host/交叉编译验证，接线后需用 `TOF=1`
  target 烧录验收。
  LD06 已接入 `robot.launch.py`（见上文），但仍未与真实里程计、
  TF 闭环和 SLAM 形成整车链路；IMX219/YOLO 的宿主机到 ROS 2 检测桥已实现，仍待在重新启动的
  ROS 容器中完成端到端话题验收。
  NRF24L01 的旧模块已故障下线；新模块的 SPI 健康检查已通过，但尚未完成手柄 Auto-ACK
  与实际收包的当前板验收，不能写成整车无线遥控当前可用。
- 机械臂：只有 host protocol 与 SIL 骨架，硬件缺货未到（智能总线舵机版），见 [manipulator/docs/decision-log.md](../manipulator/docs/decision-log.md)。

---

## 当前卡点（2026-08-25 收尾时的状态）

遥控主通路的驱动、K1 锁存、前后、横移和旋转均已在瓷砖地面验收。M3 四轮速度闭环的悬空
前进/后退/横移/旋转和低速落地基本动作也已验收。此前“动一下就停”有
两层原因：车端原先 100 ms 看门狗与正常 10 Hz 发包周期竞争；手柄还会间歇回退到 8 MHz
HSI，使硬编码的 72 MHz 定时器将发包降至约 1 Hz。车端阈值已改为 250 ms，手柄 TIM1 现按
实际 RCC 时钟计算；正式镜像读回一致，车端采样恢复约 10 Hz。重复 K1 帧现在按边沿处理，
避免手柄在收到 ACK 前保持 KEY 非零时把已开启状态再次翻转。底盘后轮已调整为标准 X 型
滚轮布局；不要再按旧照片中的 O 型布局装回去。

**安全验收也已通过（真机）**：K9 急停有效；关掉手柄电源后自动停车有效。250 ms 是
固件阈值，尚未用仪器测量精确延迟。手柄 OLED 的间歇暗屏已修复：原有“上电等待”空循环
被 `-Os` 优化删除；现用按 `SystemCoreClock` 换算的真实 100 ms 延时，拔掉 ST-Link 后多次
断电重启均正常显示。

### 两个 ST-Link 的序列号

同时接两个时用 `--serial` 指定，`st-flash` 和 `st-util` 都支持：

| ST-Link | 序列号 | 备注 |
| --- | --- | --- |
| V2J46S7 | `37FF71064E57343623CE1E43` | 编程器可换插；2026-08-30 单独连接手柄时，此设备已写入并校验 `remote-control.bin`。 |
| V2J27S6 | `37FF71064E57343602361C43` | 编程器可换插；不可仅凭序列号推断当前连接的是车端还是手柄。 |

⚠️ 序列号标识的是 **ST-Link 编程器**，不是目标板；烧录前必须先确认物理 SWD 接线，
或在只连接一个目标板时再操作。工具报告的 flash 容量来自当前目标/探测过程，不能用它
反推是哪块板。

---

## 固件结构（容易走错的地方）

- `firmware/Core/Src/` —— 生产代码，被 SIL、单元测试和真机目标共用。
- `firmware/Core/HW/` —— **裸机验证目标，不启动 FreeRTOS**，每个 target 一个 `*_main.c`。构建方式和各 target 用途见该目录 [README](../firmware/Core/HW/README.md)。
- `firmware/Core/SIL/` —— Linux 主机仿真，mock HAL + 轮询调度器。
- **编码器不用硬件定时器**：TIM2/3/4 全部产生 PWM，TIM1 编码器脚被 NRF24/USART1 占用，四路全部软件 EXTI 解码。改这块前先读 `encoder.c` 顶部注释。

### 读 GDB 变量的注意事项

`st-util` 在**启动时和 GDB 连接时都会复位芯片**。连上就立刻 halt 读到的是刚复位几毫秒的状态（还停在启动延时里），会看起来像"编码器不计数但轮子在转"。正确做法是先 `continue` 让它自由跑够时间再打断，示例见 HW 目录 README。

### 供电陷阱：SWD 连不上时先怀疑电压（2026-08-25 报废一块板）

**一块 Blue Pill 的 AMS1117 稳压器击穿短路**，把 `5V` 脚的电压直通到 3.3 V 轨，MCU 挨了 **4.8 V**（绝对最大值 4.0 V）而损坏。完整故障链和自检方法见 [power-system.md](power-system.md) 的"故障记录"一节。这里只记最容易踩的两点：

- **短路的稳压器在 3.3 V 供电下毫无症状**——只要你从 `3V3` 脚供电，它就是一根导线，能正常工作数周。只有改用 `5V` 供电时才暴露。
- **SWD 症状会伪装成接线问题**：`chipid 0x000`、读数是垃圾、时通时断、USB 掉线——排查了半天杜邦线，真因是芯片在超压/欠压下无法工作。

判据：`st-info --probe` 的 **flash 容量读数**。好芯片稳定报 `65536`（64 KiB）；坏芯片每次都不同且全错（实测见过 `262144` / `67107840` / `24576 KiB`）。**读数不稳就别再查线了，查电压。**

---

## CI

两个 workflow，推送后**请确认都变绿**（历史上 firmware-tests 红了 9 天没被发现）：

- `firmware-tests.yml` —— gcc 直接编译各单测 + CMake/CTest + 两套 SIL。
  ⚠️ **CI 并不运行 `ctest`**：它只跑显式列出的 gcc 步骤，CMake 那一步只 build 后跑
  `./sil_firmware --ci`。**新增主机单测必须同时补一条 gcc 步骤**，否则本地 CTest 绿、
  CI 却根本没跑它。`test_stall` 曾漏掉，2026-08-29 与 `test_mpu6050` 一并补上。
- `firmware-tests.yml` 在编译前运行 `tools/check_calibration_constants.sh`：它检查 C、C++、
  Python 三处 `EDGES_PER_WHEEL_REV` 均为 **448**，任一处漂移会使 CI 失败。
- `ros2-build.yml` —— 在 `ros:jazzy-ros-base` 容器里 colcon build/test，依赖清单与 `docker/Dockerfile` 保持一致（**改一处要同步另一处**）。

主机单测目标带 `-UNDEBUG`：Release 会定义 `NDEBUG` 把 `assert()` 全部编译掉，测试会"通过"但什么都没验证。别去掉这个标志。

---

## 简历红线

- `0.302 m/s` 是**协议模拟器**结果，不是真实底盘。
- 可以写：协议 / 运动学 / SIL / CI 已验证；真机开环驱动、编码器标定、占空比-转速曲线已实测。
- **不要写**：四轮 PID 已上板、DMA/IDLE 已真机验证、机械臂已完成、任何带载或落地性能数字。

## 提交约定

提交信息只保留项目作者，不添加额外署名 trailer。
