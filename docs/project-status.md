# 项目状态与协作基线

> **最后更新：2026-08-28。** 改动了真机状态、固件结构或标定常数之后，**请同步更新本文档**——多人或多端并行推进时，过时的基线会让其他开发者基于错误前提做出判断。

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
- **新普通 NRF24L01+ 尚未建立 SPI 回读。** 模块 `VCC/GND` 测得 3.3 V，模块旁已并联
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
- **未完成：慢速 SPI 读数尚未取到（实测受阻）。** 探针跑到后段时 PC13 观察到持续慢闪，
  与预期的阶段化闪烁不符，怀疑卡在某阶段或存在探针自身逻辑缺陷；本机 `st-info` 缺失、
  `arm-none-eabi-gdb` 不可用（`exit 127`），OpenOCD `mdw` 亦未回读到 stdout，
  因此慢速 `STATUS` 值**目前没有可信读数**。下一步应经 USART1/PA9 接 USB-TTL 读文本输出，
  并给探针各阶段加超时退出，再决定是否换模块。

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
  **SLAM（`/scan` + TF → `/map`）尚未跑通**，是下一步（P3）。
- **IMX219 CSI 相机可用。** Pi 识别为 3280×2464、10-bit RGGB；640×480 `RGB888`
  连续采集实测 30.56 FPS，静态拍照正常。宿主机以 `Picamera2 + ONNX Runtime 1.23.2`
  运行仓库 `yolov8n.onnx`，20 帧连续测试平均推理 163.8 ms、端到端 6.05 FPS；当前场景
  两个显示器在 20 帧中共得到 38 个 `tv` 检测框，最高置信度 0.741，另一次人体局部检测
  置信度 0.883。该结果证明 Pi 5 CPU 可承担低速视觉检测，不代表已完成 ROS 2 相机节点。
- **仓库 CSI 入口仍待实现。** `perception/detection/yolo_detect.py` 目前使用
  `cv2.VideoCapture(0)`、相对路径 `code/yolov8n.onnx` 和 `cv2.imshow()`，不适合当前
  IMX219/libcamera + headless 部署；本次仅完成独立真机验证，未把临时测试脚本提交进仓库。

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
- CTest 6/6 通过；`firmware/arm_controller` SIL 24/24 通过。
- UART DMA/IDLE 代码路径已实现（staging buffer 与软件 ring 分离），**真机未验证**。
- UART 模拟器轮速对象模型使用台面供电测得曲线的推导上限 4.27 rev/s，以及由
  5% 不转、10% 时 0.32 rev/s 推导的保守启动阈值；它不是电池供电或带载实测模型。
  模拟器轮半径已同步为实测 0.030 m。该静态模型的主机测试已通过，ROS 容器
  `verify_sil.sh` 尚待具备 Docker 的环境复核。

### 未做 / 未验证

- **遥控开环验收已完成**：K1 锁存、前后/横移/旋转、K9 急停和手柄断电失联停车均已真机验证；精确失联停车延迟尚未仪器测量。
- 带载（落地）定量转速、堵转电流、阶跃响应指标。**M2 已完成**（`Kp=100 / Ki=300`，空载，台面与电池供电下均已验证）。
- **M3 四轮闭环**：基础动作、K9 急停和 250 ms 失联停车已验收；尚未做 30 分钟连续运行、
  量化的落地带载速度/跟踪误差。四轮电池供电扫描的差异为 4.9%，RR
  最慢，后续若出现偏航应以分轮记录为依据调参。
- `lx = 0.10 m`（半轴距）和 `ly = 0.12 m`（半轮距）仅为默认估计值，均未实测；
  应量前后轮、左右轮的轴中心距后各除以 2，再替换 ROS 2 与模拟器中的默认值。
- `firmware_arch_main()` **不能在真机跑**：缺 UART/I2C 的 MSP 初始化；`motor.c` 引脚映射目前由各 HW target 在初始化时传入，不是静态表。
- IMU、ToF、Nav2 尚未上真机。LD06 已接入 `robot.launch.py`（见上文），但仍未与真实里程计、
  TF 闭环和 SLAM 形成整车链路；IMX219/YOLO 只完成 Pi 侧独立验证，仓库内仍无 CSI 相机节点。
  NRF24L01 曾完成真机验收，当前车端模块已故障下线，不能写成当前可用。
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

| 用途 | 序列号 | 固件 |
| --- | --- | --- |
| 小车 | `37FF71064E57343623CE1E43` | V2J46S7 |
| 手柄 | `37FF71064E57343602361C43` | V2J27S6 |

⚠️ 两个 ST-Link 报告的 flash 容量不同（65536 vs 67107840），**这是固件版本差异不是故障**。判据仍是"多次读数是否一致"：手柄那个连测三次都是 67107840，稳定即正常。

### 2026-08-30：ST-Link 软复位后 I²C 传感器挂零 — 已复现修复

codex 的下地前架空复测发现：ST-Link 软复位后，共用 I²C 的 MPU6050 和 ToF 同时输出全零，
四轮闭环/ROS `/cmd_vel`/RR 堵转锁存/串口链路（50 Hz、CRC 0）均已通过。按建议对整车传感器
侧彻底断电 3–5 秒再上电后复测：`imu_q` 四元数持续变化、`imu_gyro` 噪声在静止态合理范围，
ToF 读数 550–586 mm 连续变化，0 次 `ERR_TOF_TIMEOUT`——**确认是复位时序问题，不是硬件故障，
断电重上电即可恢复**。

复测过程中顺带发现 `tools/imu_watch.py`、`tools/tof_watch.py` 两个诊断脚本在磁盘上是空文件
（全 0 字节，未提交到 git），已按 `shared/protocol.h` 的 `odom_feedback_t` 布局重建（复用
`link_check.py` 已验证过的帧解析）。

⚠️ 新发现、未解决：复测期间 `error_flags` 在 `0x00` 和 `0x08` 之间间歇跳变（约一半帧）。
`protocol.h` 目前只正式定义到 `ERR_TOF_TIMEOUT = 0x04`；`link_check.py` 里本地写了个
`ERR_TOF_INVALID = 0x08` 但共享头文件没有这个宏，说明固件端可能已加了这个错误位、
头文件没跟上。同一时段 ToF 单独复测显示读数正常、`err=0x00`，暂看不出实际影响，但
`0x08` 的含义和 header 是否要补定义待确认。

**M3 通过条件尚未达成，下地前不建议直接切自由跑**：
- ~~RR 轮增益从未单独确认过~~ — 2026-08-30 已确认，见下一节，根因是 PID 调参不是硬件。
- 带载（落地）堵转电流、阶跃响应、带载跟踪误差**仍未实测**——现有 PID 参数都是空载/架空调的。
- M3 官方通过条件"四轮闭环持续 30 min 无失控"只达成了"基本运动子集"。
- 当前供电是 DP100 台式电源（不是电池），电流限制远低于 3S 40C 电池，带载堵转特性未知时
  用限流电源先测更安全。

建议先在 DP100 供电、人员值守下做低速短距（如 0.5 m）落地测试，盯 RR 轮是否滞后/过热，
确认正常后再逐步加时长，最后才换电池做无绳测试。

### 2026-08-30（续）：RR "偏慢+咔嚓" 根因是 PID 调参，不是硬件；顺带修了一个真实的急停状态 bug

延续上一节的架空复测，`drive_check.py --spin 2.5` 悬空测试四轮时最初**全部 0 edges**（不止 RR），
排查发现 `robot_control.c` 的 `robot_init()` 只锁了 `g_robot.comm_stop_latched`，没有同步锁
`g_robot.emergency_stop_active`，导致它和 `motor.c` 自己的 `emergency_stopped` 锁状态不同步——
`CMD_VEL_CTRL` 的"首次活动指令解锁"逻辑因此判断条件恒假，锁永远解不开。已修复（一行 diff，
`robot_init()` 里补 `g_robot.emergency_stop_active = true;`，让开机态正确等价于一次
comm-watchdog trip，注释里本来就是这个意图）。**但这不是这次挡住测试的直接原因**——
`drive_check.py` phase 1 的心跳间隔天然会在 100 ms 内触发一次真实 watchdog trip，
两个标志本来就会被那次 trip 正确同步；真正挡住的是下面这条。

真正原因：`tools/drive_check.py --tune-pid` 用的是 M2 单轮空转标定时的猛药参数
`Kp=100/Ki=300`。`remote_pid_drive_main.c`（无线遥控验收通过的固件）里第 27 行注释已经
明确写着"M2's Kp=100/Ki=300 were valid for one isolated FR step test, but overdrive the
four-wheel plant"——换到四轮/RR 上纯 PI 直接过冲震荡，实测 RR 单轮测试从 1.45 → 0.84 → 1.58 →
1.73 rad/s 抖动不收敛，物理上表现为"转得慢+来回咔嚓"（新装减速箱有齿隙，配合过冲震荡更明显）。
插拔 RR 接线**没有解决问题**（甚至一度更差），排除了接触不良假设。

换成验收通过的 `Kp=15/Ki=35/Kd=0`（`remote_pid_drive_main.c` 里的验证值，`integral_limit`
按 `out_max/Ki≈28.57` 推导）后，震荡消失，四轮转速收敛到 **1.66–1.73 rad/s**，彼此差距
收窄到 4% 以内——**RR 本身没有硬件问题**，之前 4.9%/更大的轮间差距是震荡状态下的假象。

⚠️ 新待办：四轮此时只到目标 2.5 rad/s 的 66–69%，比 `Kp=100/Ki=300` 时 FL/FR/RL 能到的
2.3–2.41 更低。原因是 `remote_pid_drive` 的验证方案里**大部分驱动力来自前馈**
（`speed_feedforward()`：按实测的占空比-转速曲线把目标速度直接换算成 duty，PI 只修轮间差异），
而 `rtos_drive`/`robot_control.c` 当前的 `CMD_VEL_CTRL` 路径**没有移植这个前馈项**，纯 PI 在
4 秒测试窗口内还没爬到目标附近。要不要把 `speed_feedforward()` 移植进 `robot_control.c`
是下一步要做的决定，现在还没做。

诊断过程中还顺带发现并修复了两个空文件（`tools/imu_watch.py`、`tools/tof_watch.py` 磁盘上
是全 0 字节，未提交到 git）和 `tools/drive_check.py` 结尾被截断缺失的 `sys.exit(main())`——
都是同一类"写入中途被打断"的旧问题，已按 `shared/protocol.h` 的 `odom_feedback_t` 布局
重建，复用 `link_check.py` 的帧解析。新增 `tools/rr_only_check.py`：只命令 RR、其余三轮
锁 0，用于隔离单轮问题；支持 `--kp/--ki/--kd/--integral-limit` 传参复测不同增益组合。

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
