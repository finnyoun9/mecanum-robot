# 实施路线

## Stage 0：购买前

- 获取准确 SKU 清单、舵机型号、STM32 型号和源码目录截图。
- 确认反馈项、总线协议和电源参数。
- 保存商品页和客服答复，作为硬件基线。

通过条件：没有“智能总线舵机但协议不公开”或“源码实际只有库/HEX”的信息缺口。

## Stage 1：结构入门

- 清点散件，为支架、舵机、轴承、舵盘和螺丝编号。
- 分关节装配并绘制简化运动链。
- 测量连杆中心距、关节活动范围、夹爪开口和整机重量。
- 记录装配方向、机械干涉、回差、重心和走线问题。
- 在 `hardware/` 保存照片、尺寸表及后续小车安装板约束。

通过条件：不通电状态下完成结构检查，明确每个自由度的转轴、正方向和机械限制。

## Stage 2：单舵机与协议

- 使用厂商工具确认 ID、零位和安全角度。
- 用逻辑分析仪抓取一组读写帧，核对波特率、帧格式和校验。
- 实现最小 C 驱动：ping、read position、write position、error handling。
- 记录反馈频率、延迟、超时与重试行为。

通过条件：脱离厂商上位机，可以稳定控制一个舵机并读取状态。

## Stage 3：STM32 + FreeRTOS

- 使用 HAL 初始化 UART/DMA、GPIO 和 watchdog。
- 实现 `servo_bus_task`、`motion_task`、`safety_task` 和 `host_comm_task`。
- 使用静态/有界内存；记录 flash、RAM、heap 和各任务 stack 水位。
- 注入 checksum 错误、断线、重复 ID 和主机超时。

通过条件：六轴在软限位内同步动作，故障能进入确定的安全状态，逻辑分析仪下无漏帧失控。

## Stage 4：机械与控制表征

- 标定关节零位和正方向。
- 建立正运动学和工作空间模型。
- 测量不同姿态下回差、重复定位误差、温升和电流。
- 评估简单关节插值与厂商动作组的差异。

通过条件：所有结果有 CSV/波形，不使用“运行正常”代替测量。

## Stage 5：ROS 2 / MoveIt 2

- 创建 URDF/Xacro、joint limits 和 TF。
- 实现 C++ `ros2_control SystemInterface`。
- 接入 `JointTrajectoryController` 和 MoveIt 2。
- 对比 commanded/feedback joint state 并记录 rosbag。

通过条件：RViz 与实机状态一致，规划轨迹能够安全执行和取消。

## Stage 6：上车联动

- 设计机械臂安装板、电源支路和线束固定。
- 标定 `base_link -> arm_base -> camera -> tool0`。
- 完成“导航到位—AprilTag 定位—抓取—运送—放置”状态机。
- 连续运行 10 次，统计成功率、耗时和失败原因。

通过条件：README 提供架构、视频、数据和失败复盘，能够围绕 STM32、RTOS、总线和系统调试回答面试追问。

## Optional v2

- 增加 CAN/CAN-FD 或 RS485 转接实验，贴近机器人关节通信岗位。
- 评估 ESP32 控制器作为通信吞吐和开发效率对照。
- 经典控制链稳定后，再研究 LeRobot 数据格式或模仿学习适配；不影响 v1 求职主线。
