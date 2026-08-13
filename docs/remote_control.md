# 无线遥控 (NRF24L01) / Wireless Remote Control

> 当前证据边界：机器人端驱动、摇杆映射和单元测试已完成，NRF24L01 实物收发及电机联动尚未验收。

让麦克纳姆轮机器人支持 **2.4GHz 无线手柄遥控** —— 全向运动控制。遥控器端复用江协科技(江科大)平衡车教程的遥控器硬件与固件;机器人端(STM32)新增 NRF24L01 接收驱动、摇杆→全向运动解析和麦克纳姆逆运动学。

Adds **2.4 GHz wireless hand-held remote control** with omnidirectional motion. The controller side reuses the 江协科技 (Jiangxie) balance-car remote hardware + firmware; the robot side (STM32) gains an NRF24L01 receiver driver, joystick→omnidirectional mapping, and mecanum inverse kinematics.

---

## 硬件 / Hardware

| 部分 Part | 说明 Description |
| --- | --- |
| 遥控器 Controller | 江协科技遥控器:STM32F103 + 双摇杆(4×ADC)+ 9 键 + 0.96" OLED + NRF24L01 |
| 接收模块 Receiver | NRF24L01+ 模块,接在机器人的 STM32 上,3.3V 供电 |

### 接线 / Wiring (机器人端 STM32)

NRF24L01+ 模块 ←→ STM32(GPIO 位带操作 / bit-banged SPI):

| NRF24L01 引脚 | STM32 GPIO |
| --- | --- |
| CE | PA8 |
| CSN | PA15 |
| SCK | PB3 |
| MISO | PB4 |
| MOSI | PB5 |
| VCC | 3.3V |
| GND | GND |

> PA15 / PB3 / PB4 是 JTAG 引脚,驱动初始化时通过 `__HAL_AFIO_REMAP_SWJ_NOJTAG()` 释放为普通 GPIO。引脚在 `firmware/Core/Src/nrf24l01.c` 顶部以宏定义,换板子只需改那里 + 同步 `gpio_init()`。

---

## 射频协议 / Radio Protocol

两端必须完全一致,否则收不到(直接沿用江科配置):

| 参数 Parameter | 值 Value |
| --- | --- |
| 地址 Address | `{0x11, 0x22, 0x33, 0x44, 0x55}` (5 字节) |
| 信道 Channel | 2.402 GHz (RF_CH = 0x02) |
| 速率 Data rate | 2 Mbps,0 dBm |
| 包长 Payload | 固定 32 字节 (static) |
| 自动应答 Auto-ACK | 开启 (Enhanced ShockBurst) |
| CRC | 1 字节 |

## 数据包格式 / Packet Format

发送频率 100 ms (10 Hz),前 6 字节有意义(协议与江科遥控器一致):

| Byte | 含义 Meaning | 范围 Range |
| --- | --- | --- |
| 0 | ID (0x00 = 遥控数据) | 固定 0x00 |
| 1 | 左摇杆横向 LH (strafe) | -100 ~ +100 |
| 2 | 左摇杆纵向 LV (forward/back) | -100 ~ +100 |
| 3 | 右摇杆横向 RH (rotate) | -100 ~ +100 |
| 4 | 右摇杆纵向 RV (reserved) | -100 ~ +100 |
| 5 | 按键键码 KEY | 1/9,无按键为 0 |

摇杆值已由遥控器端做过回中死区处理(±100 内归零),见遥控器 `AD.c` 的 `DataProcess()`。

## 摇杆映射 / Joystick Mapping

双摇杆 → 全向运动 (Twist):

| 摇杆 Joystick | 运动 Motion | 限幅 Limit |
| --- | --- | --- |
| 左摇杆纵向 LV | 前后平移 vx | ±0.6 m/s |
| 左摇杆横向 LH | 左右横移 vy | ±0.6 m/s |
| 右摇杆横向 RH | 原地旋转 omega | ±2.5 rad/s |
| 右摇杆纵向 RV | 保留 (reserved) | — |

映射到 4 个麦克纳姆轮的目标角速度,由固件内 C 版逆运动学计算
(`firmware/Core/Src/mecanum_ik.c`,与 ROS2 侧 `MecanumKinematics` 公式一致):

```
w1(FL) = (vx - vy - (lx+ly)·ω) / R
w2(FR) = (vx + vy + (lx+ly)·ω) / R
w3(RL) = (vx + vy - (lx+ly)·ω) / R
w4(RR) = (vx - vy + (lx+ly)·ω) / R
```

## 按键功能 / Key Mapping

| 键 Key | 功能 Function |
| --- | --- |
| K1 | 遥控启停切换:开启后摇杆接管电机速度环;关闭时目标速度归零 |
| K9 | 紧急停止:等效于 UART 紧急停止命令,立即刹车 |

遥控器上的 OLED 显示信号强度(Sig)和各摇杆值,方便调试。

## 固件集成 / Firmware Integration

机器人端新增/修改:

| 文件 File | 作用 Purpose |
| --- | --- |
| `firmware/Core/Inc|Src/nrf24l01.c/h` | NRF24L01 接收驱动(软件 SPI,轮询),配置与江科一致 |
| `firmware/Core/Inc|Src/mecanum_ik.c/h` | 麦克纳姆逆运动学(纯 C) |
| `firmware/Core/Inc|Src/remote_control.c/h` | 包解析 + 摇杆→全向映射(纯 C) |
| `firmware/Core/Src/robot_control.c` | 新增 `robot_set_target_wheels()`:写 4 轮目标速度并刷新通信看门狗 |
| `firmware/Core/Src/main.c` | 新增 `RemoteTask`:20 Hz 轮询 NRF24L01,驱动遥控逻辑 |

`RemoteTask` 优先于 `CommTask` 之外的电机命令源,与 UART 命令互不冲突 —— 谁最近发命令谁生效。遥控包到达即刷新看门狗,失联超过 100 ms 会触发急停(安全默认)。

遥控器工程(完整 Keil 工程,可直接编译烧录)放在 `firmware/remote_controller/`。

## 使用流程 / Usage

1. 用 Keil 打开 `firmware/remote_controller/Project.uvprojx`,编译烧录到遥控器 (STM32F103C8T6)
2. 机器人端按上面接线接好 NRF24L01+,上电
3. 按遥控器 **K1** 启用遥控 → 推摇杆,机器人全向移动(前后/横移/旋转)
4. 按 **K1** 停用;按 **K9** 紧急停止

## 来源 / Source & Credits

遥控器硬件与固件来自江协科技(江科大)STM32 入门教程《平衡小车》工程 —— 详见
`robot-vacuum-lab` 课程资料中「遥控器程序」与「平衡车程序/08-遥控器控制平衡车」。
本项目仅移植了机器人端接收逻辑并适配 HAL/FreeRTOS,射频协议参数完全沿用,保证与现成遥控器实物互通。

---

*License: MIT(本项目部分)。遥控器固件版权归江协科技所有,仅作学习参考。*
