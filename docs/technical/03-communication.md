# 通信：UART DMA、帧协议与 NRF24

本文描述Pi↔STM32有线链路与手柄→STM32无线链路。两条链最终进入同一控制系统，但传输介质、驱动模型和错误恢复不同。

## 链路总览

| 链路 | 介质 | 数据 | 接收模型 |
| --- | --- | --- | --- |
| Pi↔STM32 | USART1，921600 8N1 | 轮速命令、心跳、ODOM、传感器状态 | circular DMA + 软件ring + CommTask |
| 手柄→STM32 | NRF24L01 2.4 GHz | 摇杆、按键和链路状态 | 软件SPI轮询 + RemoteTask |

I2C是STM32本地外设总线，不属于Pi↔STM32通信链。它在[感知与定位](04-sensing-localization.md)中说明。

## 二进制帧协议

```text
[0xA5][0x5A][LEN][SEQ][CMD][PAYLOAD...][CRC16 little-endian]
```

| 字段 | 职责 |
| --- | --- |
| 双同步字 | 从噪声或错位字节流中重新找帧头 |
| LEN | 限制并确定payload长度 |
| SEQ | 观察重复、丢帧和顺序 |
| CMD | 区分轮速、急停、心跳、ODOM等消息 |
| CRC16 | 检测传输错误；不提供加密与身份认证 |

协议实现位于[`shared/protocol.c`](../../shared/protocol.c)，由STM32 C和Pi C++共同链接。多字节值采用little-endian；payload结构使用packed，但跨架构扩展时仍应关注浮点格式和对齐约定。

## Pi 发送路径

```text
ros2_control四轮命令
→ proto_encode()
→ 非阻塞write()
→ Linux TTY内核缓冲
→ Pi UART
→ STM32 USART1 RX
```

Pi使用`/dev/ttyAMA0`和termios raw模式。波特率必须映射为`B921600`等`speed_t`常量；把裸整数传给termios曾导致`B0`挂断语义，随后内核发送缓冲填满。

非阻塞`write()`可能只发送部分字节或返回`EAGAIN`。Pi侧对剩余部分做有界重试，避免一次瞬时背压就让`controller_manager`停用硬件，同时也限制对100 Hz控制周期的阻塞时间。

## STM32 RX：两级缓冲

```text
USART1->DR
→ DMA1 Channel5 circular
→ rx_stage[64]
→ RxEvent ISR按CNDTR排出新增区间
→ rx_ring[256]
→ 通知CommTask
→ 状态机逐字节解析
```

DMA只写`rx_stage`，CommTask只读`rx_ring`。ISR是ring的单生产者，只推进`head`；CommTask是单消费者，只推进`tail`。

```text
空：head == tail
满：next(head) == tail
占用：(head - tail + N) % N
```

256字节ring实际可用255字节。满时保留旧数据、丢弃新字节并增加`rx_overflows`。协议状态机之后重新搜索`A5 5A`恢复同步。

## IDLE、HT、TC 与 CNDTR

DMA circular模式到达缓冲区末尾后自动重新从0写入。IDLE、Half Transfer和Transfer Complete都可能触发接收事件，因此一次回调不等于一帧，一帧也可能跨越staging末尾。

项目保存上一次排水位置，通过CNDTR推导当前DMA位置，只复制新增区间。早期在RxEvent回调里重新arm DMA会遇到`HAL_BUSY`竞争，跨staging边界的帧因此损坏；改成持续circular接收后消除了该路径。

## CommTask帧状态机

```text
WAIT_SYNC0
→ WAIT_SYNC1
→ READ_HEADER
→ READ_PAYLOAD
→ READ_CRC
→ proto_decode()
→ robot_handle_command()
```

ISR只做有界复制、指针更新和任务通知。长度检查、CRC和命令分发放在任务上下文，避免不确定执行时间长期占用中断。

零payload心跳曾暴露状态机边界错误：`exp_len==0`时不能进入常规payload累计，否则永远到不了CRC状态。修复后已增加回归测试。

## STM32 TX与缓冲所有权

TX使用DMA1 Channel4 normal模式。`tx_buf`只有一份，发送者必须先获取二值信号量；DMA完成回调才归还信号量。

这保证DMA读取`tx_buf`期间不会被下一帧覆盖。忙时项目选择丢弃新的遥测帧，而不是阻塞实时控制；通信看门狗和上层采样允许偶发遥测丢失。

## UART错误恢复

STM32F1 HAL在DMA接收期间遇到ORE、FE、NE或DMA错误时会停止接收路径。错误回调先按F1要求读取SR/DR清除锁存标志，再重置DMA排水位置并重新arm circular RX。

错误计数`comm_uart_errors`与ring丢弃计数用于区分电气/波特率错误、软件消费不及时和协议CRC失败。修复时不能只看“有没有收到帧”。

## NRF24与软件SPI

车端NRF24使用GPIO位翻转SPI。SPI总线本身没有I2C式ACK位，所以模块是否响应通过寄存器读回判断；复位后的`STATUS=0x0E`是项目使用的健康探针。

NRF24无线协议本身支持Auto-ACK和重发。应区分“SPI总线无ACK”和“NRF24无线Auto-ACK”，不能笼统说NRF24没有ACK。

RemoteTask以20 Hz轮询收包。每个有效包刷新250 ms控制权租约和失联看门狗；手柄断开后，Pi控制权可恢复，电机同时进入安全停止。

## 实现入口

- [`shared/protocol.h`](../../shared/protocol.h)：帧格式与命令定义。
- [`firmware/Core/Src/main.c`](../../firmware/Core/Src/main.c)：DMA回调、ring和CommTask。
- [`rtos_drive_main.c`](../../firmware/Core/HW/rtos_drive_main.c)：USART1与DMA通道配置。
- [`serial_protocol.cpp`](../../ros2_ws/src/mcr_bringup/src/serial_protocol.cpp)：Pi termios与协议收发。
- [`nrf24l01.c`](../../firmware/Core/Src/nrf24l01.c)：车端软件SPI和无线驱动。
- [无线遥控文档](../remote_control.md)：手柄数据包、按键和接线。

## 验证证据

| 结论 | 证据级别 | 当前证据 |
| --- | --- | --- |
| 协议编码、CRC和错位恢复 | `[HOST]` | CTest协议与SIL回归 |
| UART双向链路 | `[SYSTEM]` | 921600、ODOM 50.2 Hz、CRC 0、ACK 12/12 |
| ring溢出后可恢复 | `[SIL]` | flood、噪声排空和后续合法心跳回归 |
| NRF24模块响应 | `[HW]` | 坏模块`0x00`，新模块快/慢SPI均`0x0E` |
| 无线遥控与失联停车 | `[SYSTEM]` | K1、K9、K10、250 ms租约完成基础真机验收 |

## 当前边界

CRC只能检错，协议没有加密、签名或对端认证。NRF24基础遥控已工作，但没有完成射频干扰环境、距离、丢包率和长期稳定性量化。

历史洪流测试出现过Cortex-M3 lockup；可模拟的ring和解析路径已有回归，但真实DMA错误中断重入下的lockup根因尚未完整复现定位。

## 面试追问

1. DMA staging与软件ring为什么分开？
2. IDLE事件为什么不能直接当作协议帧边界？
3. ring满时为什么选择丢新数据？
4. TX DMA为什么需要信号量保护缓冲区？
5. SPI无ACK时怎样证明NRF24真的响应？
