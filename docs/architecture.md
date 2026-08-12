# 系统架构与边界

## 控制分层

```text
Task layer (Pi 5)
  ROS 2 behavior / mobile manipulation state machine
       |                         |
MoveIt 2                    Nav2 / base controller
       |                         |
ros2_control arm HW          existing mecanum HW
       |                         |
USB serial bus              UART protocol
       |                         |
STS3215 servos              STM32 + FreeRTOS
```

Pi 5 负责规划、坐标变换、视觉和任务编排；总线舵机内部闭环负责关节位置；现有 STM32 继续负责底盘电机实时控制。第一版不额外塞一块 STM32 到机械臂控制链路里，否则只会增加协议和调试面。

STM32 可以在后续承担：

- 独立急停和电源监测
- 末端触碰/力敏传感器采集
- 自制夹爪或直流电机的实时控制
- 看门狗和故障上报

## ROS 2 packages

| Package | Responsibility |
| --- | --- |
| `ram_description` | URDF/Xacro、mesh、joint limits、mobile base mount |
| `ram_hardware` | STS3215 `ros2_control` hardware interface |
| `ram_moveit_config` | planning group、collision、kinematics |
| `ram_bringup` | controller、TF、camera 和整机 launch |
| `ram_tasks` | dock、detect、pick、place 状态机 |

## 最小演示任务

先用 AprilTag，而不是直接上 YOLO 或 VLA：

1. 底盘导航到工作台前的预定位点。
2. 相机识别 AprilTag，计算目标在 `base_link` 下的位姿。
3. MoveIt 规划预抓取位姿。
4. 低速接近、闭合夹爪、抬升。
5. 底盘移动到投放区，机械臂释放。
6. 记录规划成功、抓取成功、总耗时和失败原因。

这个任务先把标定、TF、规划、控制和联动暴露出来。跑稳后再替换成普通物体检测或 LeRobot policy。

## 电气边界

- 舵机使用独立 12 V 电源支路，Pi 使用独立 5 V 稳压支路，共地但不共用细线回流。
- 舵机电源入口加保险、急停和足够线径；记录峰值电流后再定车载电池/DC-DC。
- 桌面调通后再上车。先做静态重心和倾覆测试，机械臂伸展时限制底盘运动速度。
- 软件必须有 joint limit、速度限制、通信超时 hold/torque-off 策略。
