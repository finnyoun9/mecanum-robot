# ROS2 学习笔记

> 学习资源：[【古月居】古月·ROS2入门21讲 | 带你认识一个全新的机器人操作系统](https://www.bilibili.com/video/BV16B4y1Q7jQ/)
>
> 讲师：古月居GYH | 发布：2022-06-13 | 播放：105万+ | 官网：[guyuehome.com](https://guyuehome.com)

---

## 课程目录（21讲）

| # | 主题 | 笔记 |
|---|------|------|
| 1 | ROS和ROS2是什么 | |
| 2 | ROS2对比ROS1 | |
| 3 | ROS2安装方法 | |
| 4 | ROS2命令行操作 | |
| 5 | ROS2开发环境配置 | |
| 6 | 工作空间与功能包：开发过程的大本营 | [↓](#6-工作空间与功能包) |
| 7 | 节点：机器人的工作细胞 | [↓](#7-节点) |
| 8 | 话题：节点间传递数据的桥梁 | [↓](#8-话题) |
| 9 | 服务：节点间的你问我答 | [↓](#9-服务) |
| 10 | 通信接口：数据传递的标准结构 | [↓](#10-通信接口) |
| 11 | 动作：完整行为的流程管理 | |
| 12 | 参数：机器人系统的全局字典 | |
| 13 | 分布式通信：多计算平台的任务分配 | [↓](#13-分布式通信) |
| 14 | DDS：机器人的神经网络 | |
| 15 | Launch：多节点启动与配置脚本 | |
| 16 | TF：机器人坐标系管理神器 | |
| 17 | URDF：ROS机器人建模方法 | |
| 18 | Gazebo：三维物理仿真平台 | |
| 19 | Rviz：三维可视化显示平台 | |
| 20 | RQT：模块化可视化工具 | |
| 21 | ROS2应用与进阶攻略 | |

---

## 核心概念（结合本项目理解）

### 6. 工作空间与功能包

**工作空间** = 整个项目的"工地"，所有开发活动都在这个目录里。

```
ros2_ws/
├── src/       ← 源码：功能包都放这
├── build/     ← 编译中间产物
├── install/   ← 编译后的可运行产物
└── log/       ← 构建日志
```

关键操作：`colcon build` → `source install/setup.bash` → `ros2 run`

**功能包** = 按职责拆分的代码模块，本项目的 3 个包：

| 包 | 职责 |
|----|------|
| `mcr_description` | URDF 模型：机器人长什么样 |
| `mcr_bringup` | 硬件驱动：串口通信 + ros2_control + 运动学 |
| `mcr_navigation` | 导航：SLAM 建图 + 路径规划 |

依赖关系：`mcr_navigation` → `mcr_bringup` + `mcr_description`

---

### 7. 节点

**节点 = ROS2 最小运行单元，每个节点只做一件事**（像生物体的细胞各司其职）。

本项目 `robot.launch.py` 启动的 6 个节点：

```
robot_state_publisher    → URDF → TF 坐标广播
controller_manager      → 管理所有控制器的生命周期
joint_state_broadcaster → 发布 4 个轮子的位置/速度
mecanum_drive_controller → 接收 /cmd_vel，解算麦克纳姆轮运动学，发串口指令
imu_sensor_broadcaster  → 发布 IMU 姿态/角速度
ekf_filter_node         → 融合轮式里程计 + IMU，输出 /odometry/filtered
```

节点之间不直接调用函数，通过话题传递消息。单个节点崩溃不影响其他节点。

---

### 8. 话题

**话题 = 广播电台模型**：发布者对着话题喊，订阅者调到同一个话题就能收到。发布者不知道谁在听，订阅者不知道谁在发。

本项目的话题数据流：

```
/cmd_vel                               Nav2 / 遥控  →  mecanum_drive_controller
/joint_states                          joint_state_broadcaster → robot_state_publisher
/mecanum_drive_controller/odom         mecanum_drive_controller → ekf_filter_node
/imu_sensor_broadcaster/imu            imu_sensor_broadcaster → ekf_filter_node
/odometry/filtered                     ekf_filter_node → Nav2 / RViz
```

特点：持续单向流、无连接、一对多。适合传感器数据和速度指令。

---

### 9. 服务

**服务 = 电话模型**：一问一答，调用一次返回一次，结束。

与话题的对比：

| | 话题 | 服务 |
|---|---|---|
| 方向 | 单向发布→订阅 | 双向请求⇄响应 |
| 持续性 | 持续不断 | 一次调用 |
| 适合 | 传感器数据、速度指令 | 开关、配置、查询状态 |

本项目目前没有自定义服务，但基础设施大量使用（如 `controller_manager` 的 load/switch controller 调用）。未来场景："切换爬坡模式"、"校准IMU"用服务更合适。

---

### 10. 通信接口

**通信接口（`.msg` / `.srv` / `.action`）= 发布者和订阅者之间的"合同"**。双方约定好消息里有哪些字段、什么类型。

本项目使用的标准消息类型：

| 消息类型 | 用于 | 话题 |
|----------|------|------|
| `geometry_msgs/msg/Twist` | 速度指令 | `/cmd_vel` |
| `nav_msgs/msg/Odometry` | 里程计 | `/mecanum_drive_controller/odom` |
| `sensor_msgs/msg/Imu` | IMU | `/imu_sensor_broadcaster/imu` |
| `sensor_msgs/msg/JointState` | 关节状态 | `/joint_states` |
| `sensor_msgs/msg/LaserScan` | 激光扫描 | `/scan` |

内部结构体（如 `Twist2D`、`WheelSpeeds`）不能直接在话题上传——只有标准 `.msg` 类型才能被所有节点识别。这是为什么 `mecanum_drive_controller` 输出的里程计能被 `robot_localization`（第三方包）直接订阅。

---

### 13. 分布式通信

**ROS2 天生分布式**：没有中心服务器（ROS1 有 roscore），节点通过 DDS 自动发现。

本项目架构：

```
MacBook (调试)                      树莓派 5 (机器人本体)
┌──────────────┐                   ┌──────────────────────────┐
│ RViz2        │                   │ Docker 容器 (--network host) │
│ teleop 遥控   │                   │ controller_manager       │
└──────┬───────┘                   │ mecanum_drive_controller  │
       │                           │ ekf_filter_node          │
       └──── 同一WiFi，DDS自动发现 ──┤                          │
                                   │      ↓ 串口              │
                                   │   STM32 固件             │
                                   └──────────────────────────┘
```

关键：`docker/run.sh` 中 `--network host` 让 DDS UDP 组播穿透容器，外部设备直接看到容器内的节点。

- **UDP 组播**：无连接、低开销、无拥塞控制、不保证可靠——恰好适合传感器实时数据流（丢一帧无所谓，下一帧马上到）
- 在 Mac 上可以直接 `ros2 topic list` / `ros2 topic echo` 看到树莓派上的实时数据

---

### 17-20. 工具链：URDF / Gazebo / RViz / RQT

四个工具在本项目中的使用情况：

| 工具 | 用途 | 项目状态 |
|------|------|----------|
| **URDF** | 机器人建模——物理尺寸、坐标系树、质量惯量、硬件接口定义 | ✅ 已完整使用。`mcr_description/urdf/mcr.urdf.xacro` 定义了 4 轮 + 5 传感器的完整模型 |
| **RViz** | 3D 可视化——实时显示机器人位姿、激光点云、导航路径、TF 坐标系 | ✅ 已集成。`navigation.launch.py` 自动启动，配置文件在 `mcr_navigation/config/nav2_default_view.rviz` |
| **Gazebo** | 物理仿真——虚拟测试场，先仿真验证算法再上真机 | ⚠️ 之前在其他项目跑通过完整链路（URDF + Gazebo + SLAM + Nav2），本项目暂未加配置 |
| **RQT** | 调试工具箱——`rqt_graph` 看拓扑、`rqt_plot` 画曲线、`rqt_tf_tree` 看 TF 树 | ❌ 运行时可随时用，不需要预先配置 |

**四者关系**：URDF 是图纸 → Gazebo 是虚拟测试场 → RViz 是实时监控 → RQT 是调试工具箱。

### Mac 远程调试 RViz2

macOS 没有原生 ROS2，通过 **SSH + X11 Forwarding** 把树莓派的 RViz 画面投射到 Mac：

```
Mac (XQuartz)  ←── SSH -X ──→  树莓派 (mcr-ros2:jazzy-gui)
  显示 RViz 窗口                    跑 RViz2 + 所有节点
```

**方案 A：SSH X11 Forwarding（不可行）**

OGRE 3D 引擎无法通过远程 X11 创建 GLX 渲染窗口，RViz2 启动崩溃。

**方案 B：Xvfb + VNC（当前方案 ✅）**

容器内虚拟显示器渲染，VNC 投射到任意电脑。

```bash
# 1. (一次性) 容器内安装 VNC server
ssh pi@<IP> 'docker exec mcr_ros2 apt-get update && docker exec mcr_ros2 apt-get install -y x11vnc'

# 2. 启动虚拟显示器 + RViz2 + VNC
ssh pi@<IP> 'docker exec -d mcr_ros2 Xvfb :99 -screen 0 1280x800x24 +extension GLX'
ssh pi@<IP> 'docker exec -d -e DISPLAY=:99 -e LIBGL_ALWAYS_SOFTWARE=1 mcr_ros2 bash -c "source /opt/ros/jazzy/setup.bash && rviz2 -d /ros2_ws/src/mcr_navigation/config/nav2_default_view.rviz"'
ssh pi@<IP> 'docker exec -d mcr_ros2 x11vnc -display :99 -forever -shared -nopw -nossl -nounixpw -rfbport 5901 -o /tmp/x11vnc-5901.log'

# 3. VNC 客户端连接 192.168.0.116:5901
#    Mac: open vnc://192.168.0.116:5901
#    Win: TigerVNC / RealVNC / TightVNC → 192.168.0.116:5901
```

#### VNC 端口陷阱（2026-08-11 实机验证）

Raspberry Pi OS 桌面自带的 `wayvnc` 已占用宿主机 `5900`。容器使用
`--network host`，因此容器内的 `x11vnc` 也不能绑定 `5900`。如果客户端连
`192.168.0.116:5900`，实际连到的是 `wayvnc`，会出现账号密码提示或
`No matching security types`，而不是 ROS2 的 Xvfb 画面。

固定约定：

- `5900`：Raspberry Pi OS 的 `wayvnc`，不要用于本项目 RViz2。
- `5901`：容器内 `Xvfb :99` 对应的 `x11vnc`，当前 RViz2 远程入口。
- TigerVNC 连接：`vncviewer -SecurityTypes None 192.168.0.116:5901`。
- `x11vnc` 使用 `-nossl -nounixpw -nopw`，局域网内免账号密码；不要暴露到公网。

启动前先查端口，避免再次误连：

```bash
# 预期看到 wayvnc 占用 5900，x11vnc 占用 5901
ssh pi@192.168.0.116 'sudo ss -ltnp | grep -E ":5900|:5901"'

# 查看 x11vnc 启动失败、端口冲突等日志
ssh pi@192.168.0.116 'docker exec mcr_ros2 tail -50 /tmp/x11vnc-5901.log'
```

---

## 学习记录

### 环境信息

- OS: macOS (Darwin)
- ROS2 版本:
- 安装方式:

### 笔记

<!-- 按学习进度在此记录 -->
