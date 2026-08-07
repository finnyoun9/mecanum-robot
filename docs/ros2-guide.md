# ROS2 扫地机学习路线

## 一、ROS2 入门路线

### 1. 为什么需要 ROS

ROS 本质上是一个**分布式通信框架**，解决机器人各模块之间怎么"对话"的问题：

- 激光雷达节点发布 `/scan` 话题
- SLAM 节点订阅 `/scan`，发布 `/map` 和 `/pose`
- 导航节点订阅 `/map` + `/pose`，发布 `/cmd_vel` 控制底盘

扫地机的软件架构天然适合 ROS 的**节点化、话题订阅**模式。

### 2. 推荐学习资源

---

**B站教程（按学习顺序，播放量已核实）：**

| 优先级 | 教程 | 播放量/影响力 | 说明 |
|-|-|-|-|
| 首选 | 小鱼/FishROS《ROS 2机器人开发从入门到实践》 | 🔥 单集最高 **545.5万** 播放，全系列 **6万+** 关注 | 最全 ROS2 中文教程：10章覆盖入门→节点→话题→服务→仿真→导航→真机实战；配套书籍+社区 fishros.org.cn |
| 次选 | 《手把手学ROS2 Humble》 | **20.3万** 播放 | 偏实战：SLAM、Navigation2、RTABMap、Pixhawk 等 |
| 进阶 | ROS2 Navigation2 介绍与入门 | **20.4万** 播放 | 导航栈专项深入 |
| 经典 | 古月居《ROS2入门21讲》 | 国内 ROS2 布道者经典系列 | 概念讲解清晰，适合零基础 |

---

**GitHub 项目（按 Star 数排序，数据 2025/05 核实）：**

**核心框架 & 工具链：**

| 项目 | Stars | 为什么推荐 |
|-|-|-|
| [ros2/ros2](https://github.com/ros2/ros2) | ⭐ **5,549** | ROS2 官方核心仓库，元操作系统 |
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | ⭐ **4,295** | ROS2 官方导航栈，扫地机核心依赖 |
| [cyberbotics/webots](https://github.com/cyberbotics/webots) | ⭐ **4,379** | 开源机器人仿真器，Gazebo 替代方案 |
| [Ly0n/awesome-robotic-tooling](https://github.com/Ly0n/awesome-robotic-tooling) | ⭐ **3,820** | 机器人专业工具大全（C++/Python/ROS） |
| [introlab/rtabmap](https://github.com/introlab/rtabmap) | ⭐ **3,792** | RTAB-Map：视觉+激光 SLAM，闭环检测强 |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | ⭐ **2,489** | 工业级 2D SLAM，扫地机公司广泛使用 |
| [henki-robotics/robotics_essentials_ros2](https://github.com/henki-robotics/robotics_essentials_ros2) | ⭐ **2,347** | ROS2 + Gazebo 动手教程，适合跟练 |
| [fkromer/awesome-ros2](https://github.com/fkromer/awesome-ros2) | ⭐ **2,108** | ROS2 资源精选列表（文档/教程/工具） |
| [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | ⭐ **1,352** | Gazebo 最新版，ROS2 默认仿真器 |

**移动机器人 & 嵌入式：**

| 项目 | Stars | 为什么推荐 |
|-|-|-|
| [linorobot/linorobot2](https://github.com/linorobot/linorobot2) | ⭐ **900** | 自主移动机器人（差速/4WD/麦克纳姆），硬件+ROS2 完整方案 |
| [micro-ROS/micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) | ⭐ **493** | 把 ROS2 跑在 MCU（STM32/ESP32）上，底盘控制必备 |
| [fishros/fishbot](https://github.com/fishros/fishbot) | ⭐ **298** | 小鱼 ROS2 课程配套源码，机器人建模仿真 |

**扫地机专项：**

| 项目 | Stars | 为什么推荐 |
|-|-|-|
| [yuanzhongqiao/awesome-vacuum](https://github.com/yuanzhongqiao/awesome-vacuum) | 精选列表 | 扫地机器人开源软硬件资源合集：SLAM、路径规划、硬件参考 |
| [dinishsk/autonomous-mobile-vacuum-cleaning-robot](https://github.com/dinishsk/autonomous-mobile-vacuum-cleaning-robot) | ⭐ 15 | 完整开源扫地机项目：树莓派+Arduino+RPLiDAR+Nav2 |

### 3. 学习步骤建议（8-12 周）

**第1-2周：ROS2 核心概念**

- 节点(Node)、话题(Topic)、服务(Service)、动作(Action)
- 用 turtlesim 跑通第一个 demo
- 理解 launch 文件和参数系统
- 推荐跟练：古月居《ROS2入门21讲》或 小鱼 第1-3章

**第3-4周：仿真环境**

- Gazebo + ROS2 搭建扫地机模型（URDF/XACRO）
- 添加激光雷达、IMU、里程计传感器插件
- 用 teleop 键盘控制虚拟扫地机走动
- 推荐跟练：fishros/fishbot 项目

**第5-6周：SLAM & 导航**

- slam_toolbox 跑通建图
- Navigation2 跑通路径规划
- 理解 costmap、全局/局部规划器
- 推荐跟练：robotics_essentials_ros2 第7-8章

**第7-8周：硬件对接**

- micro-ROS 把底盘驱动接入 ROS2
- 传感器 driver 编写（激光雷达、陀螺仪、悬崖传感器）
- 里程计融合（wheel odom + IMU → EKF）
- 推荐参考：linorobot/linorobot2 硬件方案

---

## 二、从 0 构建扫地机的技术栈

### 硬件层

- **主控**: Raspberry Pi 4/5 或 NVIDIA Jetson Nano（跑 ROS2）
- **底盘 MCU**: STM32 / ESP32（跑 micro-ROS 或裸机 FreeRTOS）
- **传感器**: RPLIDAR A1/A2（激光雷达）、MPU6050/9250（IMU）、轮式里程计、悬崖/碰撞传感器
- **执行器**: 直流有刷电机 + 编码器、吸尘电机、边刷电机

### 结构层

- 学习 Fusion 360 / SolidWorks 做底盘结构设计
- 重点：驱动轮悬挂（保证贴地）、滚刷/吸口风道设计、传感器安装位置
- 可以先 3D 打印一个底盘原型验证

### 软件层（ROS2 节点图）

```
/lidar_node -> /scan
/imu_node   -> /imu/data
/odom_node  -> /odom
       ↓
/ekf_node (robot_localization) -> /odom_fused
       ↓
/slam_node (slam_toolbox) -> /map, /tf
       ↓
/nav2_planner (Navigation2) -> /cmd_vel
       ↓
/base_controller (MCU micro-ROS) -> 电机PID + 编码器反馈
```

---

## 三、起步建议

1. **不要一上来就买硬件**。先在 Gazebo 仿真里跑通完整的 SLAM + 导航流程。
2. **用 fishbot 或 turtlebot4 的仿真作为起点**，传感器配置和扫地机很接近（2D 激光 + IMU + 差分驱动）。
3. **当你能在仿真里控制虚拟扫地机建图、导航后**，再买一套低成本硬件（RPLIDAR + 树莓派 + 底盘套件）做实物验证。

---

## 四、环境搭建 (进行中)

### 当前进度

- [x] Mac Apple Silicon (ARM64) 开发环境确认

- [x] 安装 UTM 4.7.5 虚拟机软件

- [x] ~~恢复旧的 Ubuntu ARM VM~~ (内核太旧无法启动，已放弃)

- [ ] 下载 Ubuntu 24.04 ARM64 Desktop (UTM Gallery 下载中)

- [x] 启动 Ubuntu 24.04 VM，安装 ROS2 Jazzy ✅

- [x] 验证 ROS2 安装：turtlesim 小乌龟测试通过 ✅

- [x] 验证 Gazebo 仿真环境（ros-gz）✅

- [x] 切换至 Apple Virtualization 后端（GPU 加速）✅

- [x] fishbot 项目克隆并编译 ✅

- [x] **turtle_tf2 追踪 demo 完成** ✅ — 用 Hermes `delegate_task` workflow 并行构建 3 个 ROS2 包（broadcaster/listener/bringup），turtle2 成功追踪 turtle1。详见 `~/.hermes/skills/embedded/ros2-turtle-tf2/SKILL.md`

- [x] 解决 cmd_vel 通信 → turtle_tf2 listener 正确发布 `/turtle2/cmd_vel`（TF frame-aware steering）✅

- [x] Gazebo 插件迁移完成 ✅ — URDF 从 Gazebo Classic 插件迁到 gz-sim (Jazzy)：DiffDrive ✅, Odometry ✅, IMU/Lidar 传感器桥接已配置。fishbot.world 添加了传感器系统插件。
- [x] slam_toolbox 替代 cartographer ✅ — `fishbot_slam` 包（slam_toolbox online_async + RViz）
- [x] teleop 键盘控制 ✅ — `fishbot_teleop` 包（termios raw mode）
- [x] bringup launch + 学习文档 ✅ — `fishbot_bringup` + `SLAM_NAV2_GUIDE.md` (236行)
- [x] Gazebo 仿真全通 ✅ — URDF 插件迁移 + topic bridge + sensor 数据链路打通。slam_toolbox `/map` 成功发布。
- [ ] 保存地图 → Navigation2 路径规划并可视化（SaveMap 有 bug，可手动 `ros2 run nav2_map_server map_saver_cli`）

### 环境配置笔记

- **宿主机**: Mac Apple Silicon (M2 Pro) + UTM 4.7.5 (Apple Virtualization)
- **虚拟机**: Ubuntu 24.04 ARM64 Desktop (6 GB RAM, 4 核, 40 GB 磁盘)
- **目标**: ROS2 Jazzy + Gazebo + slam_toolbox + Navigation2
- **下载方式**: UTM Gallery → Ubuntu 24.04 (ARM64)

---

### 5. Ubuntu 系统操作手册

<callout emoji="💡">
**核心目标**：在 ROS2 开发中，你不需要成为 Linux 专家，但以下技能每天都会用到。掌握这些，你就能流畅地在 Ubuntu 中开发和调试机器人程序。
</callout>

#### 5.1 终端基础操作

终端（Terminal）是 Ubuntu 的核心交互界面。ROS2 的**编译、运行、调试**全部在终端完成。

| **命令** | **示例** | **说明** |
|-|-|-|
| `pwd` | `pwd` | 显示当前所在目录路径 |
| `ls` | `ls -la` | 列出目录内容（`-l` 详细信息，`-a` 含隐藏文件） |
| `cd` | `cd ~/ros2_ws` | 切换目录（`~` = home目录，`..` = 上级，`-` = 上一个目录） |
| `mkdir` | `mkdir -p ~/ros2_ws/src` | 创建目录（`-p` 递归创建父目录） |
| `cp / mv / rm` | `cp -r src dst` | 复制 / 移动 / 删除（`-r` 递归操作目录；`rm -rf` 危险！确认后再用） |
| `find / grep` | `grep -r "slam_toolbox" ~/ros2_ws/src/` | 搜索文件名 / 搜索文件内容（`-r` 递归） |
| `cat / less / tail` | `tail -f ~/.ros/log/*.log` | 查看文件内容（`tail -f` 实时追踪日志，调试 ROS2 时非常实用） |
| `echo` | `echo $ROS_DISTRO` | 输出文本 / 查看环境变量值 |

#### 5.2 软件包管理 (apt)

Ubuntu 使用 APT（Advanced Package Tool）管理系统软件。**安装 ROS2 及其依赖全靠 apt**。

```bash
# 更新软件包列表（安装任何包前先执行）
sudo apt update

# 升级所有已安装的包
sudo apt upgrade

# 搜索软件包
apt search ros-jazzy-slam

# 安装软件包（-y 自动确认）
sudo apt install ros-jazzy-slam-toolbox

# 卸载软件包
sudo apt remove ros-jazzy-slam-toolbox

# 清理不再需要的依赖
sudo apt autoremove

# 查看已安装的包
apt list --installed | grep ros
```

<callout emoji="❗">
**注意**：安装 ROS2 包时一定要带发行版前缀（如 `ros-jazzy-`），否则可能装到错误的 ROS 版本。
</callout>

#### 5.3 权限与 sudo

Linux 是多用户系统，文件操作受权限控制。`sudo` 让你临时以管理员身份执行命令。

| **概念** | **说明** |
|-|-|
| `sudo` | 以超级用户权限执行命令。安装软件、修改系统文件时必须用。首次使用会提示输入你的用户密码。 |
| `chmod` | 修改文件权限。例：`chmod +x script.sh` 给脚本添加可执行权限。ROS2 节点脚本和 launch 文件经常需要。 |
| `chown` | 修改文件所有者。例：`sudo chown $USER:$USER /dev/ttyUSB0` 让当前用户有串口权限。 |

<callout emoji="💡">
**警告**：不要在不理解命令含义的情况下盲目执行 `sudo`。尤其是 `sudo rm -rf /` 会毁掉整个系统。
</callout>

#### 5.4 环境变量与 .bashrc（ROS2 关键）

ROS2 依赖大量环境变量来定位包、工具和依赖。理解如何管理环境变量能避免 "command not found" 类问题。

| **操作** | **命令 / 说明** |
|-|-|
| 查看当前环境变量 | `env` 或 `echo $ROS_DISTRO` |
| 临时设置（仅当前终端） | `export MY_VAR="hello"` |
| 永久生效（写入 \~/.bashrc） | `echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc` |
| 使 .bashrc 立即生效 | `source ~/.bashrc` |

```bash
# 在 ~/.bashrc 末尾添加以下内容：

# ROS2 Jazzy 环境
source /opt/ros/jazzy/setup.bash

# 你的工作空间（每次打开终端自动加载）
source ~/ros2_ws/install/setup.bash

# 常用别名
alias cb="colcon build --symlink-install"
alias si="source ~/ros2_ws/install/setup.bash"
alias ros2_ws="cd ~/ros2_ws"

# Gazebo 相关
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/my_robot/models
```

<callout emoji="💡">
**ROS2 黄金法则**：每次新开终端，必须先 source setup.bash（或写入 .bashrc 自动执行）。如果提示 "ros2: command not found"，说明没 source。
</callout>

#### 5.5 系统监控

编译 ROS2 大工程时 CPU 吃满、Gazebo 仿真时内存飙高——你需要知道怎么**看一眼系统状态**。

| **命令** | **用途** | **推荐用法** |
|-|-|-|
| `htop` | 交互式进程管理器，看 CPU/内存占用 | `sudo apt install htop` 安装后直接敲 `htop` |
| `free -h` | 查看内存使用（-h 人类可读） | Gazebo 卡顿时先看这个 |
| `df -h` | 查看磁盘使用 | VM 磁盘容易满，定期检查 |
| `du -sh *` | 查看每个子目录大小 | 清理空间时用 `du -sh ~/ros2_ws/*` |
| `nvidia-smi` | NVIDIA GPU 状态（如有独显） | Jetson 或桌面 GPU 仿真时用 |
| `uptime` | 系统运行时间 + 负载 | 快速判断系统是否过载 |

#### 5.6 进程管理

ROS2 运行时会拉起很多节点，**Ctrl+C 有时不够干净**，进程可能残留。

```bash
# 查看所有进程
ps aux | grep ros

# 按名称终止进程
pkill -f "ros2"
killall gzserver    # 杀掉卡死的 Gazebo

# 交互式管理（推荐）
htop    # F3 搜索，F9 杀进程

# 后台运行命令
ros2 launch my_robot sim.launch.py &

# 查看后台任务
jobs

# 调回前台
fg %1

# 强制终止当前终端前台进程
# Ctrl+C (SIGINT) — 温和终止
# Ctrl+\ (SIGQUIT) — 更强力的终止
```

#### 5.7 SSH 远程连接

机器人开发中经常需要 SSH 到树莓派或 Jetson 上操作。Mac 和 Ubuntu 之间也可以 SSH。

```bash
# 安装 SSH 服务端（让这台机器能被远程连接）
sudo apt install openssh-server

# 从 Mac SSH 到 Ubuntu VM
ssh username@<ubuntu-vm-ip>

# 查看本机 IP
ip addr show    # 或简写 ip a

# 免密登录（在 Mac 上执行，然后按提示操作）
ssh-keygen -t ed25519
ssh-copy-id username@<ubuntu-vm-ip>

```

#### 5.8 Git 基础

ROS2 项目和代码几乎全在 GitHub 上。基础的 Git 操作每天都会用到。

| **命令** | **说明** |
|-|-|
| `git clone <url>` | 克隆仓库到本地（ROS2 源码几乎都这样拿到） |
| `git pull` | 拉取远程最新代码并合并 |
| `git status` | 查看当前修改状态（最常用） |
| `git diff` | 查看具体改了什么 |
| `git checkout -b <name>` | 创建并切换到新分支 |
| `git add <file> && git commit -m "msg"` | 暂存并提交修改 |
| `git log --oneline` | 查看提交历史（简洁模式） |

#### 5.9 终端效率快捷键

这些快捷键能让你的操作速度**快 3 倍**。

| **快捷键** | **效果** |
|-|-|
| `Ctrl+C` | 终止当前运行的命令（关闭 ROS2 节点） |
| `Ctrl+D` | 退出当前终端 / 发送 EOF |
| `Ctrl+L` | 清屏（等价于 `clear`） |
| `Ctrl+R` | 反向搜索历史命令（非常常用！输入关键词即可召回） |
| `Ctrl+A / Ctrl+E` | 跳到行首 / 行尾 |
| `Ctrl+W / Alt+Backspace` | 删除前一个单词 |
| `Ctrl+Shift+C / V` | 在终端中复制 / 粘贴（不是 Ctrl+C！那是终止命令） |
| `Tab` | 自动补全命令、文件名、ROS2 子命令（连续按两下显示所有选项） |
| `↑ / ↓` | 浏览上一条 / 下一条命令 |
| `!!` | 重复上一条命令（例：`sudo !!` = 用 sudo 重新执行上条命令） |

#### 5.10 文本编辑器速成

在终端里编辑配置文件、launch 文件、CMakeLists.txt，你需要会一个终端编辑器。

<grid>
<column width-ratio="0.500000">
**nano（新手友好）**
- `nano file.txt` 直接编辑
- 底部有快捷键提示
- `Ctrl+O` 保存，`Ctrl+X` 退出
- 适合快速改配置
</column>
<column width-ratio="0.500000">
**vim（高手标配）**
- `vim file.txt` 进入编辑器
- 按 `i` 进入编辑模式
- `Esc` 退出编辑，`:wq` 保存退出
- `:q!` 不保存强制退出
</column>
</grid>

<callout emoji="💡">
**建议**：先用 nano 快速上手。nano 会在终端底部显示所有操作提示，不需要记忆命令。当你觉得 nano 效率不够时，再学 vim。
</callout>

#### 5.11 磁盘与网络排障

| **命令** | **用途** |
|-|-|
| `df -h` | 查看磁盘使用率。VM 磁盘容易满——Gazebo 模型和日志很占空间 |
| `du -sh ~/.ros/log/` | ROS2 日志占了多少空间（日志会持续增长！） |
| `nc -zv localhost 11311` | 测试端口是否可达（ROS2 节点通信排障） |
| `ping 8.8.8.8` | 测试网络连通性 |
| `ip addr` | 查看本机 IP 地址 |
| `ss -tlnp` | 查看本机正在监听的 TCP 端口（找哪个节点占用了端口） |

---

#### 5.12 速查卡片

这些是最常用的 **10 条命令**，贴在显示器旁边，一周内就能形成肌肉记忆。

1. `ls -la` — 看目录里有什么
2. `cd ~/ros2_ws` — 进工作空间
3. `source install/setup.bash` — 加载 ROS2 环境
4. `colcon build --symlink-install` — 编译工作空间
5. `ros2 launch <pkg> <file>` — 启动节点
6. `ros2 topic list / echo <topic>` — 查看话题
7. `htop` — 看系统资源
8. `Ctrl+R` — 搜索历史命令
9. `sudo apt update && sudo apt upgrade` — 更新系统
10. `code .` — 用 VS Code 打开当前目录（推荐先装 VS Code）
