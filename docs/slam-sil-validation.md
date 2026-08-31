# SIL 中的 SLAM 建图验证

**日期**: 2026-08-31
**目的**: 在无硬件条件下验证 SLAM 链路，切分 M5「地图扭曲」问题的责任域

## 为什么要做

M5 阶段的痛点是**移动建图时地图扭曲 + 扫描匹配伪影**。但「地图扭曲」可能来自三处：

1. slam_toolbox 参数/TF 链配置错
2. 里程计数据源有问题（IMU 零偏、轮式标定）
3. 时间戳同步问题

真车上三者叠加，很难分离。SIL 用**理想里程计 + 几何精确的虚拟雷达**跑一遍，
如果地图是好的，就能把嫌疑锁定到数据源，不用再折腾参数。

## 怎么搭的

真实雷达需要 `/dev/ttyUSB0` 上的 LD06，SIL 里没有。所以写了
`tools/fake_lidar_sil.py`：

- 假设机器人在 **6m × 4m 矩形房间**里，中央有 **0.4m 见方的柱子**（提供回环特征）
- 从 TF `odom -> base_footprint` 拿当前位姿，**射线-线段求交**算出每束激光距离
- 发布 `/scan`，规格对齐 LD06：360 束 / 1° 分辨率 / 10 Hz / 0.02~12m / 8mm 噪声
- 零第三方依赖（不用 numpy，自带轻量 LCG 噪声）

一键启动：`tools/run_slam_sil.sh`（模拟器 → 底盘+EKF → 假雷达 → slam_toolbox）

## 结果

驱动小车走了一圈矩形轨迹（前进 1.5m → 左转 → 1m → 左转 → 1.5m → 左转 → 1m → 左转）：

| 指标 | 实测 | 判读 |
|---|---|---|
| 地图尺寸 | 123×82 格 @5cm = **6.15 × 4.10 m** | 真值 6.00×4.00，**误差 2.5%** |
| `map->odom` 修正量 | **5.6 cm / 0.21°** | 里程计漂移极小 |
| 房间轮廓 | 规整矩形，四角近似直角 | **无平行四边形/梯形畸变** |
| 墙线 | 笔直 | **无扭曲弯曲**（仅底墙断续、边缘毛刺） |
| 中央柱子 | 清晰可辨 | ✓ |
| 重影 | 柱子右侧 1~2 格灰影 | 轻微，可接受 |
| 占据/空闲/未知 | 8.5% / 91.5% / 0% | 房间完全探明 |

地图存于 `maps/sil_room.pgm` + `.yaml`。

## 结论 —— 这是本次最重要的产出

> **SIL 里 slam_toolbox 的参数和 TF 链是健康的，地图不扭曲。**

所以真车上的地图扭曲，**问题不在 slam_toolbox 配置，而在里程计数据源**。
排查优先级因此确定：

1. **陀螺仪零偏** — 工具已备好 `tools/gyro_bias_check.py`
2. **EKF 融合配置 / 协方差**
3. **轮式里程计标定**（`tools/wheel_calibration.py` 已有）
4. slam_toolbox 参数 — **最后才动，现在证明它没问题**

## 顺手修的两个真实 Bug

### 1. `ldlidar_stl_ros2` submodule 未初始化

fresh clone 后是空目录 → `ros2 launch mcr_bringup robot.launch.py` 报
`package 'ldlidar_stl_ros2' not found` → 雷达节点起不来。

### 2. 上游驱动在 gcc 13+ 编译失败

`ldlidar_driver/include/logger/log_module.h` 第 40 行
`//#include <pthread.h>` 被上游注释掉了。老编译器靠间接包含能过，
ROS2 Jazzy / gcc 13+ 下报：

```
error: 'pthread_mutex_unlock' was not declared in this scope
```

整个包编不过，连带 `mcr_bringup` / `mcr_perception` 被 abort。

**两个问题一起解决**：`tools/setup_ldlidar.sh`
（初始化 submodule + 自动应用 `patches/ldlidar-pthread-include.patch`，幂等）。

修复后 **5 个包全部编译通过**。

## 已知的 SIL 局限

`tools/stm32_uart_sim.py` 的 **IMU 部分是全零**（连重力都是 0，`accel_z = 0`
而非 9.81）。所以：

- SIL 里 EKF 的姿态估计没有实际意义
- `gyro_bias_check.py` 在 SIL 里必然报「零偏良好」——这是模拟器的性质，不是真实结论
- **IMU 相关的验证必须上真车做**

如果以后要在 SIL 里验证 IMU 融合，得先给模拟器加上带零偏和噪声的 IMU 模型。

## 新增工具

| 文件 | 用途 |
|---|---|
| `tools/fake_lidar_sil.py` | 虚拟 LD06，射线求交生成 `/scan` |
| `tools/run_slam_sil.sh` | SLAM SIL 全栈一键启动 |
| `tools/gyro_bias_check.py` | 陀螺仪零偏测量 + 补偿常量输出（真车用） |
| `tools/setup_ldlidar.sh` | submodule 初始化 + pthread 补丁 |
| `tools/slam_probe.sh` | SLAM 链路诊断（话题频率 / TF / lifecycle / 参数） |
| `tools/drive_map.sh` | 驱动矩形轨迹建图 + 保存地图 |

## 踩坑记录

**`/scan` 的 frame_id 必须和 URDF 一致。** 一开始假雷达发 `laser`，
但 `mcr_description` 里雷达 link 叫 **`laser_link`**，slam_toolbox 的
`base_frame` 是 **`base_footprint`**（不是 `base_link`）。frame 名不匹配的表现是：

```
Message Filter dropping message: frame 'laser' at time ... for reason
'discarding message because the queue is full'
```

slam_toolbox 会显示 `active` 且不报错，但**既不出 `/map` 也不发 `map->odom`**，
很容易误判成参数问题。改对 frame 后立刻出现
`Registering sensor: [Custom Described Lidar]`，地图开始生成。
