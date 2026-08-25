# 软件任务说明

> 建立于 2026-08-24。开工前先 `git pull` 并读
> [project-status.md](project-status.md) 的「证据分级」和「硬件是独占资源」两节。
> 这三项**全部不需要真机**，不碰 `firmware/Core/HW/`、ST-Link、烧录流程。
>
> 提交约定：commit message 只保留项目作者，不添加额外署名 trailer。
> 每完成一项：在 [software-tasks.md](software-tasks.md) 勾掉对应条目、写清验证方式、push，
> 并在 [project-status.md](project-status.md) 的「当前真实状态」里同步被你改变的事实。

---

## ⚠️ 贯穿三项的硬约束：标定常数现在是 448，不是 224

`EDGES_PER_WHEEL_REV` 实测为 224 / 圈（1 倍频，手转 10 圈 ×2），编码器改为 A 相
双边沿解码（2 倍频）后**翻倍成 448**。任何新建测试、注释、脚本里出现这个常数，
必须用 **448**，别照抄旧资料里的 224。这个常数硬编码在三处，彼此独立：

| 位置 | 语言 | 现状 |
| --- | --- | --- |
| `firmware/Core/Inc/encoder.h` 第 51 行 | C | `#define EDGES_PER_WHEEL_REV 448` |
| `ros2_ws/src/mcr_bringup/src/mcr_hardware_interface.cpp` 第 29 行 | C++ | `static constexpr double EDGES_PER_WHEEL_REV = 448.0;` |
| `tools/stm32_uart_sim.py` 第 47 行 | Python | `EDGES_PER_WHEEL_REV = 448` |

---

## T1：让协议模拟器反映实测的被控对象

**目标**：`tools/stm32_uart_sim.py` 现在直接把「命令轮速」当「实际轮速」积分成编码
计数——瞬间跟随、无死区、无上限。这让 ROS2 侧的闭环测试跑在一个理想对象上，测不出
真实系统的问题。改成分层、可测的轮速—编码对象模型。

**实测依据**（见 [hardware-closed-loop-roadmap.md](hardware-closed-loop-roadmap.md) 的
「占空比 → 转速实测曲线」，DP100 台面供电、底盘抬起所测）：
- 20% 占空比以上近似线性，斜率约 `0.986 edges/(duty·s)`
- **启动死区**：5% 占空比四轮完全不转，10% 正常
- 满占空比外推约 **4.27 wheel rev/s ≈ 0.81 m/s** 车速上限（SIL 里
  `firmware/Core/SIL/mocks/mock_hal.c` 的 `max_wheel_rps = 4.27f` 就是按这个更新的，
  参考它那条注释的写法）

**要做**：给 `tools/stm32_uart_sim.py` 的 `Stm32Sim` 加一个更接近真实的轮速→计数
模型，至少覆盖：
1. **速度上限**：命令轮速超上限时，积分速率不得线性外推出物理上不可能的速度；
   上限换算 `4.27 rev/s × 2π ≈ 26.8 rad/s`（对应 0.81 m/s）。
2. **启动死区**：命令轮速落在死区内时不产生编码计数（死区换算自实测的 5%~10% 占空比
   驱动的转速区间）。
3. **一阶惯性时间常数**：目前**没有实测数据**。如果要加，必须在代码注释里明确标注是
   **估计值而非实测**，别写成实测（参考 `mock_hal.c` 里 `max_wheel_rps` 的注释措辞）。
   不加也可以，别硬凑。

> ⚠️ **上限、死区都绑定当前供电电压**（DP100 台面供电，非电池）。转速与供电电压近似
> 成正比，换成 3S 电池（11.1–12.6 V，且随电量下降）后整个占空比曲线会整体上移，这几个
> 模型参数届时需复测。在代码注释里按这个口径标注，别把「台面供电下测出的上限」当成
> 电池供电下的物理上限。

**验收**：
- `tools/verify_sil.sh` 仍通过（该脚本在 `mcr-ros2` 容器里 `--network host` 跑）。
- 下发超过 0.81 m/s 的 `/cmd_vel` 时，模拟器输出的里程计/轮速不再线性外推出物理上
  不可能的速度（即编码计数增量受上限约束）。
- 顺手核对：模拟器第 158 行转发运动学用 `r = 0.0325`，与实测半径 **0.030 m** 不符；
  若顺手改掉则一并同步 `project-status.md`，不改就在这行加注释标注「未实测/存疑」。
  这条属于 T1 顺带发现的问题，改之前先在中文里说明你的判断。

---

## T2：给标定常数加一道防不一致的防线

**背景**：`EDGES_PER_WHEEL_REV` 曾按厂商典型值推成 1496，实际实测 224（现在 448），
**错了 6.7 倍**。它硬编码在 C / C++ / Python 三处（见上表），当时靠人工逐个改。
上一处改 224→448 也是手工三处一起改的——正好说明需要一道自动防线。

**要做**：让「某处常数被改、其他没跟上」能被 CI 自动发现，而不是等下次有人想起。

**推荐方向（主）——一个跨语言的 grep 一致性检查脚本，挂进 firmware-tests CI**：

三处是 C / C++ / Python 三种语言的字面量，靠三个各自独立的测试去守要搭三套测试通道
（Python 现在还没有），太费。改用语言无关、能在任何容器里跑的检查脚本：

- 新建 `tools/check_calibration_constants.sh`，对三个位置各自 grep 一个固定的拼写
  模式，任一位置缺失或值不对就退出非零。参照每个文件的**实际声明**写模式，避免误匹配：
  - `firmware/Core/Inc/encoder.h` → 匹配 `EDGES_PER_WHEEL_REV\s+448`
  - `ros2_ws/src/mcr_bringup/src/mcr_hardware_interface.cpp` → 匹配
    `EDGES_PER_WHEEL_REV\s*=\s*448`
  - `tools/stm32_uart_sim.py` → 匹配 `EDGES_PER_WHEEL_REV\s*=\s*448`
  - 脚本开头用 `set -e` 并先在 `SOURCE_ROOT=...` 定位仓库根（以仓库根为基准拼 path，
    别依赖调用时的 cwd）。
- 在 `.github/workflows/firmware-tests.yml` 里把这一步（比如 `Run calibration
  constant guard`）加到 gcc/CMake 那组步骤前面，容器里 grep 现成。这样任意一处被改、
  其余没跟上，推上去 CI 就红。

**推荐方向（次，可选增强）**：再给 `mcr_bringup` 加 gtest 断言「N 个编码计数 → 预期
弧度/米」用 448 写死期望值（参考已有 `ros2_ws/src/mcr_bringup/test/test_mecanum_kinematics.cpp`
与 CMakeLists.txt 第 107 行 `ament_add_gtest` 的写法）。这不是必须——grep 脚本已经
能守住一致性。若加了，C 侧 `firmware/Core/Test/test_encoder.c` 已提供同样的 host 断言，
不必重复造。

**不要**为了「单一来源」把三处收敛到一个跨语言共享文件——跨 C/C++/Python 不现实，
属于过度设计。**grep 脚本 + 现有 host 测试已经达到「防不一致」的目的。**

**验收**：故意把三处中任意一处常数改错（比如把 `stm32_uart_sim.py` 改成 224），
`tools/check_calibration_constants.sh` 退出非零、firmware-tests CI 失败；改回 448 后 CI
恢复绿。验证做完务必**改回 448**。模拟器里若出现 224 而 grep 通过，说明 grep 模式没写对，
要修模式而不是放行。

> 若你不要 grep 方案（比如嫌它靠文本匹配太脆），退而求其次才是分语言测试；那种情况下
> 若 Python 侧确实挂不进现有 CI，就在 `project-status.md` 注明「该处靠人肉核对」，别
> 假装有守卫。但 grep 方向是这个任务的最优解，优先做它。

---

## T3：标出 lx / ly 未实测

`mcr_hardware_interface.cpp` 第 18-19 行：

```cpp
static constexpr double LX_DEFAULT = 0.10;  /* Half wheelbase front-rear */
static constexpr double LY_DEFAULT = 0.12;  /* Half track-width left-right */
```

这两个值（半轴距 / 半轮距）从项目建立起就是**估计值，从未实测**。麦克纳姆运动学里横移
和旋转分量直接依赖它们，错了会在横移/转向时里程计系统性偏差，而前进方向看起来却准，
很隐蔽。模拟器 `tools/stm32_uart_sim.py` 第 158 行 `l = 0.10 + 0.12 = 0.22` 也用了
同一组估计值。

**要做**（不需要你测，拿尺量真车属硬件侧）：
1. 在 `mcr_hardware_interface.cpp` 第 18-19 行注释里明确标注这两个值**未实测**，
   并写明测量方法：轴距 = 前后轮轴中心距，轮距 = 左右轮中心距，各取一半。
2. 在 [hardware-closed-loop-roadmap.md](hardware-closed-loop-roadmap.md) 的 **M4 章节**：
   - 标注 lx / ly 未实测；
   - 在 M4 验收条件里加「用实测 lx/ly 替换默认值」。
3. 若顺手在模拟器里标注 `l = 0.10 + 0.12` 也是估计值，一并注明；不改则保持现状。

**价值**：防止这两个数被默认成「已验证」——和上次「台面限流电源 6V 被当成电机额定
12V」是同一类错误，都是把估计值当成了事实。

---

## 三项完成后

- [ ] T1 完成并 push，更新 project-status
- [ ] T2 完成并 push，更新 project-status——写明已加的 grep 守卫
      `tools/check_calibration_constants.sh` 挂进了 firmware-tests CI（不是「靠人肉」）
- [ ] T3 完成并 push，更新 project-status + hardware-closed-loop-roadmap
- [ ] 确认两个 CI workflow 都绿：`gh run list` 看 `Firmware Tests` 与 `ROS2 Build & Test`
- [ ] 在 `software-tasks.md` 勾掉三项，写清各自验证方式

---
---

# 第二轮任务（2026-08-25 交接）

> T1/T2/T3 已完成。以下是新一轮，**同样全部不需要真机**。
>
> 开工前先 `git pull`，读 [project-status.md](project-status.md)，特别是新增的**「当前卡点」**一节——
> 真机侧目前卡在遥控器供电上，那部分**不归你做**，别去碰 ST-Link 和烧录。

## 边界（与上一轮相同）

**不要碰**：`firmware/Core/HW/` 的烧录流程、ST-Link、真机测试。硬件由真机调试端独占。

**新增说明**：`firmware/Core/HW/remote_drive_main.c` 是新加的真机目标，**代码可以读、可以提改进意见，但不要擅自改动**——它正在真机验证过程中，改了会让另一端基于错误前提排查。

---

## T4：把标定守卫扩展到轮径（优先）

**背景**：T2 建的 `tools/check_calibration_constants.sh` 只守 `EDGES_PER_WHEEL_REV`。**轮径没守，然后就出事了**——

2026-08-25 发现 `firmware/Core/Src/remote_control.c` 的 `wheel_radius` 仍是 **0.0325**，而实测值 **0.030** 早已同步到 `encoder.h`、ROS 2 硬件接口、URDF 和模拟器。这个文件被漏了，**每一条下发的轮速都因此差 8%**，而且是靠人偶然读到才发现的——正是 T2 想防止的那类问题，只不过守卫没覆盖这个常数。

**当前各处的值**（已核对）：

| 位置 | 值 | 状态 |
| --- | --- | --- |
| `firmware/Core/Inc/encoder.h` | `WHEEL_DIAMETER_M 0.060f`（直径） | ✅ |
| `firmware/Core/Src/remote_control.c` | `wheel_radius = 0.030f` | ✅ 刚修 |
| `ros2_ws/src/mcr_bringup/src/mcr_hardware_interface.cpp` | `0.030` | ✅ |
| `ros2_ws/src/mcr_description/urdf/mcr.urdf.xacro` | `0.030` | ✅ |
| `tools/stm32_uart_sim.py` | `WHEEL_RADIUS_M = 0.030` | ✅ |
| `ros2_ws/src/mcr_bringup/test/test_mecanum_kinematics.cpp` | **`0.0325`** | ⚠️ 见 T5 |

**要做的**：扩展 `check_calibration_constants.sh`，让上面前五处的轮径不一致时 CI 失败。

注意 `encoder.h` 存的是**直径 0.060**，其余是**半径 0.030**——守卫要么分别匹配各自的字面量，要么在脚本里做换算。**别为了统一而去改 `encoder.h` 的语义**，那个常数的名字和用途是对的。

**验收**：故意把五处中任意一处改错，`Firmware Tests` 失败；改回后恢复绿。验证完记得**改回正确值**。

---

## T5：判断测试夹具里的 0.0325 是不是遗留错误

`ros2_ws/src/mcr_bringup/test/test_mecanum_kinematics.cpp:25` 用 `p.wheel_radius = 0.0325;`。

**需要你判断**：纯运动学单元测试用什么半径其实无所谓（只要期望值自洽），但这个数恰好是**那个已被推翻的旧轮径**。两种可能：

- 它只是个任意夹具值 → 那就**加一行注释说明"此处数值任意，与实测轮径无关"**，免得下次又有人以为它是遗留 bug（比如这次）
- 它本意是代表真车 → 改成 0.030 并同步期望值

选哪条你判断，但**必须留下痕迹**，别让它继续保持这种"看不出是有意还是遗漏"的状态。

**验收**：`colcon test` 通过；该数值的意图在代码里写明白了。

---

## T6：在有 Docker 的环境复核 SIL 端到端

T1 改完模拟器后，**`tools/verify_sil.sh` 一直没在真正的 ROS 容器里跑过**（当时开发机没有 Docker）。`project-status.md` 里如实记着这一条待办。

**要做的**：在有 Docker 的机器上按 `docker/README.md` 起容器，跑通 `colcon build` → 启动 `tools/stm32_uart_sim.py` → `ros2 launch mcr_bringup robot.launch.py serial_device:=<pty>` → `bash tools/verify_sil.sh`。

**验收**：`/odom` 能采到数据；把结果（成功或失败的完整输出）写进 `project-status.md`，把那条待办**改成已验证或记下失败原因**。跑不了就明说环境不具备，别留着含糊。

---

## T7：为 M3 准备四轮速度控制模块（可选，工作量较大）

**背景**：M2 只做了单轮速度环，跑在真机台架 `pid_step_main.c` 里。M3 要四轮闭环，需要一个能被 SIL 和真机共用的模块。

**要做的**：在 `firmware/Core/Src/` 加一个四轮速度控制器（四个 `pid_ctrl_t` 实例 + 目标速度分配 + 输出限幅），配主机单元测试。**纯逻辑，不碰任何外设**。

**必须遵守的实测前提**（来自 `hardware-closed-loop-roadmap.md`）：

- 起始增益 `Kp=100 / Ki=300 / Kd=0`（M2 实测，空载、台面与电池供电下均验证）
- **抗积分饱和**：`pid.c` 钳位的是积分本身而非积分项，`integral_max` 必须按 `out_max / Ki` 推导，不能写死。这个坑 M2 已经踩过一次
- **四轮一致性 4.9%、RR 最慢**——接口要允许**每轮独立增益**，不要写死成共用一组

**验收**：主机单测覆盖"四轮同时跟随目标速度""饱和时积分不失控""某轮增益单独调整生效"；CTest 通过。

---

## 完成后

- [ ] T4 完成并 push，`project-status.md` 里写明守卫现在覆盖哪些常数
- [ ] T5 完成并 push
- [ ] T6 完成并 push，更新 `project-status.md` 里那条 SIL 待办
- [ ] T7（若做）完成并 push
- [ ] 确认两个 CI workflow 都绿：`gh run list`
