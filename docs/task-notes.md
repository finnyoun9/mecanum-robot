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

- [ ] T1 完成并 push，更新 agent-handoff
- [ ] T2 完成并 push，更新 agent-handoff——写明已加的 grep 守卫
      `tools/check_calibration_constants.sh` 挂进了 firmware-tests CI（不是「靠人肉」）
- [ ] T3 完成并 push，更新 agent-handoff + hardware-closed-loop-roadmap
- [ ] 确认两个 CI workflow 都绿：`gh run list` 看 `Firmware Tests` 与 `ROS2 Build & Test`
- [ ] 在 `software-tasks.md` 勾掉三项，写清各自验证方式
