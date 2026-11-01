# Lab4：3.350 秒竞速候选版本

基于仓库原 3.896 秒版本，已录得一次完整竞速验收通过的 **3.349725 秒**成绩。相较历史三次合格基线中的最快成绩 3.894973 秒，快 **0.545248 秒，约 14.0%**。

**当前状态：单次竞速目标已达到，三次长时间稳定性验证尚未通过。** 当前虚拟机的可见 Unity 窗口会周期性停顿约 0.5 秒，触发原有电机看门狗。后续两次虽然也通过全部门框，但终点出现看门狗事件，不能计作整次验收通过；另一次在比赛中受到影响而失败。完整结果见 [SUB34_RESULTS.md](SUB34_RESULTS.md)。

## 当前 Ubuntu 虚拟机直接运行

代码与已编译工作空间位于独立目录，原工程保留。

```bash
bash ~/lab4_speed_20261031/run_sub34_race.sh
```

脚本会重新打开原 Unity 三维场景，准备起飞，执行比赛并显示实际过门时间，之后继续检查终点悬停。默认保留 65 秒终点观察。计时数字只表示完成计时，**还需等待录包分析输出 ACCEPTED**。当前环境可能因上述停顿而显示验收失败，不能保证每次重现 3.350 秒。

停止本工程的飞行：

```bash
bash ~/lab4_speed_20261031/stop_sub4.sh
```

重新执行三次独立启动、每次至少 60 秒连续稳定悬停的验证：

```bash
bash ~/lab4_speed_20261031/verify_sub34.sh
```

原 3.9 秒参数仍可运行：

```bash
bash ~/lab4_speed_20261031/run_sub4_race.sh
```

## 压缩包重新安装

解压后进入 `lab4`，在已有原 Lab4 依赖的 Ubuntu 20.04 / ROS Noetic 环境执行：

```bash
LAB4_UNDERLAY="$HOME/vnav_ws4/devel" bash install.sh
bash run_sub34_race.sh
```

`LAB4_UNDERLAY` 指向已构建成功的依赖工作空间，当前虚拟机使用上面的路径。包内包含源代码、参数、分析脚本及原始录包，不包含 ROS、Unity 或编译缓存。Unity 从 `~/vnav-builds/lab4/lab4.x86_64` 启动。环境配置针对当前虚拟机，迁移后须核对显示环境和依赖路径并重新复测。

## 改进内容

1. **分段时间与门内目标点联合调整。** 新轨迹门间计划时间 3.41 秒，另有 0.95 秒进场和 1.8 秒减速，总参考时长 6.16 秒。比赛时间另按实际第 1 门至第 12 门的正向穿越计算。
2. **自由偏航下使用连续的期望姿态。** 新的 `ThrustFrameReference` 从上一帧期望姿态沿新推力方向更新，避免期望角速度继承飞机自身的偏航旋转、导致前馈抵消角速度阻尼。开关默认关闭，只在新参数中启用；终点悬停恢复原策略。
3. **减少指令传输与线程调度延迟。** 旋翼订阅开启 `tcp_nodelay`，限制小矩阵计算及图形库线程数量。状态超时、参考超时和 0.35 秒电机看门狗均保持原值。
4. **修复稳定性检查的浮点边界问题。** 选择完整观察窗口时，避免选到比截止时间晚约 1 纳秒的样本。位置、速度、角速度和连续性阈值不变，新增两个回归测试。

原始 12 门的位置、尺寸、顺序、Unity 二进制、仿真质量与重力、仿真时钟和控制器指令上限均与原 3.9 秒版本一致。门内目标偏移仍限制在 ±0.70 米。试验过的软件渲染和其他显示设置没有进入交付运行配置。

## 配置与证据

- `optimization/planner_profiles/sub34.yaml`：分段时间与门内目标偏移。
- `optimization/profiles/sub34.yaml`：控制参数。
- `evidence/sub34_qualified/`：有效单次成绩、失败的长稳复测、原始 bag、过门 CSV、数值复核及日志。目录名只是资料目录，通过与否以 JSON 中的 `passed` / `qualified` 为准。
- `evidence/sub34_qualified/qualification_summary.json`：当前总结果为 `qualified: false`。
- `evidence/sub34_qualified/history/attempts.json`：含失败记录的实验索引；不足 12 门的局部时间不作为完整成绩。
- `evidence/sub34_qualified/history/tests_final_lean_*`：47 项 Python 测试、4 项 C++ 测试通过。
- `PACKAGE_SHA256SUMS.txt`：包内文件的完整性清单。

原 3.9 秒版本的说明和合格记录保留在 `HISTORY_Sub4_README.md`、`SUB4_RESULTS.md`、`evidence/sub4_qualified/`。这些历史记录不能证明新候选版本已通过长时间复验。
