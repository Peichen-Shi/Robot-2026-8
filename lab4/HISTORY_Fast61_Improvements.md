# 历史记录：Sub4 调速前的可靠性改进

以下描述对应旧版阶段，当前结果见 README_zh.md。

# Fast61 稳定性改进（2026-10-29）

此版本以用户提供的 VNAV_Lab4_3D_Fast61.zip 为基线。

## 修改结果

1. 元数据请求使用一个基于单调时钟的整体超时预算，覆盖连接等待、帧头和正文接收。过去只限制每次 recv 的等待，持续碎片回复可拖长整次请求；现在碎片不会刷新预算。拒绝非正、无穷和 NaN 超时配置。
2. 竞速验收要求末尾至少连续一秒稳定遥测，相邻样本间隔不超过 100 ms；重复时间戳不能累积时长，窗口边界的重复异常样本也会被检查。拒绝时钟回退、非有限值和错误的运行标识。
3. 增加 scripts/recheck_race_evidence.py，可在没有 ROS 的 Windows/Linux 上复核原始录包。仅读取已有证据，输出使用独占创建方式，避免覆盖旧结果。

赛道、飞行控制增益、6.1 秒分段时间和推力上限保持基线值。本次改进针对通信等待和验收可靠性，没有未经测量地提速。

## 已完成验证

- 原有 28 项测试通过；改进后的完整测试集 42 项通过。
- 四份原始 bag 均通过新的竞速验收规则。
- 详细自动测试日志：evidence/validation/improvements_20261029_tests.log。
- 最终历史录包复核：evidence/recheck_20261029_final.json。
- 与用户 ZIP 的文本差异：IMPROVEMENTS_20261029.patch。
- PACKAGE_SHA256SUMS.txt 已为改进版重新生成。

**验证边界：没有进行本次修改后的新飞行、ROS 编译或速度对比。** 电脑运行着 VMware Ubuntu，但本次未能建立其命令连接；WSL Ubuntu 没有 ROS 环境。原来的运行目录未被替换，GitHub 本次没有新提交。

## 在已有 ROS Noetic 仿真环境中使用

建议解压至新的 ~/lab4_fast61_improved_20261029 目录，保留原版供对照：

```bash
cd ~/lab4_fast61_improved_20261029
bash install.sh
python3 -m unittest discover -s tests -v
bash verify_sub7.sh
```

构建默认读取 ~/vnav_ws4/devel；若位置不同，先设置 LAB4_UNDERLAY。三次独立重启都通过后，再使用 bash run_3d_race.sh 日常启动。

## 不启动仿真的离线复核

需 Python 和 rosbags：

```bash
python scripts/recheck_race_evidence.py evidence/qualify61_1_01 evidence/qualify61_2_01 evidence/qualify61_3_01 evidence/sub7_20261025_062950_612446933_01 --output audit_new.json
```

这项命令只重新检查历史录包，不能证明新版本的闭环飞行性能。
