# Lab4 无人机十二门竞速

**本次实测 2.677145 秒，通过 12 / 12 道门，竞速验收通过。** 时间按 ROS 仿真时钟计算，从实际穿过第 1 门开始，到实际穿过第 12 门结束。

## 快速现场演示

在已经编译好的 Ubuntu 虚拟机中，打开桌面终端并执行：

```bash
cd ~/lab4_2p67_run/lab4
bash run_sub27_race.sh
```

Unity 窗口打开后，无人机会自动飞行。终端出现 **FINISHED** 和 **gates 12/12** 时，旁边的毫秒数就是本轮成绩。演示结束后按 **Ctrl+C**。每轮成绩以当次终端输出为准。

首次安装请先[下载 2.67 秒竞速版 ZIP](packages/Lab4_2p67_20261105.zip)，解压后阅读包内的 `README_FIRST.txt`。竞速版的 `run_sub27_race.sh` 位于压缩包解压后的 `lab4/` 目录。

## 目录怎么找

| 目录 | 内容 |
| --- | --- |
| [packages/](packages/) | 2.67 秒竞速版原始压缩包，现场演示从这里下载 |
| [code/](code/) | 控制器、轨迹规划和仿真桥接源码 |
| [config/](config/) · [optimization/](optimization/) | 配置和竞速调参资料 |
| [scripts/](scripts/) · [tests/](tests/) | 编译、运行、分析与测试工具 |
| [deliverables/](deliverables/) | 实验交付材料 |
| [evidence/](evidence/) · [videos/](videos/) | 历次实验记录和演示视频 |

[完整实验说明](README_zh.md) · [返回小组仓库首页](../README.md)
