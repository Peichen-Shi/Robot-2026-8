# Lab6 · 实验结果与证据

## 当前实测

| 资料 | 入口 | 怎么看 |
| --- | --- | --- |
| 扩展片段的统计摘要 | [summary.csv](office_extended/summary.csv) | 四种必做方法都成功求解 2,214 对，五点对照成功 2,211 对 |
| 扩展片段的逐帧结果 | [rpe.csv](office_extended/rpe.csv) | 每行是一对画面在一种方法上的结果 |
| 05–08 号误差图 | [曲线索引](charts/README.md) | 二维方向和三维米制平移分别作图 |
| 12 张 Ubuntu 截图 | [截图索引](screenshots/README.md) | 01–04 为短片段 RViz 回放，09–16 为构建与扩展实测证据 |
| 数据转换和话题 | [转换日志](logs/office_extended_convert.log) · [ROS bag 信息](logs/office_extended_bag_info.txt) | 核对 2,215 张 RGB、2,215 张深度图和 27,091 条里程计 |
| 输入文件校验 | [扩展 bag 的 SHA256](office_extended/dataset_sha256.txt) | 检查复跑用的输入文件 |
| 早期实验 | [213 对短片段](office_preview/rpe.csv) · [Lab5 二维回归检查](lab5_crosscheck_rpe.csv) | 保留早期试跑。Lab5 回归数据没有深度 |

## CSV 列怎么读

| 列 | 含义 |
| --- | --- |
| `pair` / `time_s` | 帧对编号，从 0 开始。时间以序列首帧为起点 |
| `matches` / `inliers` | 输入匹配数和模型内点数。Arun 的输入还要求有效深度 |
| `status` | `ok` 表示已求解，`failed` 表示未取得模型 |
| `rotation_frobenius` / `rotation_deg` | 旋转矩阵距离和角度误差 |
| `translation_rpe` | 二维方法是单位方向距离，范围 0 到 2。Arun 的单位为米 |
| `gt_baseline_m` | 两帧之间的参考移动距离，单位为米 |
| `solve_ms` | 位姿求解耗时，包含 RANSAC，排除读取和 SIFT |

缺失误差写作 `nan`。中位数与 P90 按有效误差计算，成功率使用全部评估帧对作分母。完整归档获取状态见 [Lab6 首页](../README.md)。
