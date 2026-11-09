# Lab6 实验证据

| 资料 | 入口 |
| --- | --- |
| 213 对的逐帧统计 | [RPE CSV](office_preview/rpe.csv) |
| 有效对数与误差中位数 | [摘要 CSV](office_preview/summary.csv) |
| 12 张实际截图（01–04、09–16） | [编号索引](screenshots/README.md) |
| 05–08 旋转与平移图表 | [图表索引](charts/README.md) |
| 编译与数据转换 | [运行日志](logs/) |
| 片段数据校验 | [SHA256](dataset_sha256.txt) |
| Lab5 额外二维回归检查 | [逐帧 CSV](lab5_crosscheck_rpe.csv)，该数据没有深度，Arun 无结果 |

二维平移列为单位方向距离，Arun 平移列为米。失败写为 failed，方向不可观测的样本写 nan。求解时间排除 SIFT 提取和数据读取。
