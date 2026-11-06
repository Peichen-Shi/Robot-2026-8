# 原始数据图表 Charts from Measurements

这些图由 `scripts/make_report_charts.py` 读取 `results` 生成，PNG便于浏览，PDF便于矢量导出。编号与两份报告一致。

| 图号 | 内容 | 数据或定义 |
| --- | --- | --- |
| 18 | 特征跟踪流程 | 代码流程示意，不是测量值 |
| 19 | 平均几何内点数 | 两段序列summary.csv的inliers |
| 20 | 平均内点率 | 每帧对inliers/good再平均 |
| 21 | 平均计算耗时 | ms；对数纵轴，不含读取与绘图 |
| 22 | 筛选保留率 | 每帧对good/raw再平均；不包含LK |
| 23 | 盒子图对筛选阶段 | raw、good、inliers；对数纵轴 |
| 24 | 内点率时序变化 | 51帧对后向平均，仅用于画图 |

`chart_data.json` 保留分组柱状图所用数值。正文表格采用完整原始数据，不采用平滑曲线的值。
