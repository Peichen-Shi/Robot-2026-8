# Lab5逐帧统计与结果图

| 资料 | 入口 |
| --- | --- |
| 运行画面 | [编号截图](screenshots/README.md) |
| 流程和统计图 | [图表](charts/README.md) |
| 编译与运行记录 | [日志](logs/README.md) |
| 文件校验 | [SHA256 清单](MANIFEST_SHA256.txt) |

[返回Lab5](../README.md) · [统计图表](charts/README.md)

| 目录 | 内容 |
| --- | --- |
| [pair](pair/) | box与box_in_scene双图结果，四种描述子方法 |
| [dataset1](dataset1/) | 第一段真实序列，五种方法各1583个帧对 |
| [dataset2](dataset2/) | 第二段真实序列，五种方法各1385个帧对 |

`方法_frames.csv`按帧对记录；`方法_summary.csv`保存均值。PNG是算法输出图，实际桌面截图另在screenshots目录，不把两者混为一类。

| 列 | 含义 |
| --- | --- |
| frame | 当前方法的帧对序号 |
| keypoints1 / keypoints2 | 两帧的关键点数；LK的第二列为成功跟踪数 |
| matches | 每个查询点的最佳候选；LK为有效跟踪 |
| good | 距离比筛选后数量；LK不使用距离比 |
| inliers | RANSAC模型内点数 |
| inlier_ratio | 本帧对inliers/good，空集合记0 |
| ms | 计算耗时，不含文件读取和绘图 |

汇总内点率是逐帧比率的算术平均，不是累计数相除。完整说明与局限见报告；模型内点不能当作人工真值准确率。
