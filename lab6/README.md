# Lab6 · 无人机视觉定位

无人机看了前后两帧，能判断自己转了多少、向哪儿飞吗？先用 Lab5 的 SIFT 找到同一个角点，再用不同的位姿算法“解这道题”。

**本次已验证 MIT Office 归档的前段，共 213 对相邻画面。四种必做方法和无 RANSAC 对照均已运行。当前报告以片段实测为准。**

| 想找什么 | 入口 |
| --- | --- |
| 中文报告 | [直接打开 PDF（18 页）](reports/LAB6_无人机视觉定位实验报告.pdf) |
| 英文报告 | [Open English PDF (18 pages)](reports/LAB6_Visual_Localization_Report_EN.pdf) |
| 现场演示 | [Ubuntu 命令与检查清单](reports/演示操作文档.md) |
| 看代码 | [ROS 包与位姿求解入口](code/lab6/src/pose_estimation.cpp) |
| 看结果 | [统计、4 张误差图、12 张截图及日志](results/README.md) |
| 重跑实验 | [构建、转换和运行脚本](scripts/README.md) |

| 任务 | 做了什么 |
| --- | --- |
| D1 · 五点法（10 分） | 从五对图像点推导本质矩阵，说明多解、退化与 RANSAC 次数 |
| D2 · 最小求解器（10 分） | 已知旋转时，用两对点给出平移方向，并写出 RANSAC 步骤 |
| D3 · 初始配置（10 分） | 读取 RGB、深度和里程计，核对相机内参，用 RViz 回放 |
| D4 · 二维对应（40 分） | 比较五点、八点、已知旋转两点，另做无 RANSAC 对照 |
| D5 · 三维对应（30 分） | 用深度把像素变成三维点，做 Arun + RANSAC 并画米制误差 |

| 方法 | 一句话看懂 |
| --- | --- |
| Nistér 五点 | 用五对匹配点找可行的旋转和平移方向，再让更多点投票 |
| Longuet-Higgins 八点 | 用更多点建立线性方程估计本质矩阵 |
| 已知旋转两点 | 把旋转当已知量，两对点确定平移方向。实验以真值旋转模拟 IMU 输入 |
| Arun 三点 | 给两帧的三维点云做刚体对齐，恢复带米制尺度的平移 |
| 五点无 RANSAC | 用描述子最接近的一组点只提出一次假设，作为外点对照 |

代码依赖 Lab5 的 SIFT 实现和 OpenGV。原始 Office 归档及转换后的大型输入包留在本地；仓库中的小型位姿包用于 RViz，需配合原始图像包。旧 ROS 1 链接返回 404，本次数据来自 [MIT 新版 Office 归档](https://drive.google.com/file/d/1TohOnfjR_FKf_y9wYTEkmv-XCuFPFRPt/view)。可选 Lab4 RGB-D 录制未成功，深度相机元数据解析日志见结果目录。

课程已确认更新后的 Office 数据采用 ROS 2 格式。完整录包可用公告指定的 `rosbags==0.11.9` 转为 ROS 1，见[转换命令](scripts/README.md#按课程公告转换完整-office-数据)。
