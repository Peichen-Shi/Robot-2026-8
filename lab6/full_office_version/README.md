# LAB6 无人机视觉定位

本文件夹包含 MIT VNAV 2023 Lab6 的 ROS Noetic 实现、Deliverable 3–5 团队中文报告、全量误差数据和复现实验脚本。原始 ROS2 数据未修改。

## 虚拟机中的位置

- 工作区：`/home/joe/lab6_ws`
- 源码包：`/home/joe/lab6_ws/src/lab6`
- ROS1 实验数据：`/home/joe/data/vnav/office.bag`
- 完整结果：`/home/joe/lab6_results`
- 原始共享数据：`/mnt/hgfs/vnav-lab6-office`

## 完整定量评测

```bash
source /opt/ros/noetic/setup.bash
cd ~/lab6_ws
catkin_make -j2
source devel/setup.bash
bash src/lab6/scripts/run_all.sh
```

`run_all.sh` 完整读取 3890 对同步 RGB/depth 图像，对 3889 个相邻帧对分别执行 Nister 五点、Longuet-Higgins 八点、已知旋转两点与 RGB-D Arun 三点 RANSAC。所有算法共享 SIFT 匹配。报告由真实 CSV 自动生成。重复运行会覆盖指定输出目录，可用 `bash scripts/run_all.sh /home/joe/lab6_results_new` 指定新目录。

## ROS 与 RViz 演示

```bash
source ~/lab6_ws/devel/setup.bash
roslaunch lab_6 video_tracking.launch
```

可选参数：`pose_estimator:=0`（5点）、`1`（8点）、`2`（已知旋转2点）、`3`（Arun）、`-1`（全部，默认），`rviz:=false`，`rate:=0.15`。ROS 在线运行按系统负载可能丢帧；提交的定量结果使用离线逐帧处理，无降采样。

发布话题：`/lab6/pose_5pt`、`/lab6/pose_8pt`、`/lab6/pose_2pt`、`/lab6/pose_arun`、`/gt_camera_pose`，以及 `/lab6/path_*`。全部为 world 坐标系。可视化使用上一帧真值锚定，单目恢复尺度也使用真值；这些轨迹不是独立累积的视觉里程计。

## 数据转换与依赖

Ubuntu 20.04 默认 Python 3.8，使用兼容的 `rosbags==0.9.23` 完成 ROS2 到 ROS1 转换（旧版参数使用位置参数，新版使用 `--src`）。转换只保留 RGB、depth、camera_info、odom 四个实验必需话题，共 59070 条消息，约 8.8 GiB。ROS2 CameraInfo 使用小写字段 d/k/r/p，转换后通过 `scripts/fix_camera_info.py` 修正 ROS1 字段名和 MD5，二进制数值保持不变。

安装脚本：`scripts/install_deps.sh`。OpenGV 安装在工作区内，不替换系统库。OpenCV 4.2 缺少 SIFT，因此包内包含官方 contrib 4.2 的 SIFT 源码及许可。C++17 和 `-march=native` 与 OpenGV 保持一致，避免 Eigen 对齐问题。

OpenGV commit: `91f4b19c73450833a40e463ad3648aae80b3a7f3`。
OpenCV contrib 4.2.0 commit: `65abc7090dedc84bbedec4dfd143f0340e52114f`。

## 参数及结果解释

- SIFT：最多1600个特征；FLANN k=2；Lowe ratio=0.7；去除重复目标索引。
- RANSAC：最大1000次；置信度0.999；固定随机种子。单目阈值 `2*(1-cos(atan(0.25/fx)))`；Arun为0.05米的欧氏距离。
- 旋转误差为规定的 Frobenius 弦距，另附角度误差；单目平移比较单位方向，范围0–2；Arun平移以米计。
- 两点法按题意使用真值相对旋转，其零旋转误差是已知输入带来的结果。
- `rpe.csv` 包含每一帧对、每种算法的状态、匹配数、内点数、基线、误差和求解时间；失败不使用真值填补。
- `summary.json` 提供均值、中位数、P95、RMSE和成功数；完整误差曲线不平滑、不裁剪尖峰。
- 未开展可选的 Lab4 赛车数据重采集。GitHub 上传记录见随附的提交说明。

实验要求：https://vnav.mit.edu/labs_2023/lab6/exercises.html

2026-11-11 的实测截图保留在 results 中。若演示进程已关闭，请重新运行 scripts/demo.sh。

## 本次提交版本

这是 2026-11-16 整理的完整 Office 数据版本。仓库原有版本保留；本版本上传到 `lab6/full_office_version/`。
将本目录复制为 `~/lab6_ws/src/lab6` 后按上述步骤构建。不要把两个同名 ROS 包同时放入同一个 catkin 工作区。
个人 Deliverable 1–2 与完整实验报告分别位于本次交付材料的“个人部分”“实验报告部分”。团队报告为 `LAB6_Team_Report.pdf`。
版本对比见 [COMPARISON.md](COMPARISON.md)。新建输出目录现由程序自动创建；相机反投影报告公式已修正为 `X = z * f / f_z`。
