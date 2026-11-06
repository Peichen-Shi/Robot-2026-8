# feature_tracking ROS 包

这是Lab5的可编译C++包，包名为`lab_5`。先看[实验总说明](../README.md)，再按下面的入口阅读代码。

## 从输入到结果

1. `track_features.cpp`读取两幅图，或者订阅`/images_topic`。
2. 描述子方法调用`FeatureTracker::trackFeatures`：检测点、计算描述子、匹配、筛选并检查极线几何。
3. `LKFeatureTracker`保存上一帧和角点，通过局部光流追踪，点数不足时重新检测。
4. `self_flow.cpp`为整幅图像计算Farneback光流。
5. `evaluate.cpp`按顺序读取完整rosbag，生成逐帧CSV及平均值，是报告统计的来源。

## 输入与输出约定

- `detectKeypoints`输入图像、输出关键点。
- `describeKeypoints`可能删除边界点，关键点向量与描述子行必须保持对应。
- `matchDescriptors`输出二近邻列表和通过严格0.8距离比的候选。
- 几何掩码和候选向量采用同一顺序；绿色为模型内点，红色为外点。
- 空描述子、二近邻不足、少于8个候选和零分母均有保护。

## 构建和运行

依赖ROS Noetic、glog和启用nonfree的OpenCV 4.2 contrib。不要把普通系统OpenCV与旧版opencv3_catkin同时加入此包。完整依赖构建命令见[上级README](../README.md)。

```bash
source ~/lab5_complete/ws/devel/setup.bash
export LD_LIBRARY_PATH=$HOME/lab5_complete/opencv/lib:$LD_LIBRARY_PATH
roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT
```

双图模式支持`SIFT`、`SURF`、`ORB`、`FAST`；视频模式另外支持`LK`。离线评估器把FAST+BRIEF写作`FAST_BRIEF`，这是命令接口的区别，不是不同算法。

本包基于MIT-SPARK VNAV2021教学模板完成；修改包含算法实现、OpenCV4构建兼容和清晰注释。详见[报告和来源](../README.md#截图图表与引用)。
