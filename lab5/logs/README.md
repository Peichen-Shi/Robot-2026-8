# Lab5运行与校验日志

[返回Lab5](../README.md)

| 记录 | 作用 |
| --- | --- |
| [catkin_build.log](catkin_build.log) | 原始构建过程 |
| [readability_build.log](readability_build.log) | 注释和格式整理后的重新编译 |
| [readability_validation.json](readability_validation.json) | 代码标记一致性与双图计数复核 |
| [linked_libraries.txt](linked_libraries.txt) | 实际链接的OpenCV库路径 |
| [dataset1_info.txt](dataset1_info.txt)、[dataset2_info.txt](dataset2_info.txt) | rosbag文件时长与消息信息 |
| [dataset_sha256.txt](dataset_sha256.txt) | 实际使用的数据文件指纹 |
| [roslaunch_sift.log](roslaunch_sift.log) | SIFT双图ROS启动 |
| [roslaunch_lk_video.log](roslaunch_lk_video.log) | LK视频ROS运行 |

`dataset1_方法.log`与`dataset2_方法.log`记录对应评估器输出。日志里的旧路径、时间字符串和终端控制字符属于当时运行记录，不为美观改写原始数值。
