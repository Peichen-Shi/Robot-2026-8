# 最终录屏：首门至末门 3.896 秒

计时口径：实际通过第 1 门到实际通过第 12 门的 ROS 仿真时间。不是起飞时间，也不是视频播放时长。视频保持正常播放速度。

- 开始时间（首门）：15.12058246894151 s
- 结束时间（末门）：19.01632953838693 s
- 最终用时：3.89574706944542 s，视频显示 **3.896 s**
- 结果：12/12 门，race_acceptance 验收通过。
- 视频：`videos/Lab4_Sub4_20261029_232155_872702434.mp4`
- 同名 `_data/recording_result.json` 包含原始计时、验收和视频信息。
- 原始记录：`results/sub4_20261029_082156_245617044_01`，包含在 `deliverables/ALL_RUN_RESULTS_20261029.zip` 中。

视频显示 START (sim)、END (sim) 和 FINAL。首门通过后走表，末门通过后冻结，结束画面保留最终结果。先前的三次独立重启稳定性验证全部通过，详见 SUB4_RESULTS.md 和 evidence/sub4_qualified/。

一键重新录制并保存到 Ubuntu 桌面：
```bash
bash ~/lab4_sub4_20261029/record_sub4.sh
```
依赖 ffmpeg、xwininfo 和可见 Ubuntu 图形会话，录制期间保持 Unity 窗口可见。停止飞行：`bash ~/lab4_sub4_20261029/stop_sub4.sh`。

本提交保留 Fast61 历史材料，新增/更新 sub4 控制器、配置、源码、测试、优化过程与原始结果。全部历史试验结果归档中也包含未通过的探索试验；不应将其当作最终配置验收结果。未打包 ROS 编译缓存、系统依赖和 Unity 安装文件。
