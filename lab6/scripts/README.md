# 复现实验脚本

| 脚本 | 用途 |
| --- | --- |
| [build_lab6.sh](build_lab6.sh) | 在 ROS Noetic 和已编译的 Lab5 上构建 OpenGV 与 Lab6 |
| [convert_office.py](convert_office.py) | 从 Office ROS 2 SQLite 读取所需消息，转换成 ROS 1 |
| [evaluate.sh](evaluate.sh) | 离线评估所有方法，生成逐帧误差和小型位姿包 |
| [demo.sh](demo.sh) | 在 RViz 中显示某种方法的估计位姿 |
| [full_office.sh](full_office.sh) | 继续下载完整归档、解压、转换并启动完整评估 |
| [build_reports.py](build_reports.py) | 从 CSV 重新生成图表和双语 PDF |
| [record_lab4.sh](record_lab4.sh) | 可选 Lab4 RGB-D 采集，现有仿真器的深度初始化失败 |

Ubuntu 上需要 ROS Noetic、已编译的 Lab5、Eigen、CMake 和 Git。脚本默认在 `~/lab5_complete` 找 Lab5；若路径不同，可设置 `LAB5_ROOT`。

```bash
bash scripts/build_lab6.sh
python3 -m pip install --target python_deps rosbags==0.9.23
bash scripts/full_office.sh
```

若已有转换后的 ROS 1 输入包，可运行 `bash scripts/evaluate.sh /绝对路径/office.bag`。报告生成需要 Windows 宋体和黑体，以及 Python 的 matplotlib、Pillow 和 reportlab。

完整归档下载失败时，可用已保存的片段输入包重跑本报告的范围。转换脚本支持用原始归档的前 490188800 字节复现该片段，须先核对下载响应和文件长度。全量转换请使用完整 ZIP 解出的 SQLite 文件。完整数据报告可传入 `--results office_full --full` 重新生成。
