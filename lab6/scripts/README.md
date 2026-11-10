# 复现实验脚本

| 脚本 | 用途 |
| --- | --- |
| [build_lab6.sh](build_lab6.sh) | 在 ROS Noetic 和已编译的 Lab5 上构建 OpenGV 与 Lab6 |
| [convert_office.py](convert_office.py) | 从 Office ROS 2 SQLite 读取所需消息，转换成 ROS 1 |
| [evaluate.sh](evaluate.sh) | 离线评估所有方法，生成逐帧误差和小型位姿包 |
| [demo.sh](demo.sh) | 在 RViz 中显示某种方法的估计位姿 |
| [full_office.sh](full_office.sh) | 继续下载完整归档、解压、转换并启动完整评估 |
| [capture_evidence.py](capture_evidence.py) | 在已配置的 Ubuntu 桌面显示实验输出并补拍 09–16 号截图 |
| [build_reports.py](build_reports.py) | 从 CSV 重新生成图表和双语 PDF |
| [record_lab4.sh](record_lab4.sh) | 可选 Lab4 RGB-D 采集，现有仿真器的深度初始化失败 |

## 按课程公告转换完整 Office 数据

下载并解压[课程更新后的 Office 归档](https://drive.google.com/file/d/1TohOnfjR_FKf_y9wYTEkmv-XCuFPFRPt/view)。在解压得到的 `vnav-lab6-office/` 的上一级目录，使用 **Python 3.10 或更新版本**运行：

```bash
python3 -m pip install rosbags==0.11.9
rosbags-convert --src ./vnav-lab6-office --dst office_ros1.bag
```

Ubuntu 20.04 的系统 Python 通常是 3.8。可在 Windows 的 Python 3.10+ 环境运行转换，在 Windows 终端将第一条命令的 `python3` 换成环境中的 `python`。把生成的 `office_ros1.bag` 复制到 Ubuntu，再执行：

```bash
cd ~/lab6_complete
bash evaluate.sh "$HOME/lab6_complete/data/office_ros1.bag" "$HOME/lab6_complete/results/office_full"
```

| 路线 | 用途 |
| --- | --- |
| 课程公告的 `rosbags-convert` | 将完整 ROS 2 录包转为 ROS 1 |
| 本仓库的 `convert_office.py` | 兼容现有 Ubuntu 20.04 环境，仅转换实验使用的话题，也可处理已获取的连续片段 |
| 当前报告与图表 | 基于已验证的 213 对画面，全量统计需在完整录包转换后生成 |

## Ubuntu 20.04 上的兼容路线

Ubuntu 上需要 ROS Noetic、已编译的 Lab5、Eigen、CMake 和 Git。脚本默认在 `~/lab5_complete` 找 Lab5；若路径不同，可设置 `LAB5_ROOT`。

```bash
bash scripts/build_lab6.sh
python3 -m pip install --target python_deps rosbags==0.9.23
bash scripts/full_office.sh
```

若已有转换后的 ROS 1 输入包，可运行 `bash scripts/evaluate.sh /绝对路径/office.bag`。报告生成需要 Windows 宋体和黑体，以及 Python 的 matplotlib、Pillow 和 reportlab。

完整归档下载失败时，可用已保存的片段输入包重跑本报告的范围。转换脚本支持用原始归档的前 490188800 字节复现该片段，须先核对下载响应和文件长度。全量转换请使用完整 ZIP 解出的 SQLite 文件。完整数据报告可传入 `--results office_full --full` 重新生成。
