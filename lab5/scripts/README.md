# Lab5脚本怎么用

[返回Lab5](../README.md)

| 目的 | 入口 | 运行环境与输出 |
| --- | --- | --- |
| 构建OpenCV | [build_opencv.sh](build_opencv.sh) | Ubuntu；独立安装至~/lab5_complete/opencv |
| 补齐构建模型文件 | [fix_downloads.py](fix_downloads.py) | 被构建脚本调用，下载官方依赖 |
| 下载两段bag | [download_individual.py](download_individual.py) | Ubuntu；保存至~/lab5_complete/data |
| 完整评估 | [run_all.sh](run_all.sh) | Ubuntu与ROS；输出results和logs |
| 显示统计 | [print_stats.py](print_stats.py) | 读取已完成的CSV汇总 |
| 采集实际截图 | [capture_evidence.py](capture_evidence.py)、[final_capture.sh](final_capture.sh) | Ubuntu桌面需可见；输出screenshots |
| 生成统计图 | [make_report_charts.py](make_report_charts.py) | 读取CSV，输出charts中的PNG与PDF |
| 生成双语报告 | [build_bilingual_reports.py](build_bilingual_reports.py) | Windows宋体和黑体；输出deliverables |
| 检查结果 | [verify_final.py](verify_final.py) | 检查帧对、统计量和17张截图 |

`report_content_cn.py`和`report_content_en.py`供排版器读取正文，不是推荐的生成入口。Python报告依赖见[requirements-report.txt](requirements-report.txt)。先下载或产生输入文件，再运行依赖它们的脚本。
