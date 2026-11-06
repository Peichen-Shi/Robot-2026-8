# Lab5 Visual Feature Tracking

[中文说明](README.md) · [Chinese report](deliverables/LAB5_无人机视觉跟踪实验报告.pdf) · [English report](deliverables/LAB5_Visual_Tracking_Report_EN.pdf) · [Screenshot index](screenshots/README.md) · [Charts](charts/README.md)

Both reports contain 19 pages, 17 actual Ubuntu screenshots, seven reproducible diagrams/charts and 11 tables. They follow the chapter, header, caption and blue-gray table style of the repository's Lab3/Lab4 reports. Their measurements are independent of those earlier labs.

## What was implemented

- Sphere projection and vanishing-point derivations.
- SIFT/SURF floating-point descriptors with FLANN, and ORB/FAST+BRIEF binary descriptors with Hamming matching.
- Strict two-neighbor ratio filtering and fundamental-matrix RANSAC.
- Harris corners with pyramidal LK and the optional Farneback dense flow.
- Full offline traversal of 1,583 and 1,385 neighboring RGB frame pairs.

## Reading the source

`feature_tracker.cpp` contains the shared descriptor pipeline. The four adapter files choose detection, description and matching. `lk_feature_tracker.cpp` retains previous-frame state. `track_features.cpp` provides the ROS entry point; `evaluate.cpp` supplies the ordered rosbag evaluation used in the report. Comments explain assumptions, corner cases and statistic definitions. Formatting/comment cleanup preserves all non-comment C++ tokens in 15 files; a fresh build and box-pair check passed.

## Important definitions

The reported inlier ratio is the average of each pair's inliers/good. LK has no descriptor ratio test, and its second keypoint count means valid tracks rather than independently detected image-2 points. Inliers are consistent with an estimated model, not human ground truth. Timings exclude image reading and drawing and are affected by VM scheduling.

## Running and reproducing

Use Ubuntu 20.04, ROS Noetic, GCC 9.4 and OpenCV 4.2.0 contrib with nonfree enabled. The Chinese guide contains complete shell commands; they are shared by both language versions. Build in `~/lab5_complete/ws`, download the two prescribed bags into `~/lab5_complete/data`, then run `scripts/run_all.sh`. Descriptor names are `FAST` in the ROS node and `FAST_BRIEF` in the offline evaluator.

The repository keeps source, measurements, reports and evidence. The approximately 2.1 GiB raw bags and compiled dependency archives remain on the desktop and VM. `datasets.json` records hashes. Charts can be recreated with `scripts/make_report_charts.py`. The current bilingual PDF generator uses Windows SimSun/SimHei fonts; experiment execution is on Ubuntu.

## Provenance

The template is MIT-SPARK VNAV2021 revision 2443ec7e491001d09ab8dd66f6b953177043b784. Source references, parameter details, the full screenshot index and measurement limitations are documented in the reports and Chinese guide. Figures 01-17 are actual desktop captures; Figures 18-24 are generated from saved data or the explicit pipeline.
