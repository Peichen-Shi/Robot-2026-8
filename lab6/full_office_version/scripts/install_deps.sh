#!/usr/bin/env bash
set -euo pipefail
# ROS Noetic / Ubuntu 20.04: use system OpenCV, Eigen and cv_bridge.
sudo apt-get update
sudo apt-get install -y libopencv-dev libeigen3-dev python3-pip python3-matplotlib python3-reportlab ros-noetic-cv-bridge ros-noetic-rosbag
python3 -m pip install --user rosbags==0.9.23
mkdir -p "$HOME/lab6_work" "$HOME/data/vnav"
cd "$HOME/lab6_work"
test -d opengv || git clone --depth 1 https://github.com/laurentkneip/opengv.git
git -C opengv fetch --depth 1 origin 91f4b19c73450833a40e463ad3648aae80b3a7f3
git -C opengv checkout --detach 91f4b19c73450833a40e463ad3648aae80b3a7f3
cmake -S opengv -B opengv-build -DBUILD_TESTS=OFF -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX="$HOME/lab6_ws/opengv_install"
cmake --build opengv-build -j2
cmake --install opengv-build
# rosbags 0.9.23 supports Ubuntu 20.04's Python 3.8. Newer versions use --src.
test -f "$HOME/data/vnav/office.bag" || "$HOME/.local/bin/rosbags-convert" /mnt/hgfs/vnav-lab6-office --dst "$HOME/data/vnav/office.bag" --include-topic /tesse/odom --include-topic /tesse/left_cam/rgb/image_raw --include-topic /tesse/left_cam/camera_info --include-topic /tesse/depth_cam/mono/image_raw
python3 "$HOME/lab6_ws/src/lab6/scripts/fix_camera_info.py" "$HOME/data/vnav/office.bag"
