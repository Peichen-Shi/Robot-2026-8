#!/bin/bash
set -eu
cd ~/lab5_complete/sources
curl -L --retry 3 -o opencv.tgz https://codeload.github.com/opencv/opencv/tar.gz/refs/tags/4.2.0
curl -L --retry 3 -o contrib.tgz https://codeload.github.com/opencv/opencv_contrib/tar.gz/refs/tags/4.2.0
tar xf opencv.tgz
tar xf contrib.tgz
cmake -S opencv-4.2.0 -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/lab5_complete/opencv -DOPENCV_EXTRA_MODULES_PATH=$HOME/lab5_complete/sources/opencv_contrib-4.2.0/modules -DOPENCV_ENABLE_NONFREE=ON -DBUILD_LIST=core,imgproc,imgcodecs,highgui,video,videoio,features2d,flann,calib3d,xfeatures2d -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_opencv_apps=OFF -DWITH_IPP=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_OPENCL=OFF -DWITH_QT=OFF -DWITH_GTK=ON -DOPENCV_GENERATE_PKGCONFIG=ON
python3 "$(dirname "$0")/fix_downloads.py"
cmake -S opencv-4.2.0 -B build
cmake --build build -j4
cmake --install build
echo OPENCV_BUILD_SUCCESS
