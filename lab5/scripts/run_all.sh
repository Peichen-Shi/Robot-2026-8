#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=$HOME/lab5_complete/opencv/lib:${LD_LIBRARY_PATH:-}
cd ~/lab5_complete
exe=$HOME/lab5_complete/ws/devel/lib/lab_5/lab5_evaluate
for idx in 1 2; do
 if [ "$idx" = 1 ];then bag=data/30fps_424x240_2018-10-01-18-35-06.bag;else bag=data/vnav-lab5-smooth-trajectory.bag;fi
 while [ ! -f "$bag" ]; do sleep 10; done
 mkdir -p results/dataset$idx
 rosbag info "$bag" > logs/dataset${idx}_info.txt
 for method in SIFT SURF ORB FAST_BRIEF LK FLOW;do
  echo RUNNING dataset$idx $method
  "$exe" bag "$bag" results/dataset$idx "$method" > logs/dataset${idx}_${method}.log 2>&1
 done
done
echo ALL_EVALUATIONS_COMPLETE
