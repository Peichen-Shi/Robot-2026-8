#!/bin/bash
set -e
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
cd ~/lab5_complete
while [ ! -f results/dataset2/LK_summary.csv ] || [ ! -f results/dataset2/Farneback.png ]; do sleep 3; done
xrandr --output Virtual1 --mode 1280x800
for idx in 1 2; do
 gnome-terminal --maximize --title=LAB5_Dataset${idx}_Statistics -- bash -c "python3 ~/lab5_complete/print_stats.py dataset$idx; exec bash"
 sleep 2
 if [ "$idx" = 1 ];then file=10_Dataset1_Statistics;else file=11_Dataset2_Statistics;fi
 gnome-screenshot -f "screenshots/$file.png"
done
python3 capture_evidence.py
source ws/devel/setup.bash
export LD_LIBRARY_PATH=$HOME/lab5_complete/opencv/lib:${LD_LIBRARY_PATH:-}
timeout --signal=INT 16s roslaunch lab_5 video_tracking.launch descriptor:=LK path_to_dataset:=$HOME/lab5_complete/data/30fps_424x240_2018-10-01-18-35-06.bag > logs/roslaunch_lk_video.log 2>&1 &
sleep 7
gnome-screenshot -f screenshots/17_ROS_Live_LK_Video.png
wait || true
echo FINAL_SCREENSHOTS_COMPLETE
