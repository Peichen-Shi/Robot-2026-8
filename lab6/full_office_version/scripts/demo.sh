#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source "$HOME/lab6_ws/devel/setup.bash"
mkdir -p "$HOME/lab6_results/live"
roslaunch lab_6 video_tracking.launch "$@"
