#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://localhost:11318 rosservice call /office_bag/pause_playback false
