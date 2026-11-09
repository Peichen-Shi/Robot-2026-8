#!/usr/bin/env bash
# Record the Lab4 flight with RGB, registered depth and body odometry enabled.
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
root="$script_dir"
[[ -d "$root/code/lab6" ]] || root="$(dirname "$script_dir")"
race=${LAB4_RACE_ROOT:-$HOME/lab4_2p67_run/lab4}
source "$race/env.sh"
export ROS_MASTER_URI=http://127.0.0.1:11326
cd "$race"
mkdir -p "$root"/{logs,data,screenshots}
pids=()
cleanup() {
  for ((i=${#pids[@]}-1; i>=0; --i)); do
    kill -INT -- "-${pids[i]}" 2>/dev/null || true
    wait "${pids[i]}" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() { local log=$1; shift; setsid "$@" >"$root/logs/$log" 2>&1 & pids+=("$!"); }
start capture_roscore.log roscore -p 11326
start capture_unity_stdout.log "$HOME/vnav-builds/lab4/lab4.x86_64" -screen-fullscreen 0 -screen-width 720 -screen-height 480 -screen-quality Fastest -job-worker-count 1 -logFile "$root/logs/capture_unity.log"
sleep 7
start capture_bridge.log roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_stereo_rgb:=true publish_depth:=true publish_odom:=true publish_collisions:=true publish_metadata:=true clock_from_metadata_udp:=true persistent_metadata_listener:=true frame_rate:=15.0
timeout 30 rostopic echo -n 1 /tesse/depth_cam/camera_info >/dev/null
start capture_bag.log rosbag record -O "$root/data/lab4_race_rgbd.bag" /tesse/left_cam/rgb/image_raw /tesse/right_cam/rgb/image_raw /tesse/depth_cam/mono/image_raw /tesse/left_cam/camera_info /tesse/depth_cam/camera_info /current_state
sleep 3
gnome-screenshot -f "$root/screenshots/01_TESSE_RGBD_Recording.png" || true
export LAB4_PLANNER_PROFILE="$race/optimization/planner_profiles/sub27.yaml"
export LAB4_SPEED_RESULTS="$root/results/flight"
export LAB4_EXIT_OBSERVATION=5 LAB4_SHOW_TIMER=1 LAB4_KEEP_FINAL_HOLD=0
duration=$(python3 -c 'import sys,yaml; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
bash optimization/run_trial.sh "$race/optimization/profiles/sub27.yaml" "$duration" lab6_rgbd 2.67 8
