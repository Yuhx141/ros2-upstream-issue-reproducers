#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
OUT=$1
if [ -z "$OUT" ]; then OUT="$HERE/run-$(date +%Y%m%d-%H%M%S)"; fi
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
python3 "$HERE/reproduce.py" --out-dir "$OUT" --ros-domain-id 220 --scenario missing --expected-library "$(ros2 pkg prefix ros_gz_sim)/lib/libgzserver_component.so"
