#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
OUT=$1
if [ -z "$OUT" ]; then OUT="$HERE/run-$(date +%Y%m%d-%H%M%S)"; fi
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WORK=/tmp/nav2-costmap-atomicity-$$
colcon --log-base "$WORK/log" build --base-paths "$HERE/probe_package" --build-base "$WORK/build" --install-base "$WORK/install" --packages-select p3_costmap_atomicity_probe --cmake-args -DBUILD_TESTING=OFF
python3 "$HERE/run.py" --binary "$WORK/install/p3_costmap_atomicity_probe/lib/p3_costmap_atomicity_probe/costmap_atomicity_probe" --out-dir "$OUT" --repetitions 3
