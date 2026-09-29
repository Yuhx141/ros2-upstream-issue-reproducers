#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
rm -rf results
python3 slam_toolbox_p0.py --output results --repetitions 1 --cases pause_before_first
cat results/summary.json
