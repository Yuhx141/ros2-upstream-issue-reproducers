#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
rm -rf results
python3 moveit_pipeline_p0.py --out-dir results --repetitions 1
cat results/summary.json
