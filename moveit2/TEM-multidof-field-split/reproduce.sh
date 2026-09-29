#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
python3 run_tem_mdof_probe.py results
cat results/summary.json
