#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
cmake -S . -B "$tmp/build"; cmake --build "$tmp/build" -j2
python3 run_psm_stop_world_probe.py --executable "$tmp/build/psm_stop_world_probe" --source psm_stop_world_probe.cpp --output "$tmp/results" --repetitions 1
cat "$tmp/results/summary.json"
