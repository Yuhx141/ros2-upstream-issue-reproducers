#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
cmake -S . -B "$tmp/build"; cmake --build "$tmp/build" -j2
python3 run_occupancy_init_probe.py --executable "$tmp/build/occupancy_init_probe" --source occupancy_init_probe.cpp --output "$tmp/results" --repetitions 1
cat "$tmp/results/summary.json"
