#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
cp -a . "$tmp/src"
cd "$tmp"
colcon build --base-paths src --packages-select joint_transmission_p0 --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
python3 src/run_p0.py results install/joint_transmission_p0/lib/joint_transmission_p0/joint_transmission_p0_test --repetitions 1
