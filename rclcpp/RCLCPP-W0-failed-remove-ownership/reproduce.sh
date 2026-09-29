#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
cmake -S . -B build
cmake --build build -j2
./build/rclcpp_waitset_probe
