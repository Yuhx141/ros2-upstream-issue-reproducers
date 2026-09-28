#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
OUT=$1
if [ -z "$OUT" ]; then OUT="$HERE/run-$(date +%Y%m%d-%H%M%S)"; fi
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
LIB="$(ros2 pkg prefix moveit_core)/lib"
python3 "$HERE/reproduce.py" --out-dir "$OUT/empty" --ros-domain-id 220 --scenario attach-transfer --planning-scene-lib-dir "$LIB" --plan-after-attach
python3 "$HERE/reproduce.py" --out-dir "$OUT/explicit" --ros-domain-id 221 --scenario attach-transfer --planning-scene-lib-dir "$LIB" --plan-after-attach --explicit-touch-link
