#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
OUT=$1
if [ -z "$OUT" ]; then OUT="$HERE/run-$(date +%Y%m%d-%H%M%S)"; fi
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
for f in ekf ukf; do
 python3 "$HERE/reproduce.py" --out-dir "$OUT/$f-valid" --ros-domain-id 220 --filter "$f" --scenario valid-set-pose
 python3 "$HERE/reproduce.py" --out-dir "$OUT/$f-invalid" --ros-domain-id 221 --filter "$f" --scenario current-invalid-frame || test $? -eq 3
done
