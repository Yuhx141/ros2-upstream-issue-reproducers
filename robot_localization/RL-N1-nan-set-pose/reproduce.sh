#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
OUT=$1
if [ -z "$OUT" ]; then OUT="$HERE/run-$(date +%Y%m%d-%H%M%S)"; fi
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
d=220
for f in ekf ukf; do
 for s in valid-set-pose nan-set-pose valid-covariance nan-covariance; do
  python3 "$HERE/reproduce.py" --out-dir "$OUT/$f-$s" --ros-domain-id "$d" --filter "$f" --scenario "$s" || test $? -eq 3
  d=$((d+1))
 done
done
