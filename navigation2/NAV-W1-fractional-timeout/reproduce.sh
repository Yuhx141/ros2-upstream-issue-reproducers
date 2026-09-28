#!/usr/bin/env bash
set -eo pipefail
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
OUT=$1
if [ -z "$OUT" ]; then OUT="$HERE/run-$(date +%Y%m%d-%H%M%S)"; fi
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
python3 "$HERE/reproduce.py" --output "$OUT" --repetitions 3 --domain-base 180
