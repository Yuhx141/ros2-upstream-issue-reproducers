#!/usr/bin/env bash
set -euo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
out="${1:-results}"
python3 ros_gz_world_entity_p0.py --output "$out" --repetitions 1
python3 - "$out/summary.json" <<'PY'
import json, sys
x=json.load(open(sys.argv[1]))
print(json.dumps(x.get("counts", x), indent=2, sort_keys=True))
PY
