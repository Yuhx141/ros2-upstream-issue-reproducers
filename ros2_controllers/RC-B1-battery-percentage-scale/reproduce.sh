#!/usr/bin/env bash
set -euo pipefail
REPO=$(realpath "${1:?usage: $0 /path/to/ros2_controllers}")
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
TMP=$(mktemp -d)
APPLIED=0
cleanup() {
  if [ "$APPLIED" = 1 ]; then git -C "$REPO" apply -R "$HERE/test.patch"; fi
  rm -rf "$TMP"
}
trap cleanup EXIT
git -C "$REPO" diff --quiet -- battery_state_broadcaster/test/test_battery_state_broadcaster.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/battery_state_broadcaster" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select battery_state_broadcaster --symlink-install
source "$TMP/install/setup.bash"
BIN="$TMP/build/battery_state_broadcaster/test_battery_state_broadcaster"
PARAMS="$REPO/battery_state_broadcaster/test/battery_state_broadcaster_params.yaml"
"$BIN" --ros-args --params-file "$PARAMS" -- \
  --gtest_filter=BatteryStateBroadcasterTest.empty_charge_is_zero_control
set +e
"$BIN" --ros-args --params-file "$PARAMS" -- \
  --gtest_filter=BatteryStateBroadcasterTest.percentage_matches_sensor_msgs_contract:BatteryStateBroadcasterTest.full_charge_is_one_in_sensor_msgs_contract
RC=$?
set -e
test "$RC" -ne 0
printf 'Reproduced: contract tests failed while the zero control passed.\n'
