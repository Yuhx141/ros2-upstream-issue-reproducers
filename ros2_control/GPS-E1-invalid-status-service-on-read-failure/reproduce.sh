#!/usr/bin/env bash
set -eo pipefail
REPO=$(realpath "${1:?usage: $0 /path/to/ros2_control}")
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
TMP=$(mktemp -d)
APPLIED=0
cleanup() {
  if [ "$APPLIED" = 1 ]; then git -C "$REPO" apply -R "$HERE/test.patch"; fi
  rm -rf "$TMP"
}
trap cleanup EXIT
git -C "$REPO" diff --quiet -- controller_interface/test/test_gps_sensor.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/rolling/setup.bash}"
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/controller_interface" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select controller_interface --allow-overriding controller_interface \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source "$TMP/install/setup.bash"
set +e
"$TMP/build/controller_interface/test_gps_sensor" \
  --gtest_filter=GPSSensorTest.failed_status_and_service_reads_should_use_unknown_enums \
  >"$TMP/test.log" 2>&1
RC=$?
set -e
cat "$TMP/test.log"
test "$RC" -ne 0
test "$(grep -c '(127)' "$TMP/test.log" || true)" -eq 1
test "$(grep -c 'Which is: 65535' "$TMP/test.log" || true)" -eq 1
printf 'GPS_READ_FAILURE_RESULT=INVALID STATUS=127 SERVICE=65535\n'
