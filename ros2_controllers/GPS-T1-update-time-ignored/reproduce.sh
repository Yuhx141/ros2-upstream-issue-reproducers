#!/usr/bin/env bash
set -eo pipefail
REPO=$(realpath "${1:?usage: $0 /path/to/ros2_controllers}")
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
TMP=$(mktemp -d)
APPLIED=0
cleanup() {
  if [ "$APPLIED" = 1 ]; then git -C "$REPO" apply -R "$HERE/test.patch"; fi
  rm -rf "$TMP"
}
trap cleanup EXIT
git -C "$REPO" diff --quiet -- gps_sensor_broadcaster/test/test_gps_sensor_broadcaster.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ -n "${EXTRA_CMAKE_PREFIX_PATH:-}" ]; then
  export CMAKE_PREFIX_PATH="$EXTRA_CMAKE_PREFIX_PATH:${CMAKE_PREFIX_PATH:-}"
fi
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/gps_sensor_broadcaster" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select gps_sensor_broadcaster --allow-overriding gps_sensor_broadcaster \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source "$TMP/install/setup.bash"
set +e
"$TMP/build/gps_sensor_broadcaster/test_gps_sensor_broadcaster" \
  --gtest_filter=GPSSensorBroadcasterTest.update_time_should_stamp_published_message \
  >"$TMP/test.log" 2>&1
RC=$?
set -e
cat "$TMP/test.log"
test "$RC" -ne 0
test "$(grep -c 'message.header.stamp.sec' "$TMP/test.log" || true)" -eq 1
test "$(grep -c 'message.header.stamp.nanosec' "$TMP/test.log" || true)" -eq 1
printf 'GPS_TIMESTAMP_RESULT=NODE_TIME DATA_FAILURES=0 STAMP_FAILURES=2\n'
