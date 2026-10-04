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
git -C "$REPO" diff --quiet -- imu_sensor_broadcaster/test/test_imu_sensor_broadcaster.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ -n "${EXTRA_CMAKE_PREFIX_PATH:-}" ]; then
  export CMAKE_PREFIX_PATH="$EXTRA_CMAKE_PREFIX_PATH:${CMAKE_PREFIX_PATH:-}"
fi
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/imu_sensor_broadcaster" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select imu_sensor_broadcaster --allow-overriding imu_sensor_broadcaster \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source "$TMP/install/setup.bash"
BIN="$TMP/build/imu_sensor_broadcaster/test_imu_sensor_broadcaster"
PARAMS="$REPO/imu_sensor_broadcaster/test/imu_sensor_broadcaster_params.yaml"
set +e
"$BIN" --gtest_filter=IMUSensorBroadcasterTest.ExportedStateInterfacesTrackUpdates \
  --ros-args --params-file "$PARAMS" >"$TMP/test.log" 2>&1
RC=$?
set -e
cat "$TMP/test.log"
test "$RC" -ne 0
PUBLISHED_FAILURES=$(grep -c 'published\[i\]' "$TMP/test.log" || true)
EXPORTED_FAILURES=$(grep -c 'exported.value()' "$TMP/test.log" || true)
test "$PUBLISHED_FAILURES" -eq 0
test "$EXPORTED_FAILURES" -eq 20
printf 'IMU_EXPORTED_STATE_RESULT=STALE PUBLISHED_FAILURES=%s EXPORTED_FAILURES=%s\n' \
  "$PUBLISHED_FAILURES" "$EXPORTED_FAILURES"
