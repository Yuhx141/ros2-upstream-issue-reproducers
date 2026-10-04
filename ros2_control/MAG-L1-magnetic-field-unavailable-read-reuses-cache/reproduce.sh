#!/usr/bin/env bash
set -eo pipefail

REPO=$(realpath "${1:?usage: $0 /path/to/ros2_control}")
HERE=$(cd -- "$(dirname -- "$0")" && pwd)
TMP=$(mktemp -d)
APPLIED=0

cleanup() {
  if [ "$APPLIED" = 1 ]; then
    git -C "$REPO" apply -R "$HERE/test.patch"
  fi
  rm -rf "$TMP"
}
trap cleanup EXIT

git -C "$REPO" diff --quiet -- controller_interface/test/test_magnetic_field_sensor.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1

source "${ROS_SETUP:-/opt/ros/rolling/setup.bash}"
if [ -n "${EXTRA_CMAKE_PREFIX_PATH:-}" ]; then
  export CMAKE_PREFIX_PATH="$EXTRA_CMAKE_PREFIX_PATH:${CMAKE_PREFIX_PATH:-}"
fi

colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/controller_interface" "$REPO/hardware_interface" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-up-to controller_interface \
  --allow-overriding controller_interface hardware_interface \
  --cmake-args -DBUILD_TESTING=ON

source "$TMP/install/setup.bash"
BIN="$TMP/build/controller_interface/test_magnetic_field_sensor"
set +e
"$BIN" --gtest_filter='MagneticFieldSensorAvailabilityTest.*' > "$TMP/test.log" 2>&1
RC=$?
set -e
cat "$TMP/test.log"
test "$RC" -ne 0
printf 'MAG_L1_RESULT=UNAVAILABLE_READ_REUSES_ZERO_OR_STALE_AXIS\n'
