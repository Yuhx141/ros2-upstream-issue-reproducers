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
git -C "$REPO" diff --quiet -- joint_state_broadcaster
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [ -n "${EXTRA_CMAKE_PREFIX_PATH:-}" ]; then
  export CMAKE_PREFIX_PATH="$EXTRA_CMAKE_PREFIX_PATH:${CMAKE_PREFIX_PATH:-}"
fi
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
colcon --log-base "$TMP/log" build   --base-paths "$REPO/joint_state_broadcaster"   --build-base "$TMP/build" --install-base "$TMP/install"   --packages-select joint_state_broadcaster   --allow-overriding joint_state_broadcaster   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source "$TMP/install/setup.bash"
BIN="$TMP/build/joint_state_broadcaster/test_joint_state_broadcaster"
set +e
"$BIN" --gtest_filter='JointStateBroadcasterTest.use_urdf_filter_false_without_urdf_contract_probe' >"$TMP/test.log" 2>&1
RC=$?
set -e
cat "$TMP/test.log"
test "$RC" -ne 0
printf 'JSB_U1_RESULT=FALSE_STILL_REQUIRES_URDF\n'
