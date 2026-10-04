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
git -C "$REPO" diff --quiet -- \
  parallel_gripper_controller/test/test_parallel_gripper_controller.hpp \
  parallel_gripper_controller/test/test_parallel_gripper_controller.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
set -u
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/parallel_gripper_controller" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select parallel_gripper_controller --allow-overriding parallel_gripper_controller
set +u
source "$TMP/install/setup.bash"
set -u
BIN="$TMP/build/parallel_gripper_controller/test_parallel_gripper_controller"
PARAMS="$REPO/parallel_gripper_controller/test/gripper_action_controller_params.yaml"
set +e
PG_CONTROLLER_POISON=A5 "$BIN" --ros-args --params-file "$PARAMS" -- \
  --gtest_filter=GripperControllerTest.ImmediateSuccessControl >"$TMP/success.log" 2>&1
SUCCESS_RC=$?
PG_CONTROLLER_POISON=5A "$BIN" --ros-args --params-file "$PARAMS" -- \
  --gtest_filter=GripperControllerTest.StallResultControl >"$TMP/stall.log" 2>&1
STALL_RC=$?
set -e
cat "$TMP/success.log"
cat "$TMP/stall.log"
test "$SUCCESS_RC" -ne 0
test "$STALL_RC" -ne 0
grep -q 'RESULT_EFFORT_BITS=0xa5a5a5a5a5a5a5a5' "$TMP/success.log"
grep -q 'STALL_RESULT_EFFORT_BITS=0x5a5a5a5a5a5a5a5a' "$TMP/stall.log"
printf 'Reproduced: both result branches copied the controlled raw-storage pattern.\n'
