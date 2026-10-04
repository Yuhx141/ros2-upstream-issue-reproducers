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
  parallel_gripper_controller/test/test_parallel_gripper_controller.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
set -u
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/parallel_gripper_controller" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select parallel_gripper_controller --allow-overriding parallel_gripper_controller
source "$TMP/install/setup.bash"
BIN="$TMP/build/parallel_gripper_controller/test_parallel_gripper_controller"
PARAMS="$REPO/parallel_gripper_controller/test/gripper_action_controller_params.yaml"
run() {
  "$BIN" --ros-args --params-file "$PARAMS" -- \
    "--gtest_filter=GripperControllerTest.$1"
}
run ActionCancelControl
set +e
run DeactivateTerminatesExecutingGoal
DEACTIVATE_RC=$?
run InactiveRejectsNewGoal
INACTIVE_RC=$?
set -e
test "$DEACTIVATE_RC" -ne 0
test "$INACTIVE_RC" -ne 0
printf 'Reproduced: active cancel passed; both deactivate obligations failed.\n'
