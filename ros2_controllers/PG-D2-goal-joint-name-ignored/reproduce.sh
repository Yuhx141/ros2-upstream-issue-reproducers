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
git -C "$REPO" diff --quiet -- parallel_gripper_controller/test/test_parallel_gripper_controller.cpp
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/parallel_gripper_controller" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select parallel_gripper_controller --allow-overriding parallel_gripper_controller \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source "$TMP/install/setup.bash"
BIN="$TMP/build/parallel_gripper_controller/test_parallel_gripper_controller"
PARAMS="$REPO/parallel_gripper_controller/test/gripper_action_controller_params.yaml"
set +e
"$BIN" --ros-args --params-file "$PARAMS" -- --gtest_filter=GripperControllerTest.RejectsGoalForDifferentJoint >"$TMP/test.log" 2>&1
RC=$?
set -e
cat "$TMP/test.log"
test "$RC" -ne 0
grep -q 'MISMATCH_ACCEPTED=1 REQUESTED=other_joint CONTROLLED=joint1 COMMAND=2' "$TMP/test.log"
printf 'Reproduced: the mismatched joint name was accepted and applied to joint1\n'
