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
git -C "$REPO" diff --quiet -- pid_controller/test/test_pid_controller.hpp pid_controller/test/test_pid_controller_dual_interface.cpp pid_controller/test/pid_controller_params.yaml
git -C "$REPO" apply "$HERE/test.patch"
APPLIED=1
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
colcon --log-base "$TMP/log" build \
  --base-paths "$REPO/pid_controller" \
  --build-base "$TMP/build" --install-base "$TMP/install" \
  --packages-select pid_controller --symlink-install
source "$TMP/install/setup.bash"
BIN="$TMP/build/pid_controller/test_pid_controller_dual_interface"
PARAMS="$REPO/pid_controller/test/pid_controller_params.yaml"
"$BIN" --ros-args --params-file "$PARAMS" -- \
  --gtest_filter=PidControllerDualInterfaceTest.external_measured_state_configured_order_control
set +e
"$BIN" --ros-args --params-file "$PARAMS" -- \
  --gtest_filter=PidControllerDualInterfaceTest.external_measured_state_permutation_preserves_mapping
RC=$?
set -e
test "$RC" -ne 0
printf 'Reproduced: the valid permutation failed while configured order passed.\n'
