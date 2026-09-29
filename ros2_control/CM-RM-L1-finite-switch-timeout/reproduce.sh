#!/usr/bin/env bash
set -euo pipefail
ws=${1:?pass a ros2_control source workspace}
cp test.patch "$ws/test.patch"
cd "$ws"
git apply --check test.patch
git apply test.patch
echo 'Patch applied. Build controller_manager tests, then run:'
echo './build/controller_manager/test_controller_manager_srvs --gtest_filter=TestControllerManagerSrvs.switch_controller_controllers_taking_long_time_to_activate'
