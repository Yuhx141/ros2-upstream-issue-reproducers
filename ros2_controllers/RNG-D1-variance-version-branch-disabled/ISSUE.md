## Description

`range_sensor_broadcaster` declares a `variance` parameter, but on current Jazzy, release 6.9.0, and master builds the configured value is never copied into `sensor_msgs/msg/Range`. A configuration with `variance: 1.0` publishes `variance: 0.0`.

The assignment and all existing variance assertions are guarded by `#if SENSOR_MSGS_VERSION_MAJOR >= 5`. `range_sensor_broadcaster/CMakeLists.txt` never defines `SENSOR_MSGS_VERSION_MAJOR`, so the preprocessor treats it as zero and removes both the product behavior and its tests. `sensor_msgs` 5.3.8 is installed and its `Range` message contains the field.

### Expected behavior

The published `Range.variance` should equal the configured variance, narrowed to the message's `float32` type. A value of zero should remain the documented “unknown variance” value.

### Actual behavior

With `variance=1.0`, the published value is always `0.0`. The focused black-box test reproduced this in 3/3 independent processes.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build `range_sensor_broadcaster`, and run `RangeSensorBroadcasterTest.ConfiguredVarianceIsPublished`; or use `reproduce.sh`.
3. The test fails with actual `msg.variance == 0` and expected `1`.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy; `sensor_msgs` 5.3.8
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` and Jazzy head `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635` have byte-identical broadcaster source and CMake files, including the missing definition.

## Additional context

The included minimal fix defines `SENSOR_MSGS_VERSION_MAJOR` from CMake's `sensor_msgs_VERSION_MAJOR` and uses the same explicit `float` cast as the other numeric fields. Enabling the branch also exposed one existing test's implicit double-to-float conversion under the package's `-Werror=float-conversion`; the fix updates that assertion.

The focused test passes 3/3 with the fix, and both package CTest targets pass. Open and closed issue/PR searches found no same-root report. PR #1071 introduced the compatibility guard but did not add the corresponding CMake definition.
