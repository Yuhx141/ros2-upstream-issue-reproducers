## Description

All `range_sensor_broadcaster` parameters are writable through the ROS parameter API, but the implementation reads them only in `on_init()` / `on_configure()` and caches the interface binding and message metadata. Runtime updates succeed and read back as the new values while the requested state interface and subsequent `Range` messages keep the old values.

The reproduced fields are `sensor_name`, `frame_id`, `radiation_type`, `field_of_view`, `min_range`, and `max_range`. `variance` has a separate version-guard defect and is excluded from this report's behavioral assertion.

### Expected behavior

Either reject these configuration changes as read-only, or apply every accepted value to the subsequent interface binding/output. An accepted public parameter value and unchanged component behavior should not coexist.

### Actual behavior

Starting with interface `range_sensor/range`, frame `range_sensor_frame`, radiation `1`, FOV `0.1`, and range limits `[0.1, 7.0]`, all updates are accepted. The interface remains `range_sensor/range` and messages retain every old metadata value. This reproduced in 3/3 independent processes.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build the package, and run `RangeSensorBroadcasterTest.AcceptedParameterUpdatesAreAppliedOrRejected`; or use `reproduce.sh`.
3. The test accepts either coherent implementation choice. Current code takes the “accepted” branch but then fails the updated interface and output assertions.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` and Jazzy head `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635` have byte-identical implementation and parameter schema.

## Additional context

The included minimal fix marks the seven configure-time parameters read-only. The focused test then passes 3/3 and both package CTest targets pass. Applying dynamic metadata is another valid choice, but `sensor_name` still requires a lifecycle-safe interface rebind.

This is the same public-state/output-state inconsistency family as #2657 in `force_torque_sensor_broadcaster`, but it is a separate component implementation and requires a separate code change. Open and closed Range-specific searches found no report.
