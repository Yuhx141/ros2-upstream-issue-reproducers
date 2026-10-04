## Description

`force_torque_sensor_broadcaster` continues configuring successfully when `filters::FilterChain::configure()` returns false. An explicitly configured filter that cannot be parsed, loaded, or configured is therefore silently treated like no filter: the controller can activate and publish `~/wrench`, but the requested `~/wrench_filtered` output never exists.

### Expected behavior

If an explicit filter chain fails to configure, the controller's configure transition should return `ERROR`. A missing filter-chain parameter remains a valid empty chain: `FilterChain::configure()` returns true with length zero, and the controller can configure without a filtered publisher path.

### Actual behavior

The reproducer specifies a nonexistent Wrench filter plugin. The underlying API logs a fatal plugin-load failure and returns false, immediately followed by the controller's success log:

```text
Could not load library for filters/ThisWrenchFilterDoesNotExist ...
configure successful
```

The controller configure callback returned success in 3/3 independent processes.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build `force_torque_sensor_broadcaster`, and run `ForceTorqueSensorBroadcasterTest.invalid_filter_configuration_rejects_configure`; or use `reproduce.sh`.
3. Observe that filter configuration reports failure but the controller configure transition succeeds.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` has byte-identical broadcaster implementation.

## Additional context

`FilterChain::configure()` already distinguishes the cases needed here: valid empty configuration returns true with length zero; parse, plugin-load, or plugin-configure failures return false. The included minimal fix returns controller configure `ERROR` only for the false case, then uses the length to detect a valid nonempty chain.

With the fix the target passes 3/3. The complete package regression, including valid empty and valid dummy-filter configurations plus six new normal-path closure tests, passes 44 tests with zero failures.

Open and closed issue/PR searches for Force/Torque filter chain, configure failure, invalid filter, and plugin-load failure found no same-root report. PR #1814 introduced filtering and tests only valid chains.
