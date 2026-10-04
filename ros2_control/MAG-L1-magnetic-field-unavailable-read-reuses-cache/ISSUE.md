## Description

`semantic_components::MagneticFieldSensor` publishes finite values when a state interface cannot be read by the nonblocking `get_optional()` call. Before any successful read it uses the cache initializer `{0, 0, 0}`. After a successful sample it updates readable axes but reuses the previous value for each unavailable axis.

This affects `magnetometer_broadcaster`, which publishes the returned vector and stamps it with the current controller update time. For example, after sample `A=(1,2,3)`, changing the interfaces to `B=(10,20,30)` and making only y unavailable produces `(10,2,30)`. That vector was never a complete sensor sample.

The current source comment says stale data may be returned to avoid discontinuity. The issue is narrower than stale data alone: the initial zero is not a previous measurement, and per-axis reuse creates mixed-sample vectors while the final message is presented as a current measurement.

### Expected behavior

An unavailable axis should not be represented as a fabricated or previous finite measurement. `sensor_msgs/msg/MagneticField` already specifies NaN for an axis that is not reported. Skipping the publication or another explicit invalid-data strategy would also preserve the measurement semantics.

### Actual behavior

- If all three reads time out before the first successful sample, the message contains `(0,0,0)`.
- If one read times out after a previous sample, that axis comes from the previous sample while the readable axes come from the current one.
- `get_values_as_message()` still returns `true` in both cases.

The reproducer holds each `StateInterface` mutex deterministically. The `LoanedStateInterface` diagnostics confirm 10 missed calls and a timeout for every unavailable read.

## To reproduce

1. Check out `ros-controls/ros2_control` at current master commit `a0f133d921e043aa2cfec8bd7c61effbb8ac132c`.
2. Apply `test.patch` or run `reproduce.sh` in a matching ROS 2 Rolling source environment.
3. Run `test_magnetic_field_sensor` with filter `MagneticFieldSensorAvailabilityTest.*`.

Both added tests fail on the original source. Across three independent runs, the normal existing test passed 3/3 and the two-case reproducer failed 3/3.

The same behavior was also reproduced through `magnetometer_broadcaster` from `ros2_controllers` 6.9.0: both first-read and partial-read cases failed in 3/3 independent processes. The broadcaster implementation is byte-identical at the current Jazzy head and master.

## System information

- OS: Ubuntu 24.04, x86_64
- Compiler: GCC 13.3.0
- ROS 2 underlay: Jazzy, with matching source packages added for the current-master build
- `ros2_control`: `a0f133d921e043aa2cfec8bd7c61effbb8ac132c`
- `control_msgs`: `e0648e4c00b429fa3239606c501741ebf6cfce30`
- `ros2_controllers` release integration check: 6.9.0, `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- `sensor_msgs`: 5.3.8

## Additional context

`fix.patch` contains a minimal candidate fix that uses quiet NaN when `get_optional()` returns empty, matching the existing `RangeSensor` failure representation. With that change:

- the latest-master focused test passed in 3/3 runs and its CTest target passed;
- all 18 focused `magnetometer_broadcaster` runs passed;
- the broadcaster package regression passed both CTest targets (14 tests, 0 failures).

Open and closed issue/PR searches in both `ros2_control` and `ros2_controllers` found no same-root report. The only direct matches were the original component/broadcaster introduction PRs.
