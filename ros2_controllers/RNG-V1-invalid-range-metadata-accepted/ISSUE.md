## Description

`range_sensor_broadcaster` validates only that `sensor_name` and `frame_id` are nonempty. It accepts configurations that cannot describe a valid `sensor_msgs/msg/Range` message:

- `radiation_type=-1` (later cast to uint8) or `radiation_type=2`, although the message defines only ULTRASOUND=0 and INFRARED=1;
- a negative `field_of_view`;
- `min_range > max_range`;
- a negative variance.

Each configuration initializes and configures successfully in the current implementation.

### Expected behavior

Reject configuration values that violate the output message's enum, geometric interval, and variance semantics before activating the broadcaster.

### Actual behavior

All five invalid classes configure successfully. The focused tests reproduced each class in 3/3 independent processes. For example, a negative radiation enum is narrowed to an unrelated uint8 value, and inverted range limits are copied directly to the outgoing message metadata.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build the package, and run `RangeSensorBroadcasterTest.RejectsInvalid*`; or use `reproduce.sh`.
3. All five tests fail because `configure_succeeds()` returns true (or initialization fails after applying the fix).

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` and Jazzy head `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635` have byte-identical implementation and parameter schema.

## Additional context

The included fix uses existing `generate_parameter_library` validators for the single-field constraints and one configure-time check for `min_range <= max_range`. The five focused tests pass 3/3 and both package CTest targets pass.

Finite range samples outside `[min_range, max_range]` are intentionally not part of this report: REP 117 leaves those samples for consumers to discard, and the broadcaster's current tests explicitly require pass-through. Open and closed issue/PR searches found no same-root validation report.
