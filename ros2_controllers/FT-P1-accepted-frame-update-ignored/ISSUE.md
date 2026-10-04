## Description

`force_torque_sensor_broadcaster` accepts a runtime update to its `frame_id` parameter and the node reads back the new value, but subsequent raw and filtered Wrench messages keep the frame captured during `on_configure()`.

The update loop refreshes the generated parameter struct on every cycle (`try_get_params(params_)`) and uses refreshed offsets/multipliers, while both message headers receive `frame_id` only once during configure. This leaves the public parameter state and published data semantics inconsistent.

### Expected behavior

Either reject `frame_id` changes as read-only, or apply an accepted value to subsequent `WrenchStamped.header.frame_id` fields. With the current dynamic parameter-refresh design, applying the accepted value is the minimal behavior-preserving fix.

### Actual behavior

The reproducer configures frame `fts_sensor_frame`, publishes a positive-control message, then sets `frame_id=updated_fts_sensor_frame` while active. The parameter result is successful and `get_parameter()` returns the updated value, but the next message still contains `fts_sensor_frame`. Force and torque data remain correct.

This reproduced in 3/3 independent processes.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build the package, and run `ForceTorqueSensorBroadcasterTest.accepted_frame_parameter_update_changes_message_frame`; or use `reproduce.sh`.
3. The parameter acceptance/readback assertions and wrench data assertions pass; only the updated frame assertion fails.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` has byte-identical broadcaster implementation.

## Additional context

The parameter schema does not mark `frame_id` read-only. The included fix refreshes the raw frame after parameter refresh and the filtered frame after a successful filter update. The target passes 3/3 with the fix; the combined package regression passes 44 tests with zero failures.

Open and closed issue/PR searches for Force/Torque frame, `frame_id` parameter update, and dynamic parameters found no same-root report. Issue #1999 concerns relative namespacing in `diff_drive_controller` and is unrelated.
