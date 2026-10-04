## Description

`imu_sensor_broadcaster` exports ten chainable state interfaces as owning `StateInterface` objects, initialized from `state_message_` at export time. `update_and_write_commands()` subsequently updates and publishes `state_message_`, but never writes the new values to those exported objects. A downstream chained controller that retains the handles returned by `export_state_interfaces()` therefore reads the initial defaults forever while the `~/imu` topic publishes current data.

### Expected behavior

After each controller update, the ten exported state interfaces should contain the same transformed orientation, angular velocity, and linear acceleration values that the broadcaster publishes.

### Actual behavior

The reproducer exports once after configuration, retains those handles, then applies two distinct ten-field IMU samples. Both topic messages contain every expected value, but all ten retained exported handles remain at their export-time defaults during both updates:

```text
IMU_EXPORTED_STATE_RESULT=STALE PUBLISHED_FAILURES=0 EXPORTED_FAILURES=20
```

This reproduced in 3/3 independent processes.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build `imu_sensor_broadcaster`, and run `IMUSensorBroadcasterTest.ExportedStateInterfacesTrackUpdates`; or run the supplied `reproduce.sh`.
3. The test makes two updates. Its topic assertions pass, while the same exported handles fail all 20 value assertions.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: tag `6.9.0`, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- `ros-jazzy-controller-interface`: `4.48.0-1noble.20260905.072534`
- `ros-jazzy-hardware-interface`: `4.48.0-1noble.20260905.070541`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` (checked 2026-09-28) has byte-identical `imu_sensor_broadcaster.cpp`, so the behavior is still present there.

## Additional context

The included `fix.patch` writes the transformed `state_message_` fields to the base class's existing `ordered_exported_state_interfaces_` handles after each update. With that change, the target test passes 3/3 and the complete package suite passes (14 tests, zero failures).

The generic chainable-controller test already updates `ordered_exported_state_interfaces_` during each update. The IMU broadcaster's existing test exports only after publishing and therefore constructs fresh handles from the latest message, which does not cover retained-handle behavior.

Open and closed issue searches for IMU broadcaster exported/chained/stale state found no matching report. Issues #2441 and #1522 appeared only in broad searches and concern unrelated plugin loading and missing hardware interfaces.
