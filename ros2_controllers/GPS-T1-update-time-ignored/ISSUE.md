## Description

`GPSSensorBroadcaster::update()` ignores its `time` argument and stamps each `sensor_msgs::msg::NavSatFix` with `get_node()->now()`. This can put GPS samples on a different time base from the controller-manager cycle and from sibling broadcasters that use the supplied update time.

### Expected behavior

The published `NavSatFix.header.stamp` should equal the `rclcpp::Time` passed to `update()`. This follows the controller update contract and the rule previously recorded for other controllers in #289 and #290.

### Actual behavior

The reproducer calls `update()` with `123 s + 456 ns`. All frame, status, service, latitude, longitude, and altitude assertions pass, but the published stamp uses the node clock instead:

```text
expected sec/nanosec: 123 / 456
actual sec/nanosec:   current node-clock time
```

Both timestamp fields failed in 3/3 independent processes, with zero data-field failures.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build `gps_sensor_broadcaster`, and run `GPSSensorBroadcasterTest.update_time_should_stamp_published_message`; or run the supplied `reproduce.sh`.
3. The data assertions pass and the two timestamp assertions fail.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: tag `6.9.0`, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` (checked 2026-09-28) has byte-identical GPS broadcaster implementation, so the behavior is still present there.

## Additional context

The included two-line `fix.patch` names the existing argument and assigns it to `header.stamp`. With that change, the target test passes 3/3 and the complete package suite passes (14 tests, zero failures), including reverse interface order, all five legal status values, a combined service bitmask, full static covariance, and deactivate/rebind/reactivate coverage.

Issues #289 and #290 establish this timing rule for joint-state and joint-trajectory components; neither covers `gps_sensor_broadcaster`, which was added later. Broader open and closed searches for GPS timestamp, broadcaster update time, and `node now` found no GPS report with this root cause.
