## Description

`semantic_components::GPSSensor::get_status()` and `get_service()` return the maximum value of their integer type when `LoanedStateInterface::get_optional()` cannot read an interface. `get_values_as_message()` writes those values directly into `sensor_msgs::msg::NavSatFix`.

The resulting status `127` is outside the defined `NavSatStatus` range (`STATUS_UNKNOWN=-2` through `STATUS_GBAS_FIX=2`), and service `65535` sets bits outside the four defined service flags. A consumer therefore receives invalid enum/bitmask values precisely when the reading is unavailable.

### Expected behavior

An unavailable status or service read should produce the message-defined unknown values:

```text
status  = sensor_msgs::msg::NavSatStatus::STATUS_UNKNOWN  (-2)
service = sensor_msgs::msg::NavSatStatus::SERVICE_UNKNOWN (0)
```

### Actual behavior

The reproducer first verifies a normal sample (`status=1`, `service=13`), then holds the two underlying interface mutexes so the real loaned-interface reads fail while latitude, longitude, and altitude remain readable:

```text
message.status.status  = 127
message.status.service = 65535
```

The two invalid values reproduced in 3/3 independent processes. The three coordinate fields remained correct.

## To reproduce

1. Check out `ros-controls/ros2_control` at `a0f133d921e043aa2cfec8bd7c61effbb8ac132c`.
2. Apply `test.patch`, build `controller_interface`, and run `GPSSensorTest.failed_status_and_service_reads_should_use_unknown_enums`; or run the supplied `reproduce.sh` in a current Rolling source environment.
3. The assertions report status `127` instead of `-2`, and service `65535` instead of `0`.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay used for the dynamic check: Jazzy
- `ros2_control`: master `a0f133d921e043aa2cfec8bd7c61effbb8ac132c` (checked 2026-09-28)
- Compiler: GCC 13.3.0

The source-under-test header was unmodified current master. Because the local Jazzy underlay has the older two-argument `LoanedStateInterface` constructor, the local fixture used only constructor-call compatibility edits; the supplied `test.patch` is the native one-argument master version and applies cleanly to the commit above.

## Additional context

The included two-line `fix.patch` returns `STATUS_UNKNOWN` and `SERVICE_UNKNOWN`. With that change, the target test passes 3/3 and the complete `test_gps_sensor` binary passes all 7 tests.

Open and closed issue searches for GPS sensor, `NavSatStatus`, `get_optional` failure, and state-interface read failure found no report with this root cause.
