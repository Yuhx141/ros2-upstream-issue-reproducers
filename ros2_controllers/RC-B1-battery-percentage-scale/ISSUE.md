## Description

`battery_state_broadcaster` publishes `sensor_msgs/msg/BatteryState.percentage` on a 0–100 scale, while the message contract requires a 0–1 range.

Both supported paths are affected:

- A configured `battery_percentage` state interface is documented as 0–100 and is copied directly into the message.
- When percentage is derived from `minimum_voltage` and `maximum_voltage`, the formula explicitly multiplies by 100.

The aggregate battery percentage is computed from the same raw percentages and therefore has the same scale error.

### Expected behavior

- A direct state value of `66` percent should publish `0.66`.
- A voltage of `5 V` with a configured range of `0–10 V` should publish `0.5`.
- A full battery should publish `1.0`.

### Actual behavior

The same cases publish `66`, `50`, and `100`. With one direct battery at 66% and one derived battery at 50%, the aggregate publishes `58` instead of `0.58`.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Run `./reproduce.sh /path/to/ros2_controllers` from this directory. Set `ROS_SETUP` if the ROS installation is not `/opt/ros/jazzy/setup.bash`.
3. The zero-value control passes. The midpoint/direct/aggregate and full-charge contract tests fail with the values above.

The attached `test.patch` contains the independent-oracle tests. The issue reproduced in three independent processes for every target case.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635` (current Jazzy head when checked on 2026-09-28)
- Installed released package used for comparison: `ros-jazzy-battery-state-broadcaster 4.42.1`
- Compiler: GCC 13.3.0

## Additional context

- Message contract: `sensor_msgs/msg/BatteryState.percentage` says “Charge percentage on 0 to 1 range.”
- Relevant source: `battery_state_broadcaster/src/battery_state_broadcaster.cpp`, direct percentage assignment and voltage-derived percentage calculation.
- The existing upstream tests assert values such as 50, 66, 58, and 100, so they encode the implementation scale instead of the message contract.
- A minimal counterfactual (divide the direct 0–100 interface by 100 and remove the derived-path multiplication by 100) makes all three contract tests pass. See `fix.patch`.
- The same code remains present on master commit `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` when checked.
- Searches for `BatteryState percentage`, `battery 0..1`, and `battery 0 100` found no matching issue or PR on 2026-09-28.
