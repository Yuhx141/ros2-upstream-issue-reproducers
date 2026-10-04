## Description

When `use_external_measured_states=true`, `pid_controller` stores `~/measured_state` in input array order and later consumes it in configured DOF order. It does not validate or use `MultiDOFCommand.dof_names` beyond checking the array length.

This swaps feedback between DOFs when a valid message uses a different name order. Unknown or duplicate names of the expected cardinality are also accepted and silently mapped to configured DOFs by position.

### Expected behavior

With configured order `[joint1, joint2]`, this valid state:

```text
dof_names:  [joint2, joint1]
values:     [11, 10]
values_dot: [6, 5]
```

should result in configured-order feedback `[10, 11, 5, 6]`.

### Actual behavior

The feedback is `[11, 10, 6, 5]`. After a valid baseline, `[joint1, unknown]` and `[joint1, joint1]` inputs containing sentinel `999` replace the state with `[20, 999, 8, 999]` instead of being rejected atomically.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Run `./reproduce.sh /path/to/ros2_controllers` from this directory.
3. The configured-order control passes and the valid permutation fails. `test.patch` also includes unknown, duplicate, and field-length controls.

The seven non-crash obligations were run in three independent processes: the configured-order and three length-validation controls passed 3/3; permutation, unknown-name, and duplicate-name obligations failed 3/3.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`
- Installed released package used for comparison: `ros-jazzy-pid-controller 4.42.1`
- Compiler: GCC 13.3.0

## Additional context

The callback contains an explicit TODO to sort input values by names and then directly stores the message. The update loop subsequently indexes the values in configured DOF order.

The mapping-only counterfactual in `fix.patch` validates that every configured name appears and reorders `values` and `values_dot`. All seven mapping and validation tests then pass. A separate empty-`values_dot` defect remains after this change, so the two root causes were isolated independently.

Master commit `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` still contains the same TODO and direct assignment. Searches for `pid_controller measured_state dof_names`, `external measured states`, and the exact TODO found no matching issue or PR on 2026-09-28.
