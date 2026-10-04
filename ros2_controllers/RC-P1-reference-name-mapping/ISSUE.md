## Description

`pid_controller` does not preserve the association between `MultiDOFCommand.dof_names`, `values`, and `values_dot` on `~/reference`.

With two configured DOFs, a valid message containing the same names in reverse order assigns both values and derivatives to the wrong DOFs. Messages containing an unknown or duplicate name are also accepted by length and silently assign that element to another configured DOF.

### Expected behavior

For configured order `[joint1, joint2]`, this message:

```text
dof_names:  [joint2, joint1]
values:     [16, 15]
values_dot: [7, 6]
```

should produce the configured-order reference `[15, 16, 6, 7]`.

### Actual behavior

It produces `[16, 15, 7, 6]`. An input `[joint1, unknown]` or `[joint1, joint1]` with sentinel `999` produces `[15, 999, 6, 999]` instead of rejecting the complete message and retaining the previous reference.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Run `./reproduce.sh /path/to/ros2_controllers` from this directory.
3. The configured-order control passes; the permutation test fails with the values above. The attached patch also contains unknown-name and duplicate-name isolation tests.

All four obligations were run in three independent processes; the control passed 3/3 and each target test failed 3/3.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`
- Installed released package used for comparison: `ros-jazzy-pid-controller 4.42.1`
- Compiler: GCC 13.3.0

## Additional context

The callback aliases the incoming shared pointer:

```cpp
auto ref_msg = msg;
reset_controller_reference_msg(*msg, reference_and_state_dof_names_);
```

Resetting `*msg` therefore also replaces the names seen through `ref_msg`; the subsequent search can only observe configured order. The callback also compares an iterator from `ref_msg->dof_names` with `msg->dof_names.end()`.

A minimal counterfactual deep-copies the input and compares with the matching container end iterator. The control, permutation, unknown-name, and duplicate-name tests then pass 4/4. See `fix.patch`.

The same callback remains present on master commit `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0`. Searches for `pid_controller MultiDOFCommand`, `reorder`, and `dof_names values` found no matching issue or PR on 2026-09-28.
