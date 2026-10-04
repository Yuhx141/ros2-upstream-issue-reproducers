## Description

`parallel_gripper_controller` publishes `ParallelGripperCommand` success and stall results with `state.effort[0]` copied from `computed_command_`, but that member is never initialized or assigned.

Consequently, a normal terminal action result can contain arbitrary effort data from the controller object's previous storage contents.

### Expected behavior

Every non-empty result field should have a defined source. Because this controller does not claim an effort state interface, `state.effort` may be left empty as permitted by `JointState`; alternatively, it must be populated from an explicitly defined source.

### Actual behavior

With identical goals, parameters, measured state, and update sequence, only changing the byte used to fill the controller's raw storage before construction changes the result exactly as follows:

| Raw storage byte | Result effort bits | Printed value |
|---|---|---|
| `00` | `0x0000000000000000` | `0` |
| `A5` | `0xa5a5a5a5a5a5a5a5` | `-2.49834e-127` |
| `5A` | `0x5a5a5a5a5a5a5a5a` | `1.78389e+127` |

The same behavior occurs in both the reached-goal success result and the stalled/aborted result.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Run `./reproduce.sh /path/to/ros2_controllers` from the reproducer directory.
3. The script controls the pre-construction storage pattern for the test subclass, sends normal action goals, and prints the leaked effort bit patterns.

Success and stall were each run with `00`, `A5`, and `5A` in three independent processes: 18/18 results copied the selected raw-storage pattern while all action codes, flags, and positions remained correct.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`
- Installed package: `ros-jazzy-parallel-gripper-controller 4.42.1-1noble.20260905.073118`
- Compiler: GCC 13.3.0

The tested implementation is identical to the installed 4.42.1 header. Tag `6.9.0` (`78d6508`) and current `master` (`2520ae5`, checked 2026-09-28) still contain the same uninitialized member and both result reads.

## Additional context

The only component references are the declaration and two reads:

```cpp
double computed_command_;
```

```cpp
pre_alloc_result_->state.effort[0] = computed_command_;
```

There is no assignment to `computed_command_` in `parallel_gripper_controller`.

The current fixture's implicit value initialization happens to make ordinary runs print zero, which hid the problem. The reproducer gives the test subclass a user-provided constructor and controls only its raw allocation bytes; it does not access or write `computed_command_`.

The minimal counterfactual in `fix.patch` leaves the optional effort array empty and removes the unused member. Success and stall then pass 18/18 across all storage patterns, and 15 upstream/non-conflicting regression tests pass.

Searches across open and closed issues/PRs for `computed_command_`, parallel-gripper result effort, uninitialized effort, and `ParallelGripperCommand state effort` found no matching report. #1229 discusses effort command forwarding and limits, not result data sourced from uninitialized storage.
