## Description

The parameter schema accepts `goal_tolerance = 0.0`, but the controller tests success with a strict `<` comparison. An action goal whose position is bit-for-bit equal to the measured position therefore cannot reach the success branch.

### Expected behavior

The parameter contract should be internally consistent: either reject zero at configuration time, or let an exact zero-error goal succeed.

### Actual behavior

Configuration and activation succeed with `goal_tolerance = 0.0`. A goal and measured position both set from the same `double` value produce error 0, `update()` returns OK, but the action remains executing:

```text
ZERO_TOLERANCE_RESULT=TIMEOUT ERROR=0 TOLERANCE=0
```

This reproduced in 3/3 independent processes. The same action path succeeds with the default positive tolerance.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Apply `test.patch`, build `parallel_gripper_controller`, and run `GripperControllerTest.ZeroToleranceAcceptsExactGoal`; or run the supplied `reproduce.sh`.
3. The result times out despite exact zero position error.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`
- Installed package: `ros-jazzy-parallel-gripper-controller 4.42.1-1noble.20260905.073118`
- Compiler: GCC 13.3.0

Current master `2520ae5` (checked 2026-09-28) still has `gt_eq: [0.0]` in the schema and `fabs(error_position) < params_.goal_tolerance` in the success branch.

## Additional context

The included behavior fix changes the comparison to `<=`; an equally valid contract fix would reject zero in the parameter schema. The behavior fix makes the target pass 3/3, and the 19 non-conflicting component tests pass.

Open and closed issue/PR searches for parallel-gripper zero/exact goal tolerance found no matching report. The deprecated classic gripper controller has the same strict comparison, but that code similarity is not treated as evidence that zero is intentionally accepted and unreachable.
