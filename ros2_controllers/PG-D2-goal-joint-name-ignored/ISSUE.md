## Description

`parallel_gripper_controller` accepts a `ParallelGripperCommand` whose non-empty `command.name` requests a different joint, then applies its values to the controller's configured joint.

`ParallelGripperCommand` uses `sensor_msgs/JointState` for the command, and its definition describes `name` as the joint name(s) requested by the command. Empty names can remain supported as an omitted optional mapping, but a non-empty name should not silently route to another joint.

### Expected behavior

A goal with `command.name = ["other_joint"]` should be rejected by a controller configured for `joint1`. A goal with an empty name or `name = ["joint1"]` should remain accepted.

### Actual behavior

The goal is accepted. After one normal controller update, the hardware command for `joint1/position` becomes the requested `2.0`:

```text
MISMATCH_ACCEPTED=1 REQUESTED=other_joint CONTROLLED=joint1 COMMAND=2
```

This reproduced in 3/3 independent processes. A matching-name goal was accepted and canceled first as a positive action-path control.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Apply `test.patch`, build `parallel_gripper_controller`, and run `GripperControllerTest.RejectsGoalForDifferentJoint`; or run the supplied `reproduce.sh`.
3. The test fails because the mismatched goal handle is non-null and prints the command mapping above.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`
- Installed package: `ros-jazzy-parallel-gripper-controller 4.42.1-1noble.20260905.073118`
- Compiler: GCC 13.3.0

Current master `2520ae5` (checked 2026-09-28) still validates only `position.size()` and never reads `command.name`.

## Additional context

The minimal fix accepts empty names for compatibility, while requiring a non-empty name to contain exactly the configured joint. The target test then passes 3/3, and the 19 non-conflicting component tests pass.

Open and closed issue/PR searches for `ParallelGripperCommand command.name`, wrong joint name, and `JointState name` found no matching report.
