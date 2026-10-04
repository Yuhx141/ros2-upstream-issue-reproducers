## Description

`joint_state_broadcaster` silently omits an `extra_joints` entry from `JointState` when a state interface with the same joint name exists but does not map to position, velocity, or effort.

The extra-joint branch checks whether `name_if_value_mapping_` contains the joint name. That map includes custom-only resources used outside `JointState`, so their presence suppresses the documented extra zero-valued movement fields even though the joint is absent from the outgoing `JointState.name` list.

### Expected behavior

If a joint is not already present in `JointState`, listing it in `extra_joints` should append it with position, velocity, and effort set to 0. A custom interface such as `measured_position` or `temperature` should not make it count as already present in `JointState`.

This also matches the ROS 1 behavior that #179 migrated: `addExtraJoints()` checks the outgoing `msg.name`, not the existence of any internal interface with the same resource name.

### Actual behavior

The reproducer supplies only `joint1/measured_position`, requests it explicitly, and also sets `extra_joints=[joint1]`. Activation and update succeed, but `JointState.name` is empty instead of containing `joint1` with three zero fields.

This reproduced 3/3 on both 6.9.0 and Jazzy 4.42.1. The included minimal fix checks `joint_names_` (the outgoing JointState set) and inserts the three zero slots; the probe passes 3/3 and the 6.9 package regression passes 27 tests with zero failures.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build `joint_state_broadcaster`, and run `JointStateBroadcasterTest.extra_joint_with_custom_only_interface_probe`; or use `reproduce.sh`.
3. The first message assertion fails: expected `{ "joint1" }`, actual `{}`.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Also reproduced: Jazzy 4.42.1, commit `aacd842600a09d556b983ac3d53a0983e9ebcbb1`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` has byte-identical component source and tests to 6.9.0.

## Additional context

The ordinary fresh-name `extra_joints` test passes; the blind spot is specifically a name already present only in the general interface map. Open and closed searches for `extra_joints`, custom interfaces, and joint-state omission found no same-root report.
