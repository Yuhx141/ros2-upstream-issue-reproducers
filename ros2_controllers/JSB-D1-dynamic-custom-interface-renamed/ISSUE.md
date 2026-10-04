## Description

On the maintained Jazzy branch, `map_interface_to_joint_state` also renames the source interface in `DynamicJointState`, although the parameter and feature were introduced to map custom measurements into standard fields of `sensor_msgs/JointState`.

The implementation rewrites the interface name before inserting it into the single cache used by both messages. `init_dynamic_joint_state_msg()` therefore sees only the mapped target name. If a real standard interface and a mapped custom source coexist, they can also collapse into the same cache entry.

### Expected behavior

With `joint1/measured_position=3.5` mapped to `JointState.position`, the two outputs should be:

- `JointState`: `position=3.5`
- `DynamicJointState`: interface `measured_position=3.5`

Jazzy's user documentation, added by #1865/#1871, says the dynamic topic publishes all available interfaces including custom ones. The original mapping PR #217 scopes the mapping to fields in `JointState`.

### Actual behavior

`JointState.position` is correct, but the dynamic message reports interface name `position`; the original `measured_position` identity is lost. This reproduced in 3/3 independent processes on Jazzy 4.42.1.

The included causal patch keeps a raw interface cache for `DynamicJointState` and a mapped cache for `JointState`. The probe then passes 3/3. The valid package regression passes 29 tests with zero failures after updating the two existing mapping assertions that encoded the old implementation and removing one pre-existing out-of-bounds assertion in the reactivation test.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `4.42.1` (`aacd842600a09d556b983ac3d53a0983e9ebcbb1`).
2. Apply `test.patch`, build `joint_state_broadcaster`, and run `JointStateBroadcasterTest.dynamic_custom_mapping_preserves_source_interface_probe`; or use `reproduce.sh`.
3. The JointState value assertion passes, while the dynamic interface-name assertion fails: expected `measured_position`, actual `position`.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 4.42.1, commit `aacd842600a09d556b983ac3d53a0983e9ebcbb1`
- Jazzy head checked: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`, byte-identical component source/tests
- Compiler: GCC 13.3.0

## Additional context

This is version-scoped. PR #2187 removed `DynamicJointState` from 6.9/master and was explicitly not backported, so the report applies to the supported Jazzy implementation rather than current master.

Open and closed searches for custom mapping, `map_interface_to_joint_state`, and `dynamic_joint_states` found no same-root report.
