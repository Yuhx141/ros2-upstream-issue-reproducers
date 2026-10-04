## Description

In `joint_state_broadcaster` 6.9.0, setting `use_urdf_to_filter=false` does not disable the URDF-dependent selection path when `joints` or `interfaces` is empty. The controller still requires a valid robot description and requests only interfaces belonging to URDF joints.

`use_urdf_joint_interfaces()` currently depends only on whether either selector is empty. It is used both to choose `INDIVIDUAL_BEST_EFFORT` over `ALL` and to make a missing robot description a configure error, without considering `use_urdf_to_filter`.

### Expected behavior

With empty selectors and `use_urdf_to_filter=false`, configuration should not require a robot description. The state interface configuration should be `ALL`, allowing the broadcaster to publish position/velocity/effort interfaces from hardware joints that are not in the URDF.

This follows the parameter contract: when filtering is false, the broadcaster publishes data from any interface of type position, velocity, or effort.

### Actual behavior

With an empty robot description, configuration returns `ERROR`. With a valid robot description, the returned configuration is `INDIVIDUAL_BEST_EFFORT` and contains only names generated from `model_.joints_`; a hardware-only joint cannot be assigned by ControllerManager.

Both observations reproduced in 3/3 independent processes. The included minimal fix makes both probes pass 3/3; the package-level regression then passes 27 tests with zero failures.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at tag `6.9.0` (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`).
2. Apply `test.patch`, build `joint_state_broadcaster`, and run `JointStateBroadcasterTest.use_urdf_filter_false_without_urdf_contract_probe`; or use `reproduce.sh`.
3. The assertion that configure succeeds fails because the controller treats the missing URDF as fatal despite filtering being disabled.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 underlay: Jazzy
- `ros2_controllers`: tag 6.9.0, commit `78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`
- Compiler: GCC 13.3.0

Current master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` has byte-identical component source and tests.

## Additional context

The behavior was introduced by #2187, which switched empty-selector mode from `ALL` to URDF-based `INDIVIDUAL_BEST_EFFORT`. That PR was intentionally not backported. The minimal fix separates “all selectors are implicit” from “implicit selectors should be derived from the URDF”: disabled filtering uses `ALL`, enabled filtering keeps the current best-effort URDF path.

Open and closed searches for `joint_state_broadcaster`, `use_urdf_to_filter`, robot description, and `INDIVIDUAL_BEST_EFFORT` found no same-root report.
