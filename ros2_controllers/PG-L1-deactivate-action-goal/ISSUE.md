## Description

`parallel_gripper_controller` does not terminate an active `ParallelGripperCommand` goal when the controller is deactivated. The action server also remains available while the controller is inactive and accepts another goal, although no controller update loop can execute it.

This leaves clients waiting indefinitely for a terminal result and lets an inactive controller report a newly accepted/executing goal.

### Expected behavior

When an active controller with an executing goal transitions to inactive:

1. the in-flight goal should receive a terminal result (`ABORTED` or `CANCELED`); and
2. the inactive controller should make the action server unavailable or reject new goals.

### Actual behavior

- The in-flight goal's result future is still not ready after 1 second (20 action-monitor periods).
- The action server remains discoverable after deactivation and accepts a new valid goal.
- An explicit client cancel while active completes normally, so the timeout is not caused by a client/executor setup failure.

## To reproduce

1. Check out `ros-controls/ros2_controllers` at Jazzy commit `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`.
2. Run `./reproduce.sh /path/to/ros2_controllers` from the reproducer directory.
3. The active-cancel control passes. The deactivation test then fails because the old goal remains non-terminal, and the inactive-goal test fails because the new goal is accepted.

Each obligation was run in three independent processes. The control passed 3/3; both affected behaviors failed 3/3.

## System information

- OS: Ubuntu 24.04, x86_64
- ROS 2 distro: Jazzy
- Install method: source overlay on Jazzy APT underlay
- `ros2_controllers`: `1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`
- Installed package: `ros-jazzy-parallel-gripper-controller 4.42.1-1noble.20260905.073118`
- Compiler: GCC 13.3.0

The implementation header in the installed 4.42.1 package is byte-for-byte identical to the tested source. The relevant `on_deactivate()` implementation is also unchanged in tag `6.9.0` (`78d6508`) and current `master` (`2520ae5`, checked 2026-09-28).

## Additional context

`on_deactivate()` only clears the five hardware-interface references. It does not inspect or terminate `rt_active_goal_`, reset `goal_handle_timer_`, or destroy `action_server_`.

The existing tests only assert that the lifecycle callback succeeds; they do not create an action client or observe a goal across deactivation.

A minimal counterfactual in `fix.patch` aborts and immediately publishes the active goal result before resetting the goal timer and action server. The two failing obligations then pass 3/3, the active cancel and goal preemption controls remain unchanged, and the complete package test result is 18 tests with no failures.

`joint_trajectory_controller::on_deactivate()` already terminates and clears its active goal, which is consistent with the expected lifecycle behavior here.

Searches across open and closed issues/PRs for the component name, action type, deactivation, inactive goal acceptance, and non-terminal goals found no matching report. #1229 concerns command interfaces/effort behavior, while #1851 concerns realtime safety.
