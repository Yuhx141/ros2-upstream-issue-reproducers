# Upstream duplicate check

Checked on 2026-09-28 against issues and pull requests in
`ros-controls/ros2_controllers`.

- RC-B1 queries: `BatteryState percentage`, `battery 0..1`, `battery 0 100`.
- RC-P1 queries: `pid_controller MultiDOFCommand`, `pid_controller reorder`,
  `dof_names values`.
- RC-P2/RC-P3 queries: `pid_controller measured_state dof_names`,
  `external measured states`, and the exact measured-state sorting TODO.

- PG-L1 queries: `parallel_gripper deactivate`, `gripper action inactive goal`, `goal remains executing deactivate`, `parallel_gripper_controller`, and `ParallelGripperCommand`. Related #1229 and #1851 do not cover lifecycle goal cleanup.

No issue or PR with the same root cause was found. Related PID feature or CI issues were not treated as duplicates. This records the searched surface and does not claim that unavailable discussions cannot exist.

- GPS-T1 queries: `gps timestamp`, `gps broadcaster`, `timestamp update time`, and `node now`. Issues #289/#290 establish the rule for other components but do not cover the later GPS broadcaster. No same GPS root was found.

- FT-E1 matches the already submitted IMU-E1 retained-handle root and was added to #2654 rather than filed separately. FT-F1/FT-P1 queries: `force torque filter chain`, `filter configure failure`, `invalid filter`, `force torque frame`, `frame_id parameter`, and `parameter update`. No same root was found; #1999 concerns diff_drive namespacing.

- JSB-U1 queries: `joint_state_broadcaster use_urdf_to_filter`, `false`, `robot description`, and `INDIVIDUAL_BEST_EFFORT`. No same root was found.
- JSB-E1 queries: `extra_joints`, `extra joints`, custom interface, and omission. No same root was found. ROS 1 source confirms deduplication against outgoing `JointState.name`.
- JSB-D1 queries: `dynamic_joint_states`, custom mapping, `map_interface_to_joint_state`, and renamed interface. No same root was found; the issue is explicitly scoped to Jazzy because #2187 removed the topic from 6.9/master.
- JSB-O1 matches closed #159 and #1572, so current Jazzy/6.9 evidence was posted to #159 instead of opening a duplicate.


- RNG-D1 queries: `range sensor broadcaster`, `variance`, `SENSOR_MSGS_VERSION_MAJOR`, and the component history. No same-root report was found. PR #1071 introduced the guard but omitted the CMake definition.
- RNG-P1 queries: `range_sensor_broadcaster parameter update`, `dynamic parameter`, and the component name. No Range report was found. It is linked to FT-P1/#2657 as the same behavior family with a separate implementation and fix.
- RNG-V1 queries: `radiation_type`, `min_range`, `max_range`, `validation`, and the component name. No same-root issue or PR was found. Finite sample pass-through was excluded under REP 117 rather than reported.
