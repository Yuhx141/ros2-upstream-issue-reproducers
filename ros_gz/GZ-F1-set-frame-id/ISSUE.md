## Environment

* OS Version: Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem
* Source build at commit 0fa70cb7c15f7500c495190020dd6292188c8e54
* ROS 2 Jazzy
* RMW: rmw_fastrtps_cpp

## Description

* Expected behavior: Known relative frames should be transformed; missing frames should be rejected without mutation.
* Actual behavior: Both known and missing frames are accepted and numeric coordinates are applied as world values.

## Steps to reproduce

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/ros_gz/GZ-F1-set-frame-id

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/GZ-F1-set-frame-id/evidence).

## Output

Both known and missing frames are accepted and numeric coordinates are applied as world values.

The paired control passes. The result reproduced 3/3.

## Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/GZ-F1-set-frame-id
- Source location: ros_gz_sim/src/gz_simulation_interfaces/services/set_entity_state.cpp
- Reproduced: 3/3
- Assessment: reproduced product defect
- Why ordinary tests miss this: World-only inputs never exercise frame semantics.
- Duplicate search: No exact issue located. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
