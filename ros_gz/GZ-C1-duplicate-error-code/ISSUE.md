## Environment

* OS Version: Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem
* Source build at commit 0fa70cb7c15f7500c495190020dd6292188c8e54
* ROS 2 Jazzy
* RMW: rmw_fastrtps_cpp

## Description

* Expected behavior: A duplicate name with allow_renaming=false should return NAME_NOT_UNIQUE (101).
* Actual behavior: The request is rejected with generic result code 4.

## Steps to reproduce

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/ros_gz/GZ-C1-duplicate-error-code

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/GZ-C1-duplicate-error-code/evidence).

## Output

The request is rejected with generic result code 4.

The paired control passes. The result reproduced 3/3.

## Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/GZ-C1-duplicate-error-code
- Source location: ros_gz_sim/src/gz_simulation_interfaces/services/spawn_entity.cpp
- Reproduced: 3/3
- Assessment: reproduced product defect
- Why ordinary tests miss this: Boolean success/failure agrees; the public result enum does not.
- Duplicate search: No exact issue located; source contains a related TODO. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
