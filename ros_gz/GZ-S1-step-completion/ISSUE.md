## Environment

* OS Version: Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem
* Source build at commit 0fa70cb7c15f7500c495190020dd6292188c8e54
* ROS 2 Jazzy
* RMW: rmw_fastrtps_cpp

## Description

* Expected behavior: A successful paused step response should mean all requested iterations completed.
* Actual behavior: RESULT_OK arrives while time is still at the pre-step value; advancement appears later.

## Steps to reproduce

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/ros_gz/GZ-S1-step-completion

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/GZ-S1-step-completion/evidence).

## Output

RESULT_OK arrives while time is still at the pre-step value; advancement appears later.

The paired control passes. The result reproduced 3/3.

## Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/GZ-S1-step-completion
- Source location: ros_gz_sim/src/gz_simulation_interfaces/services/step_simulation.cpp
- Reproduced: 3/3
- Assessment: reproduced product defect
- Why ordinary tests miss this: Response-only checks miss delayed completion.
- Duplicate search: No exact issue located. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
