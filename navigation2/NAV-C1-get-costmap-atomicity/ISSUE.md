## Bug report

**Required Info:**

- Operating System:
  - Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem
- Computer:
  - x86_64 workstation
- ROS2 Version:
  - Jazzy binaries
- Version or commit hash:
  - Navigation2 1.3.13; source reference f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501
- DDS implementation:
  - rmw_fastrtps_cpp

#### Steps to reproduce issue

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/navigation2/NAV-C1-get-costmap-atomicity

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/navigation2/NAV-C1-get-costmap-atomicity/evidence).

#### Expected behavior

The service should return a coherent snapshot and wait while the master mutex is held.

#### Actual behavior

It returns immediately with [254,0] while the writer later completes [254,253].

#### Reproduction instructions

The linked directory contains the executable probe, launcher, and completed repeated-run evidence.

#### Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/navigation2/NAV-C1-get-costmap-atomicity
- Source location: nav2_costmap_2d/src/costmap_2d_publisher.cpp
- Reproduced: 3/3 deterministic
- Assessment: reproduced product defect
- Why ordinary tests miss this: A deterministic mutex-holding probe reaches the otherwise rare inter-layer window.
- Duplicate search: No exact service-snapshot issue located; #6009/#6429 use other read/resize paths. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
