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
       cd ros2-upstream-issue-reproducers/navigation2/NAV-1-dead-service-future

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/navigation2/NAV-1-dead-service-future/evidence).

#### Expected behavior

After bond loss, hard reset should complete or time out.

#### Actual behavior

Crash is detected, but the survivor remains active and manager service gives no response after 12 seconds.

#### Reproduction instructions

The linked directory contains the executable probe, launcher, and completed repeated-run evidence.

#### Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/navigation2/NAV-1-dead-service-future
- Source location: nav2_util/include/nav2_util/service_client.hpp
- Reproduced: 3/3
- Assessment: reproduced high-confidence implementation candidate
- Why ordinary tests miss this: Discovery succeeded before the crash; the blind spot is the unbounded response future.
- Duplicate search: #3033/#3071 concern discovery; #5437/#5438 concern shutdown cancellation, not this trigger. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
