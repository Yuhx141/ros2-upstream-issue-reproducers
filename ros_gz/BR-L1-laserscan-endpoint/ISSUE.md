## Environment

* OS Version: Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem
* Source build at commit 0fa70cb7c15f7500c495190020dd6292188c8e54
* ROS 2 Jazzy
* RMW: rmw_fastrtps_cpp

## Description

* Expected behavior: Angles 0, 0.1, 0.2 with increment 0.1 describe three samples.
* Actual behavior: The Gazebo scan reports count=2 and keeps only the first two samples; reverse control passes.

## Steps to reproduce

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/ros_gz/BR-L1-laserscan-endpoint

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/BR-L1-laserscan-endpoint/evidence).

## Output

The Gazebo scan reports count=2 and keeps only the first two samples; reverse control passes.

The paired control passes. The result reproduced 3/3.

## Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/BR-L1-laserscan-endpoint
- Source location: ros_gz_bridge/src/convert/sensor_msgs.cpp
- Reproduced: 3/3
- Assessment: reproduced product defect
- Why ordinary tests miss this: Count, endpoint algebra, and payload length must be checked together.
- Duplicate search: No exact issue located; #368 is performance-related. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
