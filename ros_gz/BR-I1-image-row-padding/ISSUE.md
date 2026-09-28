## Environment

* OS Version: Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem
* Source build at commit 0fa70cb7c15f7500c495190020dd6292188c8e54
* ROS 2 Jazzy
* RMW: rmw_fastrtps_cpp

## Description

* Expected behavior: A 2x2 mono image with step=4 should preserve row pixels while skipping padding.
* Actual behavior: Both directions copy a tight prefix: expected [21,22,23,24], observed [21,22,201,202]. Tight controls pass.

## Steps to reproduce

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/ros_gz/BR-I1-image-row-padding

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/BR-I1-image-row-padding/evidence).

## Output

Both directions copy a tight prefix: expected [21,22,23,24], observed [21,22,201,202]. Tight controls pass.

The paired control passes. The result reproduced 3/3 both directions.

## Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/ros_gz/BR-I1-image-row-padding
- Source location: ros_gz_bridge/src/convert/sensor_msgs.cpp
- Reproduced: 3/3 both directions
- Assessment: reproduced product defect
- Why ordinary tests miss this: Most images have tight rows; a padded row is required to expose the branch.
- Duplicate search: No exact issue located; TF child-frame issues #848/#410 are unrelated. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
