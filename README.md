# ROS 2 upstream issue reproducers

Source-guided reproducers and saved evidence for upstream ROS 2 issue reports prepared on 2026-09-28.

Each root-cause directory contains the target repository issue body, proposed title, launcher, probe, and completed repeated-run summary. Launchers use /opt/ros/jazzy/setup.bash by default. To test a source build:

    ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

Tested references:

- ros2_controllers: Jazzy 1bc19b63e82a7c04c8ba7c2741b47674c2a6c635; IMU latest tag 6.9.0 at 78d6508ebc82fb1692cfed6e9840b6ebfc0c6756
- ros2_control: master a0f133d921e043aa2cfec8bd7c61effbb8ac132c; GPS and MagneticField semantic components
- ros_gz: 0fa70cb7c15f7500c495190020dd6292188c8e54
- Navigation2: runtime 1.3.13; source reference f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501
- robot_localization: 3efa714fb9c1ff40966327b7bed7053b2570be4d
- MoveIt 2: 002f40e58ebf5322c89468f5049867791e5f67fd

Four reports retain candidate wording. Exact and conservative upstream duplicates are listed in UPSTREAM_DEDUP.md.
