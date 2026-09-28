**Describe the bug**

NaN pose/covariance yields non-finite odometry and later finite input does not recover.

**To Reproduce**

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/robot_localization/RL-N1-nan-set-pose

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/robot_localization/RL-N1-nan-set-pose/evidence).

**Expected behavior**

Non-finite pose/covariance should be rejected or isolated.

**Desktop (please complete the following information):**
- OS: Ubuntu 24.04.4 LTS, x86_64
- ROS Distribution: Jazzy
- robot_localization Package Version: source commit 3efa714fb9c1ff40966327b7bed7053b2570be4d
- RMW: rmw_fastrtps_cpp

**Additional context**

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/robot_localization/RL-N1-nan-set-pose
- Source location: src/ros_filter.cpp RosFilter<T>::setPoseCallback
- Reproduced: 12/12 variants; 12/12 controls
- Assessment: reproduced robustness candidate
- Why ordinary tests miss this: The service completes; every output field and recovery phase must be checked.
- Duplicate search: No exact issue located; #360 does not involve NaN. Checked 2026-09-28. This is a directed search, not a claim of first discovery.

This report keeps the robustness-candidate wording because the service contract does not explicitly specify rejection of this input, while the observed state mutation is reproducible.
