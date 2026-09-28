**Describe the bug**

Valid x=2 becomes approximately x=0 after invalid-frame set_pose in EKF and UKF.

**To Reproduce**

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/robot_localization/RL-F1-invalid-frame-set-pose

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/robot_localization/RL-F1-invalid-frame-set-pose/evidence).

**Expected behavior**

An unavailable frame should be rejected or preserve current state.

**Desktop (please complete the following information):**
- OS: Ubuntu 24.04.4 LTS, x86_64
- ROS Distribution: Jazzy
- robot_localization Package Version: source commit 3efa714fb9c1ff40966327b7bed7053b2570be4d
- RMW: rmw_fastrtps_cpp

**Additional context**

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/robot_localization/RL-F1-invalid-frame-set-pose
- Source location: src/ros_filter.cpp RosFilter<T>::setPoseCallback
- Reproduced: 6/6 variants; 6/6 controls
- Assessment: reproduced robustness candidate
- Why ordinary tests miss this: preparePose failure is ignored before zero-initialized state is installed.
- Duplicate search: No exact issue located; #335 and #360 differ. Checked 2026-09-28. This is a directed search, not a claim of first discovery.

This report keeps the robustness-candidate wording because the service contract does not explicitly specify rejection of this input, while the observed state mutation is reproducible.
