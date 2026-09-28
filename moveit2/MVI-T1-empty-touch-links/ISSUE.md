## Description

Apply and identity transfer succeed, but the object collides with its own link and planning fails; explicit link succeeds.

This is a contract candidate. The message documentation says link_name is considered by default when touch_links is empty, while the direct ApplyPlanningScene path behaves differently from an explicit touch link.

## ROS Distro

Jazzy

## OS and version

Ubuntu 24.04.4 LTS, x86_64, Linux 6.17.0-1032-oem

## Source or binary build?

Source

## Source version

Commit 002f40e58ebf5322c89468f5049867791e5f67fd; runtime library 2.12.4

## Which RMW are you using?

FastRTPS (rmw_fastrtps_cpp)

## Steps to Reproduce

1. Clone the reproducer repository.

       git clone https://github.com/Yuhx141/ros2-upstream-issue-reproducers.git
       cd ros2-upstream-issue-reproducers/moveit2/MVI-T1-empty-touch-links

2. Run against a source installation or the default Jazzy binary installation.

       ROS_SETUP=/path/to/install/setup.bash ./reproduce.sh

   ROS_SETUP may be omitted to use /opt/ros/jazzy/setup.bash.
3. Inspect the generated run directory and the saved [repeated-run evidence](https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/moveit2/MVI-T1-empty-touch-links/evidence).

## Expected behavior

The message contract says link_name is considered by default when touch_links is empty.

## Actual behavior

Apply and identity transfer succeed, but the object collides with its own link and planning fails; explicit link succeeds.

## Backtrace or Console output

No crash occurs. Empty touch_links: planning code 99999, 0 trajectory points, 3/3 runs. Explicit touch_links containing slider: success code 1, 101 trajectory points, 3/3 controls.

## Additional information

- Reproducer and evidence: https://github.com/Yuhx141/ros2-upstream-issue-reproducers/tree/main/moveit2/MVI-T1-empty-touch-links
- Source location: moveit_core/planning_scene/src/planning_scene.cpp
- Reproduced: 3/3 variants; 3/3 controls
- Assessment: reproduced contract candidate
- Why ordinary tests miss this: Apply success and object identity both look healthy; validity/planning reveal the effect.
- Duplicate search: No exact issue located; high-level attachObject already fills link_name. Checked 2026-09-28. This is a directed search, not a claim of first discovery.
