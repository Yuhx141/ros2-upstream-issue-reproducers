# 去重排除项

精确上游重复 6 个：

- BOND-1 → [bond_core PR #93](https://github.com/ros/bond_core/pull/93)
- BAG-1 → [rosbag2 issue #2409](https://github.com/ros2/rosbag2/issues/2409)
- NAV-M1 → [navigation2 issue #6232](https://github.com/ros-navigation/navigation2/issues/6232) / [PR #6237](https://github.com/ros-navigation/navigation2/pull/6237)
- BR-TF1 → [ros_gz issue #848](https://github.com/gazebosim/ros_gz/issues/848) / [#410](https://github.com/gazebosim/ros_gz/issues/410)
- MVI-E1 → [moveit2 issue #2808](https://github.com/moveit/moveit2/issues/2808)
- TF2-P1 → [common_interfaces issue #305](https://github.com/ros2/common_interfaces/issues/305)

GZ-R1 没有精确同题 issue，但与 [ros_gz PR #872](https://github.com/gazebosim/ros_gz/pull/872) 的缓存 world stats / paused-state 修复机制明显重合。因此严格口径剩 18，保守提交口径再扣除它后剩 17。
