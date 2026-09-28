# Upstream duplicate audit (2026-09-28)

- Raw independent families: 24
- Exact upstream duplicates: 6
- Strict remainder: 18
- Related-fix overlap excluded conservatively: 1 (GZ-R1 vs [ros_gz #872](https://github.com/gazebosim/ros_gz/pull/872))
- Prepared submission folders: 17
- Maintainer-confirmed first discoveries: 0

Exact matches removed: [bond_core #93](https://github.com/ros/bond_core/pull/93), [rosbag2 #2409](https://github.com/ros2/rosbag2/issues/2409), [navigation2 #6232/#6237](https://github.com/ros-navigation/navigation2/issues/6232), [ros_gz #848](https://github.com/gazebosim/ros_gz/issues/848), [moveit2 #2808](https://github.com/moveit/moveit2/issues/2808), and [common_interfaces #305](https://github.com/ros2/common_interfaces/issues/305).

Related reports rechecked: [Nav2 #6429](https://github.com/ros-navigation/navigation2/issues/6429) concerns CostmapSubscriber/read-resize paths; NAV-C1 is the GetCostmap service snapshot path. [Nav2 #5437/#5438](https://github.com/ros-navigation/navigation2/issues/5437) concerns shutdown cancellation; NAV-1 stalls after a previously discovered service dies during hard reset. robot_localization [#335](https://github.com/cra-ros-pkg/robot_localization/issues/335) and [#360](https://github.com/cra-ros-pkg/robot_localization/issues/360) do not cover invalid-frame overwrite or NaN contamination. No exact MoveIt issue for the direct empty-touch_links path was located; its [high-level API](https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/src/move_group_interface.cpp) fills the default itself.

“No exact issue located” describes a bounded GitHub title/body/source search and does not prove novelty.
