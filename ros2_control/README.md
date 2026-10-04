# ros2_control confirmed roots

| ID | Component | Root cause | Current-source result | Fixcheck |
|---|---|---|---|---|
| GPS-E1 | `semantic_components::GPSSensor` | Failed interface reads publish invalid status/service values 127/65535 | 3/3 violation at master `a0f133d` | target 3/3 + GPS suite 7/7 pass |
| MAG-L1 | `semantic_components::MagneticFieldSensor` | Failed reads reuse initial zero or a previous per-axis cache | baseline 3/3 pass and two-case reproducer 3/3 fail at master `a0f133d` | target 3/3 + focused CTest 1/1 + broadcaster package 14/14 pass |

Each directory contains an issue title/body, upstream-native test patch, minimal root-cause fix, launcher, evidence, and upstream URL.

- GPS-E1: https://github.com/ros-controls/ros2_control/issues/3643
- MAG-L1: https://github.com/ros-controls/ros2_control/issues/3644
