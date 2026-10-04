# ros2_controllers confirmed roots

The first eight directories reproduce against Jazzy commit
`1bc19b63e82a7c04c8ba7c2741b47674c2a6c635`, which was still the Jazzy head on
2026-09-28. IMU-E1 reproduces against tag 6.9.0
(`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`). All corresponding affected code
was still present on master `2520ae5b6d18c01f44083491cd9a285b1e2cdfe0` when checked.

| ID | Component | Root cause | Current-source result | Fixcheck |
|---|---|---|---|---|
| RC-B1 | battery_state_broadcaster | Publishes percentage in 0–100 instead of message contract 0–1 | target cases 3/3 violation | 3/3 tests pass |
| RC-P1 | pid_controller `~/reference` | Aliased input is reset before name lookup | permutation/unknown/duplicate each 3/3 violation | 4/4 pass |
| RC-P2 | pid_controller `~/measured_state` | Names are ignored and arrays are consumed by position | permutation/unknown/duplicate each 3/3 violation | 7/7 pass |
| RC-P3 | pid_controller `~/measured_state` | Accepted empty derivative vector is indexed | assertion build aborts 3/3 | isolated test passes |

| PG-L1 | parallel_gripper_controller | on_deactivate leaves the active goal and action server alive | two symptom classes each 3/3 violation | 15/15 pass |
| PG-D1 | parallel_gripper_controller | Terminal result effort reads never-initialized `computed_command_` | success/stall controlled storage patterns 18/18 violation | 18/18 targeted + 15/15 regression pass |
| PG-D2 | parallel_gripper_controller | Ignores non-empty goal joint name and commands configured joint | mismatched name accepted and command applied 3/3 | target 3/3 + 19/19 regression pass |
| PG-B1 | parallel_gripper_controller | Schema accepts zero tolerance but strict comparison rejects exact error | exact zero-error goal stays executing 3/3 | target 3/3 + 19/19 regression pass |

| IMU-E1 | imu_sensor_broadcaster | Exported state objects are initialized once and never synchronized after update | topic correct, retained handles stale 3/3 | target 3/3 + 14/14 package tests pass |

Each directory contains an issue title/body, an independently applicable test patch, the minimal root-cause fix used for the fixcheck, a launcher, and compact evidence.

| GPS-T1 | gps_sensor_broadcaster | Ignores controller update time and stamps with node clock | timestamp fields fail 3/3; data fields pass | target 3/3 + package 14/14 pass |
| FT-E1 | force_torque_sensor_broadcaster | Same retained-export liveness root as IMU-E1 | topic 0/12 failures, handles 12/12 stale in 3/3 | target 3/3 + combined package 44/44 pass |
| FT-F1 | force_torque_sensor_broadcaster | Invalid FilterChain configure result is ignored | target 3/3 violation | target 3/3 + combined package 44/44 pass |
| FT-P1 | force_torque_sensor_broadcaster | Accepted frame parameter update is not applied to messages | target 3/3 violation | target 3/3 + combined package 44/44 pass |

| JSB-U1 | joint_state_broadcaster | `use_urdf_to_filter=false` still enters URDF-only selection | two target branches 3/3 violation | targets 3/3 + valid regression 27/27 pass |
| JSB-E1 | joint_state_broadcaster | extra dedup uses all cached interfaces instead of outgoing JointState names | Jazzy and 6.9 each 3/3 omitted | each target 3/3; valid regressions pass |
| JSB-D1 | joint_state_broadcaster (Jazzy) | DynamicJointState reuses mapped JointState cache and loses custom interface identity | target 3/3 violation | target 3/3 + valid regression 29/29 pass |
| JSB-O1 | joint_state_broadcaster | URDF model map yields lexical order, same family as #159/#1572 | Jazzy and 6.9 each 3/3 repeat | explicit-order control each 3/3 pass |


| RNG-D1 | range_sensor_broadcaster | Undefined `SENSOR_MSGS_VERSION_MAJOR` removes variance assignment and matching tests | configured 1.0 publishes 0.0 in 3/3 | target 3/3 + package CTest 2/2 pass |
| RNG-P1 | range_sensor_broadcaster | Writable parameters are sampled only during configure | six used fields/interface remain stale in 3/3 | target 3/3 + package CTest 2/2 pass |
| RNG-V1 | range_sensor_broadcaster | Range metadata lacks enum, numeric, and cross-field validation | five invalid classes accepted in 3/3 | targets 3/3 + package CTest 2/2 pass |

RNG-P1 is the same behavior family as FT-P1/#2657 but occurs in a separate component implementation. RNG-D1 and RNG-V1 are independent roots. Range results were reproduced on 6.9.0; Jazzy head and current master have byte-identical component source, parameter schema, and CMake files.
