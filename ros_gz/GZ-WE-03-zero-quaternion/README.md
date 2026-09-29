# SetEntityState zero-quaternion reproducer

Tested with `ros_gz_sim` 1.0.24 on ROS 2 Jazzy. The runner includes valid-request and health controls and saves one JSON record per arm. The relevant arm is `invalid_pose`; affected output reports `RESULT_OK` instead of `INVALID_POSE`.
