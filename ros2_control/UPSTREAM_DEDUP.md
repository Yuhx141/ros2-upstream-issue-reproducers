# Upstream duplicate check

Checked 2026-09-28 against open and closed issues and pull requests in `ros-controls/ros2_control`.

- GPS-E1 queries: `gps sensor`, `NavSatStatus`, `get_optional failure`, and `state interface read failed`.
- No issue or PR with the same root cause was found.

## MAG-L1

- Final queries covered `MagneticFieldSensor`, `magnetometer stale`, `magnetometer lock`, `magnetometer NaN`, and `magnetic field mixed values` in both `ros2_control` and `ros2_controllers`.
- Only the component/broadcaster introduction PRs (#2627 and ros2_controllers#2214 plus backports) matched; no same-root issue or fix PR was found.
