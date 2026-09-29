# TrajectoryExecutionManager misroutes MultiDOF velocities and drops accelerations

Splitting a trajectory between controllers preserves each MultiDOF joint's transform, velocity and acceleration by name.

Affected behavior: For two reordered MultiDOF joints, names and transforms are correct, velocities become `[2, 0]` instead of `[1, 2]`, and accelerations are empty. The target failed 3/3; one-MultiDOF and single-DOF controls passed 6/6.
