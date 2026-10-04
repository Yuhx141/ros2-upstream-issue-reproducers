I checked the sibling `force_torque_sensor_broadcaster` and found the same retained-handle failure mode.

At 6.9.0 (`78d6508ebc82fb1692cfed6e9840b6ebfc0c6756`; implementation byte-identical on current master `2520ae5`), I exported once before activation, retained the same six handles, then applied two distinct six-axis samples. The `~/wrench` topic was correct for all 12 field observations, while all 12 retained exported values stayed at their export-time zeros. This reproduced in 3/3 independent processes.

This is the same root cause rather than a separate issue: `on_export_state_interfaces_list()` creates owning state objects from the current `wrench_raw_`, and `update_and_write_commands()` never writes later wrench values to `ordered_exported_state_interfaces_`. PR #1215 describes the exported values as the same offset-adjusted values published for chaining, but its test exports only after publishing and sees a fresh snapshot.

A fix that records each exported object's semantic axis and writes the calibrated raw wrench into those retained objects after every update makes the target pass 3/3. With two unrelated Force/Torque fixes and six normal-path closure tests present, the complete package regression passes 44 tests with zero failures.
