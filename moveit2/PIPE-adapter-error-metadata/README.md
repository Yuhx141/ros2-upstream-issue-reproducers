# PlanningPipeline drops request-adapter error message and source

The adapter's complete `MoveItErrorCode` should reach the pipeline result, including `message` and `source`.

For an out-of-bounds start, the pipeline stage keeps code `-26` but has empty message/source. Copying the full status restores `Start state out of bounds.` and `CheckStartStateBounds`. This reproduced 3/3.
