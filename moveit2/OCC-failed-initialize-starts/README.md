# OccupancyMapMonitor starts an updater after initialize() returns false

An updater that returns false from `initialize()` should not be added or started.

A fixed-failure updater records `initialize(false), set_params, start` in 3/3 runs. Preserving the bool and rejecting the updater stops the sequence after `initialize(false)`.
