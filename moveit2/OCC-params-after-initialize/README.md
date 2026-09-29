# Occupancy map updater parameters are applied after initialize()

The public updater lifecycle is `setMonitor -> setParams -> initialize`, so an updater may rely on configured parameters during initialization.

The observed order is `initialize(false), set_params, start`. A parameter-dependent updater fails initialization before receiving its parameters and is still started. Reordering the calls gives `set_params, initialize(true), start`.
