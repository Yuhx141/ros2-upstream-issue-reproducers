# Transmission public-header composition reproducer

`./reproduce.sh` configures a two-include translation unit and builds it. Affected versions fail with two declarations of `HW_IF_ABSOLUTE_POSITION`.
