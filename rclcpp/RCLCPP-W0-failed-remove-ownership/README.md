# rclcpp RCLCPP-W0 portable reproducer

The reproducer uses only public rclcpp APIs. It checks that a failed removal from a non-owning WaitSet does not release the GuardCondition ownership held by another WaitSet.

## Build and run

Source the ROS 2 environment, then run:

    cmake -S . -B build
    cmake --build build
    ./build/rclcpp_waitset_probe

Expected output:

    direct_cross_add_threw=1
    wrong_remove_threw=1
    post_failure_cross_add_threw=1

Affected output:

    direct_cross_add_threw=1
    wrong_remove_threw=1
    post_failure_cross_add_threw=0

An affected build exits with status 1. The installed Jazzy 28.1.21 release reproduced 3/3. Jazzy commit 2209942eb1361fdaf48ec8512b6dccf70235bccb and the relevant templates from rolling commit eee4b508d4357c81c0847f922617e5f498d1d4b7 also reproduced 3/3.

The fixcheck patch is causal evidence only. A production patch should cover the shared remove ordering for every entity type and subscription mask, with failure-atomic tests.
