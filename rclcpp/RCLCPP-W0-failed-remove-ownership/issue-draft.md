# Title

WaitSet failed remove clears entity ownership and permits the same guard condition in two wait sets

## Bug report

**Required Info:**

- Operating System:
  - Ubuntu 24.04.4 LTS
- Installation type:
  - ROS 2 Jazzy binaries; also reproduced with the current Jazzy source header
- Version or commit hash:
  - Binary package: rclcpp 28.1.21-1noble.20260615.133124
  - Jazzy: 2209942eb1361fdaf48ec8512b6dccf70235bccb
  - The affected code is also present in rolling: eee4b508d4357c81c0847f922617e5f498d1d4b7
- DDS implementation:
  - rmw_fastrtps_cpp, although the reproducer does not create DDS entities
- Client library:
  - rclcpp

#### Steps to reproduce issue

Portable reproducer and build instructions:

REPLACE_WITH_IMMUTABLE_PUBLIC_URL_BEFORE_SUBMISSION

The essential sequence is:

    rclcpp::WaitSet owner;
    rclcpp::WaitSet other;
    auto gc = std::make_shared<rclcpp::GuardCondition>();

    owner.add_guard_condition(gc);
    EXPECT_THROW(other.add_guard_condition(gc), std::runtime_error);
    EXPECT_THROW(other.remove_guard_condition(gc), std::runtime_error);
    EXPECT_THROW(other.add_guard_condition(gc), std::runtime_error);

The first cross-set add is a control. The remove from other correctly throws because other does not contain gc. The final add checks that the failed remove preserved owner ownership.

The portable program was run in three independent processes for each checked version.

#### Expected behavior

The final other.add_guard_condition(gc) should throw runtime_error. owner still contains gc, and the failed remove from other should not change that membership or the entity's in-use state.

Expected probe output:

    direct_cross_add_threw=1
    wrong_remove_threw=1
    post_failure_cross_add_threw=1

#### Actual behavior

The final add succeeds:

    direct_cross_add_threw=1
    wrong_remove_threw=1
    post_failure_cross_add_threw=0

This was reproduced 3/3 with the installed Jazzy 28.1.21 release and 3/3 with the current Jazzy branch header. The affected WaitSetTemplate and DynamicStorage path is unchanged in current rolling, where the same template instantiation also reproduced 3/3.

#### Additional information

remove_guard_condition clears in_use_by_wait_set before DynamicStorage verifies that the guard condition belongs to this WaitSet. DynamicStorage then throws for a missing member, leaving the global in-use flag false even though the original WaitSet still contains the guard condition. The next add to another WaitSet therefore succeeds.

A minimal causal check that validates/removes from storage before clearing the in-use flag changes the exact probe from 3/3 failures to 3/3 passes. Similar ordering appears in timer, client, service, waitable, and subscription removal paths, so a fix should review the shared failure-atomicity behavior rather than only the GuardCondition reproducer.

I searched existing rclcpp issues and pull requests for failed remove, WaitSet ownership, and in_use_by_wait_set; I found adjacent WaitSet issues but no report with this trigger and state corruption.
