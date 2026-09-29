#include <iostream>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool direct_cross_add_threw = false;
  bool wrong_remove_threw = false;
  bool post_failure_cross_add_threw = false;

  {
    rclcpp::WaitSet owner;
    rclcpp::WaitSet other;
    auto guard_condition = std::make_shared<rclcpp::GuardCondition>();

    owner.add_guard_condition(guard_condition);
    try {
      other.add_guard_condition(guard_condition);
    } catch (const std::runtime_error &) {
      direct_cross_add_threw = true;
    }

    try {
      other.remove_guard_condition(guard_condition);
    } catch (const std::runtime_error &) {
      wrong_remove_threw = true;
    }

    try {
      other.add_guard_condition(guard_condition);
    } catch (const std::runtime_error &) {
      post_failure_cross_add_threw = true;
    }
  }

  rclcpp::shutdown();
  std::cout
    << "direct_cross_add_threw=" << direct_cross_add_threw << "\n"
    << "wrong_remove_threw=" << wrong_remove_threw << "\n"
    << "post_failure_cross_add_threw=" << post_failure_cross_add_threw << "\n";

  return direct_cross_add_threw && wrong_remove_threw && post_failure_cross_add_threw ? 0 : 1;
}
