cmake_minimum_required(VERSION 3.8)
project(p3_costmap_atomicity_probe)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(costmap_atomicity_probe src/costmap_atomicity_probe.cpp)
ament_target_dependencies(costmap_atomicity_probe
  nav2_costmap_2d nav2_msgs nav2_util rclcpp)
install(TARGETS costmap_atomicity_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
