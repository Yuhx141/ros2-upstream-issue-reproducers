cmake_minimum_required(VERSION 3.16)
project(joint_transmission_p0)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transmission_interface REQUIRED)

ament_add_gtest(joint_transmission_p0_test
  joint_transmission_p0_test.cpp simple_transmission_probe.cpp)
ament_target_dependencies(joint_transmission_p0_test
  hardware_interface joint_limits pluginlib rclcpp transmission_interface)
install(TARGETS joint_transmission_p0_test DESTINATION lib/${PROJECT_NAME})
ament_package()
