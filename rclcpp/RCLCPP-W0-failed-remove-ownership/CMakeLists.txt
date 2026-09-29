cmake_minimum_required(VERSION 3.16)
project(rclcpp_waitset_probe LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(rclcpp_waitset_probe rclcpp_waitset_probe.cpp)
target_compile_features(rclcpp_waitset_probe PRIVATE cxx_std_17)
target_include_directories(rclcpp_waitset_probe BEFORE PRIVATE "${RCLCPP_SOURCE_INCLUDE}")
ament_target_dependencies(rclcpp_waitset_probe rclcpp)
