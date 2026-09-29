cmake_minimum_required(VERSION 3.8)
project(p3_moveit_psm_probe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(psm_stop_world_probe psm_stop_world_probe.cpp)
ament_target_dependencies(psm_stop_world_probe rclcpp moveit_ros_planning moveit_msgs shape_msgs)
