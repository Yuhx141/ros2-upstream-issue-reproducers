cmake_minimum_required(VERSION 3.8)
project(p3_moveit_tem_probe)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(tem_mdof_probe tem_mdof_probe.cpp)
ament_target_dependencies(tem_mdof_probe rclcpp moveit_ros_planning moveit_msgs)
