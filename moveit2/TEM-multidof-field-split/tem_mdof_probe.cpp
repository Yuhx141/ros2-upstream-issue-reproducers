#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr auto robot_urdf = R"(<robot name="p3_tem">
  <link name="world"/>
  <link name="base"/>
  <link name="slider"/>
  <link name="tool"/>
  <joint name="base_joint" type="floating">
    <parent link="world"/><child link="base"/>
  </joint>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="slider"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="floating">
    <parent link="slider"/><child link="tool"/>
  </joint>
</robot>)";
constexpr auto robot_srdf = R"(<robot name="p3_tem">
  <group name="all"><joint name="base_joint"/><joint name="slide"/><joint name="tool_joint"/></group>
</robot>)";

template <typename T>
std::string join(const std::vector<T>& values)
{
  std::ostringstream out;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i)
      out << ',';
    out << values[i];
  }
  return out.str();
}

std::string escape(const std::string& value)
{
  std::string out;
  for (const char c : value)
  {
    if (c == '\\' || c == '"')
      out += '\\';
    if (c == '\n')
      out += "\\n";
    else
      out += c;
  }
  return out;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const std::string mode = argc > 1 ? argv[1] : "single_dof";
  const bool two_mdof = mode == "mdof_two_reorder";
  const std::vector<std::string> controller_joints =
      mode == "single_dof" ? std::vector<std::string>{ "slide" } :
      (two_mdof ? std::vector<std::string>{ "base_joint", "tool_joint" } :
                  std::vector<std::string>{ "base_joint" });

  std::vector<rclcpp::Parameter> parameters{
    { "robot_description", std::string(robot_urdf) },
    { "robot_description_semantic", std::string(robot_srdf) },
    { "moveit_controller_manager", "moveit_simple_controller_manager/MoveItSimpleControllerManager" },
    { "moveit_simple_controller_manager.controller_names", std::vector<std::string>{ "p3_controller" } },
    { "moveit_simple_controller_manager.p3_controller.action_ns", "follow_joint_trajectory" },
    { "moveit_simple_controller_manager.p3_controller.type", "FollowJointTrajectory" },
    { "moveit_simple_controller_manager.p3_controller.default", true },
    { "moveit_simple_controller_manager.p3_controller.joints", controller_joints },
    { "trajectory_execution.control_multi_dof_joint_variables", false }
  };
  auto options = rclcpp::NodeOptions().parameter_overrides(parameters).automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("p3_tem_mdof_probe", options);
  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  const auto model = loader.getModel();
  if (!model)
    return 2;

  const auto* base_joint = model->getJointModel("base_joint");
  const auto* tool_joint = model->getJointModel("tool_joint");
  const bool harness_valid = base_joint && tool_joint &&
      base_joint->getType() == moveit::core::JointModel::FLOATING &&
      tool_joint->getType() == moveit::core::JointModel::FLOATING;

  bool push_return = false;
  std::string exception;
  std::size_t queued = 0;
  std::vector<std::string> out_joint_names;
  std::vector<std::string> out_mdof_names;
  std::vector<double> out_transform_x;
  std::vector<double> out_velocity_x;
  std::vector<double> out_acceleration_x;

  try
  {
    auto* manager = new trajectory_execution_manager::TrajectoryExecutionManager(node, model, nullptr, false);
    moveit_msgs::msg::RobotTrajectory trajectory;
    if (mode == "single_dof")
    {
      trajectory.joint_trajectory.joint_names = { "slide" };
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = { 0.25 };
      point.time_from_start.sec = 1;
      trajectory.joint_trajectory.points = { point };
    }
    else
    {
      trajectory.multi_dof_joint_trajectory.joint_names =
          two_mdof ? std::vector<std::string>{ "tool_joint", "base_joint" } :
                     std::vector<std::string>{ "base_joint" };
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
      for (std::size_t i = 0; i < trajectory.multi_dof_joint_trajectory.joint_names.size(); ++i)
      {
        geometry_msgs::msg::Transform transform;
        transform.translation.x = two_mdof ? (i == 0 ? 20.0 : 10.0) : 10.0;
        transform.rotation.w = 1.0;
        point.transforms.push_back(transform);
        geometry_msgs::msg::Twist velocity;
        velocity.linear.x = two_mdof ? (i == 0 ? 2.0 : 1.0) : 1.0;
        point.velocities.push_back(velocity);
        if (two_mdof)
        {
          geometry_msgs::msg::Twist acceleration;
          acceleration.linear.x = i == 0 ? 4.0 : 3.0;
          point.accelerations.push_back(acceleration);
        }
      }
      point.time_from_start.sec = 1;
      trajectory.multi_dof_joint_trajectory.points = { point };
    }

    push_return = manager->push(trajectory);
    queued = manager->getTrajectories().size();
    if (queued == 1 && !manager->getTrajectories()[0]->trajectory_parts_.empty())
    {
      const auto& part = manager->getTrajectories()[0]->trajectory_parts_[0];
      out_joint_names = part.joint_trajectory.joint_names;
      out_mdof_names = part.multi_dof_joint_trajectory.joint_names;
      if (!part.multi_dof_joint_trajectory.points.empty())
      {
        const auto& point = part.multi_dof_joint_trajectory.points[0];
        for (const auto& value : point.transforms)
          out_transform_x.push_back(value.translation.x);
        for (const auto& value : point.velocities)
          out_velocity_x.push_back(value.linear.x);
        for (const auto& value : point.accelerations)
          out_acceleration_x.push_back(value.linear.x);
      }
    }
  }
  catch (const std::exception& error)
  {
    exception = error.what();
  }

  std::cout << std::boolalpha << "P3_RESULT {"
            << "\"mode\":\"" << mode << "\","
            << "\"harness_valid\":" << harness_valid << ','
            << "\"controller_joints\":\"" << join(controller_joints) << "\","
            << "\"push_return\":" << push_return << ','
            << "\"exception\":\"" << escape(exception) << "\","
            << "\"queued\":" << queued << ','
            << "\"out_joint_names\":\"" << join(out_joint_names) << "\","
            << "\"out_mdof_names\":\"" << join(out_mdof_names) << "\","
            << "\"out_transform_x\":\"" << join(out_transform_x) << "\","
            << "\"out_velocity_x\":\"" << join(out_velocity_x) << "\","
            << "\"out_acceleration_x\":\"" << join(out_acceleration_x) << "\"}" << std::endl;
  std::_Exit(harness_valid ? 0 : 3);
}
