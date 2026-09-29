#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr auto robot_urdf = R"(<robot name="p3_psm">
  <link name="base"/>
  <link name="slider"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
  <joint name="slide" type="prismatic"><parent link="base"/><child link="slider"/>
    <axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
</robot>)";
constexpr auto robot_srdf = R"(<robot name="p3_psm"><virtual_joint name="world_joint" type="fixed" parent_frame="world" child_link="base"/></robot>)";

moveit_msgs::msg::CollisionObject box(const std::string& id)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = id;
  object.header.frame_id = "base";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = { 0.1, 0.1, 0.1 };
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  object.primitives = { shape };
  object.primitive_poses = { pose };
  return object;
}

std::string join(std::vector<std::string> values)
{
  std::sort(values.begin(), values.end());
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i)
      stream << ',';
    stream << values[i];
  }
  return stream.str();
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::vector<rclcpp::Parameter> parameters{
    rclcpp::Parameter("robot_description", std::string(robot_urdf)),
    rclcpp::Parameter("robot_description_semantic", std::string(robot_srdf))
  };
  auto options = rclcpp::NodeOptions().parameter_overrides(parameters);
  auto node = std::make_shared<rclcpp::Node>("p3_psm_stop_probe", options);
  auto monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description", "p3_psm");
  if (!monitor->getPlanningScene())
  {
    std::cerr << "planning scene unavailable\n";
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  std::atomic<unsigned> updates{ 0 };
  monitor->addUpdateCallback([&](auto) { ++updates; });
  const std::string collision_topic = "/p3_psm_collision";
  const std::string world_topic = "/p3_psm_world";
  auto collision_pub = node->create_publisher<moveit_msgs::msg::CollisionObject>(collision_topic, rclcpp::ServicesQoS());
  auto world_pub = node->create_publisher<moveit_msgs::msg::PlanningSceneWorld>(world_topic, rclcpp::ServicesQoS());
  monitor->startWorldGeometryMonitor(collision_topic, world_topic, false);

  auto ids = [&] {
    planning_scene_monitor::LockedPlanningSceneRO scene(monitor);
    return scene->getWorld()->getObjectIds();
  };
  auto wait_for = [](auto condition, auto timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (condition())
        return true;
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  };
  const bool discovered = wait_for(
      [&] { return collision_pub->get_subscription_count() == 1 && world_pub->get_subscription_count() == 1; }, 2s);

  collision_pub->publish(box("collision_before"));
  const bool collision_before = wait_for([&] { return join(ids()) == "collision_before"; }, 1s);
  const auto collision_before_ids = join(ids());

  moveit_msgs::msg::PlanningSceneWorld world;
  world.collision_objects = { box("world_before") };
  world_pub->publish(world);
  const bool world_before = wait_for([&] { return join(ids()) == "world_before"; }, 1s);
  const auto world_before_ids = join(ids());

  monitor->stopWorldGeometryMonitor();
  const bool double_stop = std::getenv("P3_DOUBLE_STOP") != nullptr;
  if (double_stop)
    monitor->stopWorldGeometryMonitor();
  wait_for([&] { return collision_pub->get_subscription_count() == 0; }, 1s);
  std::vector<std::string> topics_after;
  monitor->getMonitoredTopics(topics_after);
  const auto collision_subscriptions_after = collision_pub->get_subscription_count();
  const auto world_subscriptions_after = world_pub->get_subscription_count();

  const auto updates_before_collision = updates.load();
  for (int i = 0; i < 10; ++i)
  {
    collision_pub->publish(box("collision_after"));
    std::this_thread::sleep_for(20ms);
  }
  const bool collision_updated_after_stop = join(ids()) != "world_before" || updates.load() != updates_before_collision;
  const auto collision_after_ids = join(ids());

  const auto updates_before_world = updates.load();
  world.collision_objects = { box("world_after") };
  for (int i = 0; i < 10 && join(ids()) != "world_after"; ++i)
  {
    world_pub->publish(world);
    std::this_thread::sleep_for(20ms);
  }
  const bool world_updated_after_stop = wait_for(
      [&] { return join(ids()) == "world_after" || updates.load() != updates_before_world; }, 500ms);
  const auto world_after_ids = join(ids());

  const bool harness_valid = discovered && collision_before && world_before && !collision_updated_after_stop;
  std::cout << "P3_RESULT {"
            << "\"double_stop\":" << double_stop << ','
            << "\"harness_valid\":" << harness_valid << ','
            << "\"discovered\":" << discovered << ','
            << "\"collision_before\":" << collision_before << ','
            << "\"world_before\":" << world_before << ','
            << "\"collision_subscriptions_after\":" << collision_subscriptions_after << ','
            << "\"world_subscriptions_after\":" << world_subscriptions_after << ','
            << "\"topics_after\":\"" << join(topics_after) << "\","
            << "\"collision_before_ids\":\"" << collision_before_ids << "\","
            << "\"world_before_ids\":\"" << world_before_ids << "\","
            << "\"collision_updated_after_stop\":" << collision_updated_after_stop << ','
            << "\"collision_after_ids\":\"" << collision_after_ids << "\","
            << "\"world_updated_after_stop\":" << world_updated_after_stop << ','
            << "\"world_after_ids\":\"" << world_after_ids << "\"}\n";

  monitor.reset();
  executor.cancel();
  spin.join();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return harness_valid ? 0 : 3;
}
