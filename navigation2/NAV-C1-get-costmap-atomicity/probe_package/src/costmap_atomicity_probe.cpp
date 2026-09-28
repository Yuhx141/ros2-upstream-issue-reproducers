#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto publisher_node = std::make_shared<nav2_util::LifecycleNode>("p3_atomicity_publisher");
  auto client_node = std::make_shared<rclcpp::Node>("p3_atomicity_client");
  nav2_costmap_2d::Costmap2D costmap(4, 4, 0.1, 0.0, 0.0, 0);
  auto publisher = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    publisher_node, &costmap, "map", "atomicity_probe", true);
  auto client = client_node->create_client<nav2_msgs::srv::GetCostmap>(
    "/get_atomicity_probe");

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(publisher_node->get_node_base_interface());
  executor.add_node(client_node);
  std::thread spinner([&executor]() {executor.spin();});

  if (!client->wait_for_service(2s)) {
    std::cerr << "GetCostmap service unavailable\n";
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  std::mutex ready_mutex;
  std::condition_variable ready_cv;
  bool center_written = false;
  std::thread writer([&]() {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap.getMutex());
      costmap.setCost(1, 1, 254);
      {
        std::lock_guard<std::mutex> ready_lock(ready_mutex);
        center_written = true;
      }
      ready_cv.notify_one();
      std::this_thread::sleep_for(1000ms);
      costmap.setCost(2, 1, 253);
    });

  {
    std::unique_lock<std::mutex> lock(ready_mutex);
    ready_cv.wait(lock, [&center_written]() {return center_written;});
  }

  auto started = std::chrono::steady_clock::now();
  auto future = client->async_send_request(std::make_shared<nav2_msgs::srv::GetCostmap::Request>());
  bool returned_while_locked = future.wait_for(400ms) == std::future_status::ready;
  if (!returned_while_locked && future.wait_for(2s) != std::future_status::ready) {
    std::cerr << "GetCostmap response timeout\n";
    writer.join();
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  auto response = future.get();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - started).count();
  writer.join();
  const auto center = response->map.data.at(5);
  const auto neighbor = response->map.data.at(6);
  std::string classification = "unexpected";
  if (returned_while_locked && center == 254 && neighbor == 0) {
    classification = "partial_response";
  } else if (!returned_while_locked && center == 254 && neighbor == 253) {
    classification = "atomic_response";
  }

  std::cout << "{\"classification\":\"" << classification << "\","
            << "\"returned_while_locked\":" << (returned_while_locked ? "true" : "false") << ","
            << "\"latency_ms\":" << elapsed << ","
            << "\"center\":" << static_cast<int>(center) << ","
            << "\"neighbor\":" << static_cast<int>(neighbor) << "}\n";

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return classification == "unexpected" ? 1 : 0;
}
