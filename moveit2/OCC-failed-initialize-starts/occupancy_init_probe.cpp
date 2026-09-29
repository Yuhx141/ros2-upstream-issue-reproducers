#include <moveit/occupancy_map_monitor/occupancy_map_monitor.hpp>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
struct State
{
  bool params_set{ false };
  bool init_result{ false };
  bool started{ false };
  std::vector<std::string> calls;
};

class FakeUpdater : public occupancy_map_monitor::OccupancyMapUpdater
{
public:
  FakeUpdater(std::shared_ptr<State> state, std::string arm)
    : OccupancyMapUpdater("P3FakeUpdater"), state_(std::move(state)), arm_(std::move(arm))
  {
  }

  bool setParams(const std::string&) override
  {
    state_->calls.emplace_back("set_params");
    state_->params_set = true;
    return true;
  }

  bool initialize(const rclcpp::Node::SharedPtr&) override
  {
    state_->calls.emplace_back("initialize");
    state_->init_result = arm_ == "always_success" || (arm_ == "requires_params" && state_->params_set);
    return state_->init_result;
  }

  void start() override
  {
    state_->calls.emplace_back("start");
    state_->started = true;
  }

  void stop() override
  {
    state_->calls.emplace_back("stop");
  }

  occupancy_map_monitor::ShapeHandle excludeShape(const shapes::ShapeConstPtr&) override
  {
    return 1;
  }

  void forgetShape(occupancy_map_monitor::ShapeHandle) override
  {
  }

private:
  std::shared_ptr<State> state_;
  std::string arm_;
};

class FakeMiddleware : public occupancy_map_monitor::OccupancyMapMonitor::MiddlewareHandle
{
public:
  FakeMiddleware(std::shared_ptr<State> state, std::string arm)
    : updater_(std::make_shared<FakeUpdater>(std::move(state), std::move(arm)))
  {
  }

  occupancy_map_monitor::OccupancyMapMonitor::Parameters getParameters() const override
  {
    return { 0.1, "", { { "p3_sensor", "p3_fake" } } };
  }

  occupancy_map_monitor::OccupancyMapUpdaterPtr loadOccupancyMapUpdater(const std::string&) override
  {
    return updater_;
  }

  void initializeOccupancyMapUpdater(occupancy_map_monitor::OccupancyMapUpdaterPtr updater) override
  {
    updater->initialize(nullptr);
  }

  void createSaveMapService(SaveMapServiceCallback) override
  {
  }

  void createLoadMapService(LoadMapServiceCallback) override
  {
  }

private:
  occupancy_map_monitor::OccupancyMapUpdaterPtr updater_;
};

std::string callsJson(const std::vector<std::string>& calls)
{
  std::string result = "[";
  for (std::size_t i = 0; i < calls.size(); ++i)
  {
    if (i)
      result += ',';
    result += "\"" + calls[i] + "\"";
  }
  return result + "]";
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2)
    return 2;
  rclcpp::init(argc, argv);
  auto state = std::make_shared<State>();
  {
    auto middleware = std::make_unique<FakeMiddleware>(state, argv[1]);
    occupancy_map_monitor::OccupancyMapMonitor monitor(std::move(middleware), nullptr);
    monitor.startMonitor();
    std::cout << "P3_RESULT {\"arm\":\"" << argv[1] << "\",\"params_set\":"
              << (state->params_set ? "true" : "false") << ",\"init_result\":"
              << (state->init_result ? "true" : "false") << ",\"started\":"
              << (state->started ? "true" : "false") << ",\"calls\":" << callsJson(state->calls) << "}\n";
  }
  rclcpp::shutdown();
  return 0;
}
