#include <gtest/gtest.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits/joint_limits_rosparam.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transmission_interface/differential_transmission.hpp"
#include "transmission_interface/four_bar_linkage_transmission.hpp"

namespace
{
using Data = joint_limits::JointControlInterfacesData;
using Limiter = joint_limits::JointLimiterInterface<Data>;
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;

constexpr double kEpsilon = 1e-9;
constexpr char kRange[] = "joint_limits/JointInterfacesSaturationLimiter";
constexpr char kSoft[] = "joint_limits/JointInterfacesSoftLimiter";

void ensure_ros()
{
  if (!rclcpp::ok())
  {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

struct LimitResult
{
  bool enforced;
  Data desired;
};

LimitResult enforce_direct(
  const std::string & plugin, const joint_limits::JointLimits & limits, Data desired,
  const std::vector<joint_limits::SoftJointLimits> & soft = {})
{
  ensure_ros();
  pluginlib::ClassLoader<Limiter> loader(
    "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>");
  std::unique_ptr<Limiter> limiter(loader.createUnmanagedInstance(plugin));
  auto node = std::make_shared<rclcpp::Node>("direct_limit_probe");
  if (!limiter->init(
      {"joint"}, {limits}, soft, nullptr, node->get_node_logging_interface()))
  {
    throw std::runtime_error("limiter init failed");
  }
  Data current;
  current.joint_name = "joint";
  current.position = 0.0;
  current.velocity = 0.0;
  current.acceleration = 0.0;
  current.jerk = 0.0;
  current.effort = 0.0;
  if (!limiter->configure(current))
  {
    throw std::runtime_error("limiter configure failed");
  }
  desired.joint_name = "joint";
  return {limiter->enforce(current, desired, rclcpp::Duration::from_seconds(1.0)), desired};
}

struct ParameterLimiter
{
  pluginlib::ClassLoader<Limiter> loader{
    "joint_limits", "joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>"};
  std::unique_ptr<Limiter> limiter;
  rclcpp::Node::SharedPtr node;

  ParameterLimiter(const std::string & name, const std::vector<rclcpp::Parameter> & overrides)
  {
    ensure_ros();
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    node = std::make_shared<rclcpp::Node>(name, options);
    limiter.reset(loader.createUnmanagedInstance(kRange));
    if (!limiter->init({"joint"}, node))
    {
      throw std::runtime_error("parameter limiter init failed");
    }
    Data current;
    current.joint_name = "joint";
    current.position = 0.0;
    current.velocity = 0.0;
    if (!limiter->configure(current))
    {
      throw std::runtime_error("parameter limiter configure failed");
    }
  }

  std::pair<bool, Data> enforce(Data desired)
  {
    Data actual;
    actual.joint_name = "joint";
    actual.position = 0.0;
    actual.velocity = 0.0;
    desired.joint_name = "joint";
    return {
      limiter->enforce(actual, desired, rclcpp::Duration::from_seconds(1.0)),
      desired};
  }
};

TEST(JointTransmissionP0, EnabledLimitControls)
{
  joint_limits::JointLimits limits;
  limits.has_position_limits = true;
  limits.min_position = -1.0;
  limits.max_position = 1.0;
  limits.has_jerk_limits = true;
  limits.max_jerk = 0.5;
  Data desired;
  desired.position = 2.0;
  desired.jerk = 2.0;
  const auto range = enforce_direct(kRange, limits, desired);
  const auto soft = enforce_direct(kSoft, limits, desired);
  std::cout << "P3_OBSERVATION case=enabled_limit_controls range_pos="
            << range.desired.position.value() << " range_jerk=" << range.desired.jerk.value()
            << " soft_pos=" << soft.desired.position.value() << " soft_jerk="
            << soft.desired.jerk.value() << std::endl;
  EXPECT_TRUE(range.enforced);
  EXPECT_TRUE(soft.enforced);
  EXPECT_NEAR(range.desired.position.value(), 1.0, kEpsilon);
  EXPECT_NEAR(range.desired.jerk.value(), 0.5, kEpsilon);
  EXPECT_NEAR(soft.desired.position.value(), 1.0, kEpsilon);
  EXPECT_NEAR(soft.desired.jerk.value(), 0.5, kEpsilon);
}

TEST(JointTransmissionP0, DisabledPositionKeepsVelocityLimit)
{
  joint_limits::JointLimits limits;
  limits.has_position_limits = false;
  limits.min_position = -1.0;
  limits.max_position = 1.0;
  limits.has_velocity_limits = true;
  limits.max_velocity = 0.5;
  Data desired;
  desired.position = 2.0;
  const auto result = enforce_direct(kRange, limits, desired);
  std::cout << "P3_OBSERVATION case=disabled_position_keeps_velocity_limit enforced="
            << result.enforced << " position=" << result.desired.position.value() << std::endl;
  EXPECT_TRUE(result.enforced);
  EXPECT_NEAR(result.desired.position.value(), 0.5, kEpsilon);
}

TEST(JointTransmissionP0, SingleDynamicUpdate)
{
  ParameterLimiter fixture(
    "single_dynamic_update",
    {{"joint_limits.joint.has_velocity_limits", true},
     {"joint_limits.joint.max_velocity", 1.0},
     {"joint_limits.joint.has_effort_limits", true},
     {"joint_limits.joint.max_effort", 10.0}});
  const auto service = fixture.node->set_parameter(
    rclcpp::Parameter("joint_limits.joint.max_velocity", 2.0));
  Data desired;
  desired.velocity = 1.5;
  const auto [enforced, output] = fixture.enforce(desired);
  std::cout << "P3_OBSERVATION case=single_dynamic_update service=" << service.successful
            << " enforced=" << enforced << " velocity=" << output.velocity.value() << std::endl;
  EXPECT_TRUE(service.successful);
  EXPECT_FALSE(enforced);
  EXPECT_NEAR(output.velocity.value(), 1.5, kEpsilon);
}

TEST(JointTransmissionP0, AtomicChangeThenSame)
{
  ensure_ros();
  auto helper_node = std::make_shared<rclcpp::Node>("atomic_helper_probe");
  joint_limits::JointLimits updated;
  updated.has_velocity_limits = true;
  updated.max_velocity = 1.0;
  updated.max_effort = 10.0;
  const std::vector<rclcpp::Parameter> request{
    {"joint_limits.joint.max_velocity", 2.0},
    {"joint_limits.joint.max_effort", 10.0}};
  const bool helper_changed = joint_limits::check_for_limits_update(
    "joint", request, helper_node->get_node_logging_interface(), updated);

  ParameterLimiter fixture(
    "atomic_change_then_same",
    {{"joint_limits.joint.has_velocity_limits", true},
     {"joint_limits.joint.max_velocity", 1.0},
     {"joint_limits.joint.has_effort_limits", true},
     {"joint_limits.joint.max_effort", 10.0}});
  const auto service = fixture.node->set_parameters_atomically(request);
  Data desired;
  desired.velocity = 1.5;
  const auto [enforced, output] = fixture.enforce(desired);
  std::cout << "P3_OBSERVATION case=atomic_change_then_same helper_changed=" << helper_changed
            << " service=" << service.successful << " enforced=" << enforced << " velocity="
            << output.velocity.value() << std::endl;
  EXPECT_TRUE(service.successful);
  EXPECT_TRUE(helper_changed);
  EXPECT_FALSE(enforced);
  EXPECT_NEAR(output.velocity.value(), 1.5, kEpsilon);
}

TEST(JointTransmissionP0, DisablePositionRuntime)
{
  ensure_ros();
  auto helper_node = std::make_shared<rclcpp::Node>("disable_position_helper");
  joint_limits::JointLimits updated;
  updated.has_position_limits = true;
  updated.min_position = -1.0;
  updated.max_position = 1.0;
  const std::vector<rclcpp::Parameter> request{
    {"joint_limits.joint.has_position_limits", false}};
  const bool helper_changed = joint_limits::check_for_limits_update(
    "joint", request, helper_node->get_node_logging_interface(), updated);

  ParameterLimiter fixture(
    "disable_position_runtime",
    {{"joint_limits.joint.has_position_limits", true},
     {"joint_limits.joint.min_position", -1.0},
     {"joint_limits.joint.max_position", 1.0}});
  const auto service = fixture.node->set_parameters_atomically(request);
  Data desired;
  desired.position = 2.0;
  const auto [enforced, output] = fixture.enforce(desired);
  std::cout << "P3_OBSERVATION case=disable_position_runtime helper_changed=" << helper_changed
            << " service=" << service.successful << " enforced=" << enforced << " position="
            << output.position.value() << std::endl;
  EXPECT_TRUE(service.successful);
  EXPECT_TRUE(helper_changed);
  EXPECT_FALSE(enforced);
  EXPECT_NEAR(output.position.value(), 2.0, kEpsilon);
}

void expect_disabled_position_passthrough(const char * plugin, const char * case_name)
{
  joint_limits::JointLimits limits;
  limits.has_position_limits = false;
  limits.min_position = -1.0;
  limits.max_position = 1.0;
  Data desired;
  desired.position = 2.0;
  const auto result = enforce_direct(plugin, limits, desired);
  std::cout << "P3_OBSERVATION case=" << case_name << " enforced=" << result.enforced
            << " position=" << result.desired.position.value() << std::endl;
  EXPECT_FALSE(result.enforced);
  EXPECT_NEAR(result.desired.position.value(), 2.0, kEpsilon);
}

TEST(JointTransmissionP0, DisabledPositionRetainedRange)
{
  expect_disabled_position_passthrough(kRange, "disabled_position_retained_range");
}

TEST(JointTransmissionP0, DisabledPositionRetainedSoft)
{
  expect_disabled_position_passthrough(kSoft, "disabled_position_retained_soft");
}

void expect_disabled_jerk_passthrough(const char * plugin, const char * case_name)
{
  joint_limits::JointLimits limits;
  limits.has_jerk_limits = false;
  limits.max_jerk = 0.5;
  Data desired;
  desired.jerk = 2.0;
  const auto result = enforce_direct(plugin, limits, desired);
  std::cout << "P3_OBSERVATION case=" << case_name << " enforced=" << result.enforced
            << " jerk=" << result.desired.jerk.value() << std::endl;
  EXPECT_FALSE(result.enforced);
  EXPECT_NEAR(result.desired.jerk.value(), 2.0, kEpsilon);
}

TEST(JointTransmissionP0, DisabledJerkRetainedRange)
{
  expect_disabled_jerk_passthrough(kRange, "disabled_jerk_retained_range");
}

TEST(JointTransmissionP0, DisabledJerkRetainedSoft)
{
  expect_disabled_jerk_passthrough(kSoft, "disabled_jerk_retained_soft");
}

TEST(JointTransmissionP0, DifferentialFormulaPower)
{
  const std::vector<double> ar{2.0, -4.0}, jr{-3.0, 5.0}, offset{0.25, -0.75};
  double ap[2]{6.0, 8.0}, av[2]{2.0, -12.0}, ae[2]{7.0, -11.0};
  double jp[2]{}, jv[2]{}, je[2]{};
  transmission_interface::DifferentialTransmission transmission(ar, jr, offset);
  std::vector<JointHandle> joints;
  std::vector<ActuatorHandle> actuators;
  for (int i = 0; i < 2; ++i)
  {
    const auto suffix = std::to_string(i);
    joints.emplace_back("joint" + suffix, HW_IF_POSITION, &jp[i]);
    joints.emplace_back("joint" + suffix, HW_IF_VELOCITY, &jv[i]);
    joints.emplace_back("joint" + suffix, HW_IF_EFFORT, &je[i]);
    actuators.emplace_back("actuator" + suffix, HW_IF_POSITION, &ap[i]);
    actuators.emplace_back("actuator" + suffix, HW_IF_VELOCITY, &av[i]);
    actuators.emplace_back("actuator" + suffix, HW_IF_EFFORT, &ae[i]);
  }
  transmission.configure(joints, actuators);
  transmission.actuator_to_joint();
  const double expected_jp0 = (ap[0] / ar[0] + ap[1] / ar[1]) / (2.0 * jr[0]) + offset[0];
  const double expected_jp1 = (ap[0] / ar[0] - ap[1] / ar[1]) / (2.0 * jr[1]) + offset[1];
  const double expected_jv0 = (av[0] / ar[0] + av[1] / ar[1]) / (2.0 * jr[0]);
  const double expected_jv1 = (av[0] / ar[0] - av[1] / ar[1]) / (2.0 * jr[1]);
  const double expected_je0 = jr[0] * (ae[0] * ar[0] + ae[1] * ar[1]);
  const double expected_je1 = jr[1] * (ae[0] * ar[0] - ae[1] * ar[1]);
  const double actuator_power = ae[0] * av[0] + ae[1] * av[1];
  const double joint_power = je[0] * jv[0] + je[1] * jv[1];
  std::cout << "P3_OBSERVATION case=differential_formula_power position=" << jp[0] << ","
            << jp[1] << " velocity=" << jv[0] << "," << jv[1] << " effort=" << je[0]
            << "," << je[1] << " power_in=" << actuator_power << " power_out=" << joint_power
            << std::endl;
  EXPECT_NEAR(jp[0], expected_jp0, kEpsilon);
  EXPECT_NEAR(jp[1], expected_jp1, kEpsilon);
  EXPECT_NEAR(jv[0], expected_jv0, kEpsilon);
  EXPECT_NEAR(jv[1], expected_jv1, kEpsilon);
  EXPECT_NEAR(je[0], expected_je0, kEpsilon);
  EXPECT_NEAR(je[1], expected_je1, kEpsilon);
  EXPECT_NEAR(actuator_power, joint_power, kEpsilon);
}

TEST(JointTransmissionP0, FourBarFormulaPower)
{
  const std::vector<double> ar{2.0, -4.0}, jr{-3.0, 5.0}, offset{0.25, -0.75};
  double ap[2]{6.0, 8.0}, av[2]{2.0, -12.0}, ae[2]{7.0, -11.0};
  double jp[2]{}, jv[2]{}, je[2]{};
  transmission_interface::FourBarLinkageTransmission transmission(ar, jr, offset);
  std::vector<JointHandle> joints;
  std::vector<ActuatorHandle> actuators;
  for (int i = 0; i < 2; ++i)
  {
    const auto suffix = std::to_string(i);
    joints.emplace_back("joint" + suffix, HW_IF_POSITION, &jp[i]);
    joints.emplace_back("joint" + suffix, HW_IF_VELOCITY, &jv[i]);
    joints.emplace_back("joint" + suffix, HW_IF_EFFORT, &je[i]);
    actuators.emplace_back("actuator" + suffix, HW_IF_POSITION, &ap[i]);
    actuators.emplace_back("actuator" + suffix, HW_IF_VELOCITY, &av[i]);
    actuators.emplace_back("actuator" + suffix, HW_IF_EFFORT, &ae[i]);
  }
  transmission.configure(joints, actuators);
  transmission.actuator_to_joint();
  const double expected_jp0 = ap[0] / (jr[0] * ar[0]) + offset[0];
  const double expected_jp1 = (ap[1] / ar[1] - ap[0] / (jr[0] * ar[0])) / jr[1] + offset[1];
  const double expected_jv0 = av[0] / (jr[0] * ar[0]);
  const double expected_jv1 = (av[1] / ar[1] - av[0] / (jr[0] * ar[0])) / jr[1];
  const double expected_je0 = jr[0] * ae[0] * ar[0] + ae[1] * ar[1];
  const double expected_je1 = jr[1] * ae[1] * ar[1];
  const double actuator_power = ae[0] * av[0] + ae[1] * av[1];
  const double joint_power = je[0] * jv[0] + je[1] * jv[1];
  std::cout << "P3_OBSERVATION case=four_bar_formula_power position=" << jp[0] << "," << jp[1]
            << " velocity=" << jv[0] << "," << jv[1] << " effort=" << je[0] << "," << je[1]
            << " power_in=" << actuator_power << " power_out=" << joint_power << std::endl;
  EXPECT_NEAR(jp[0], expected_jp0, kEpsilon);
  EXPECT_NEAR(jp[1], expected_jp1, kEpsilon);
  EXPECT_NEAR(jv[0], expected_jv0, kEpsilon);
  EXPECT_NEAR(jv[1], expected_jv1, kEpsilon);
  EXPECT_NEAR(je[0], expected_je0, kEpsilon);
  EXPECT_NEAR(je[1], expected_je1, kEpsilon);
  EXPECT_NEAR(actuator_power, joint_power, kEpsilon);
}
}  // namespace
