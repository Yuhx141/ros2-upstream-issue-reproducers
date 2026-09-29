#include <gtest/gtest.h>

#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/simple_transmission.hpp"

namespace
{
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;

constexpr double kEpsilon = 1e-9;

TEST(JointTransmissionP0, SimpleFormulaPower)
{
  double ap = 3.0, av = 4.0, ae = 5.0, jp = 0.0, jv = 0.0, je = 0.0;
  transmission_interface::SimpleTransmission transmission(-2.0, 0.5);
  transmission.configure(
    {JointHandle("joint", HW_IF_POSITION, &jp), JointHandle("joint", HW_IF_VELOCITY, &jv),
     JointHandle("joint", HW_IF_EFFORT, &je)},
    {ActuatorHandle("actuator", HW_IF_POSITION, &ap),
     ActuatorHandle("actuator", HW_IF_VELOCITY, &av),
     ActuatorHandle("actuator", HW_IF_EFFORT, &ae)});
  transmission.actuator_to_joint();
  std::cout << "P3_OBSERVATION case=simple_formula_power position=" << jp
            << " velocity=" << jv << " effort=" << je << " power_in=" << ae * av
            << " power_out=" << je * jv << std::endl;
  EXPECT_NEAR(jp, -1.0, kEpsilon);
  EXPECT_NEAR(jv, -2.0, kEpsilon);
  EXPECT_NEAR(je, -10.0, kEpsilon);
  EXPECT_NEAR(ae * av, je * jv, kEpsilon);
}
}  // namespace
