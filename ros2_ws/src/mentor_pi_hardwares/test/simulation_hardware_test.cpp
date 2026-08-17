#include "mentor_pi_hardwares/simulation_hardware.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <string>

namespace mentor_pi::simulation {
namespace {

using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;

hardware_interface::InterfaceInfo Interface(const std::string& name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::ComponentInfo VelocityJoint(const std::string& name) {
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces = {Interface(hardware_interface::HW_IF_VELOCITY)};
  joint.state_interfaces = {Interface(hardware_interface::HW_IF_POSITION),
                            Interface(hardware_interface::HW_IF_VELOCITY)};
  return joint;
}

hardware_interface::ComponentInfo SteeringJoint(const std::string& name) {
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  joint.command_interfaces = {Interface(hardware_interface::HW_IF_POSITION)};
  joint.state_interfaces = {Interface(hardware_interface::HW_IF_POSITION)};
  return joint;
}

hardware_interface::HardwareInfo MecanumInfo(
    const std::string& robot_name = "mecanum_sim_test") {
  hardware_interface::HardwareInfo info;
  info.name = "mentor_pi_simulation";
  info.type = "system";
  info.hardware_class_type = "mentor_pi/MecanumSimulationHardware";
  info.hardware_parameters = {
      {"robot_name", robot_name},
      {"wheel_angular_speed_limit_rad_s", "2.0"},
      {"wheel_angular_acceleration_limit_rad_s2", "4.0"},
  };
  info.joints = {
      VelocityJoint("wheel_left_front_joint"),
      VelocityJoint("wheel_right_front_joint"),
      VelocityJoint("wheel_left_rear_joint"),
      VelocityJoint("wheel_right_rear_joint"),
  };
  return info;
}

hardware_interface::HardwareInfo AckermannInfo(
    const std::string& robot_name = "ackermann_sim_test") {
  hardware_interface::HardwareInfo info;
  info.name = "mentor_pi_simulation";
  info.type = "system";
  info.hardware_class_type = "mentor_pi/AckermannSimulationHardware";
  info.hardware_parameters = {
      {"robot_name", robot_name},
      {"wheel_angular_speed_limit_rad_s", "2.0"},
      {"wheel_angular_acceleration_limit_rad_s2", "4.0"},
      {"steering_angle_min_rad", "-0.6"},
      {"steering_angle_max_rad", "0.6"},
      {"steering_rate_limit_rad_s", "1.0"},
  };
  info.joints = {
      SteeringJoint("wheel_left_front_joint"),
      SteeringJoint("wheel_right_front_joint"),
      VelocityJoint("wheel_left_rear_joint"),
      VelocityJoint("wheel_right_rear_joint"),
  };
  return info;
}

class SimulationHardwareTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_{
      "hardware_interface", "hardware_interface::SystemInterface"};
};

TEST(SimulationActuatorTest, AppliesAccelerationAndTrapezoidalIntegration) {
  VelocityActuator actuator;
  actuator.target = 3.0;
  ASSERT_TRUE(StepVelocityActuator(&actuator, 0.25, 2.0, 4.0));
  EXPECT_DOUBLE_EQ(actuator.velocity, 1.0);
  EXPECT_DOUBLE_EQ(actuator.position, 0.125);

  ASSERT_TRUE(StepVelocityActuator(&actuator, 0.25, 2.0, 4.0));
  EXPECT_DOUBLE_EQ(actuator.velocity, 2.0);
  EXPECT_DOUBLE_EQ(actuator.position, 0.5);

  actuator.target = -3.0;
  ASSERT_TRUE(StepVelocityActuator(&actuator, 0.25, 2.0, 4.0));
  EXPECT_DOUBLE_EQ(actuator.velocity, 1.0);
  EXPECT_DOUBLE_EQ(actuator.position, 0.875);
  EXPECT_TRUE(StepVelocityActuator(&actuator, 0.0, 2.0, 4.0));
  EXPECT_DOUBLE_EQ(actuator.velocity, 1.0);
  EXPECT_DOUBLE_EQ(actuator.position, 0.875);
  EXPECT_FALSE(StepVelocityActuator(&actuator, -0.1, 2.0, 4.0));
}

TEST_F(SimulationHardwareTest, MecanumLimitsAllWheelsAndStopsSafely) {
  auto hardware =
      loader_.createSharedInstance("mentor_pi/MecanumSimulationHardware");
  ASSERT_EQ(hardware->on_init(MecanumInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  auto commands = hardware->export_command_interfaces();
  auto states = hardware->export_state_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  ASSERT_EQ(states.size(), 8U);
  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  commands[0].set_value(10.0);
  commands[1].set_value(-10.0);
  commands[2].set_value(1.0);
  commands[3].set_value(-1.0);
  ASSERT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 250'000'000)),
            ReturnType::OK);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 1.0);
  EXPECT_DOUBLE_EQ(states[3].get_value(), -1.0);
  EXPECT_DOUBLE_EQ(states[5].get_value(), 1.0);
  EXPECT_DOUBLE_EQ(states[7].get_value(), -1.0);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.125);
  EXPECT_DOUBLE_EQ(states[2].get_value(), -0.125);

  ASSERT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 250'000'000)),
            ReturnType::OK);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 2.0);
  EXPECT_DOUBLE_EQ(states[3].get_value(), -2.0);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.5);
  EXPECT_DOUBLE_EQ(states[2].get_value(), -0.5);

  const double position_before_stop = states[0].get_value();
  ASSERT_EQ(hardware->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(1, 0)),
            ReturnType::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), position_before_stop);
}

TEST_F(SimulationHardwareTest, AckermannModelsOnePhysicalSteeringState) {
  auto hardware =
      loader_.createSharedInstance("mentor_pi/AckermannSimulationHardware");
  ASSERT_EQ(hardware->on_init(AckermannInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  auto commands = hardware->export_command_interfaces();
  auto states = hardware->export_state_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  ASSERT_EQ(states.size(), 6U);
  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  commands[0].set_value(0.6);
  commands[1].set_value(0.2);
  commands[2].set_value(3.0);
  commands[3].set_value(-3.0);
  ASSERT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 100'000'000)),
            ReturnType::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.1);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 0.1);
  EXPECT_DOUBLE_EQ(states[3].get_value(), 0.4);
  EXPECT_DOUBLE_EQ(states[5].get_value(), -0.4);
  EXPECT_DOUBLE_EQ(states[2].get_value(), 0.02);
  EXPECT_DOUBLE_EQ(states[4].get_value(), -0.02);

  commands[0].set_value(1.2);
  commands[1].set_value(1.0);
  ASSERT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(1, 0)),
            ReturnType::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 0.6);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 0.6);
}

TEST_F(SimulationHardwareTest, NonfiniteCommandsAndPeriodsFailClosed) {
  auto mecanum =
      loader_.createSharedInstance("mentor_pi/MecanumSimulationHardware");
  ASSERT_EQ(mecanum->on_init(MecanumInfo("mecanum_invalid_command_test")),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(mecanum->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  auto commands = mecanum->export_command_interfaces();
  auto states = mecanum->export_state_interfaces();
  ASSERT_EQ(mecanum->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(mecanum->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::ERROR);
  EXPECT_DOUBLE_EQ(states[1].get_value(), 0.0);

  auto ackermann =
      loader_.createSharedInstance("mentor_pi/AckermannSimulationHardware");
  ASSERT_EQ(ackermann->on_init(AckermannInfo("ackermann_invalid_period_test")),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(ackermann->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(ackermann->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(ackermann->read(rclcpp::Time(0), rclcpp::Duration(-1, 0)),
            ReturnType::ERROR);
}

TEST_F(SimulationHardwareTest, InvalidConfigurationAndInterfacesAreRejected) {
  auto invalid_limit = MecanumInfo("mecanum_invalid_limit_test");
  invalid_limit.hardware_parameters["wheel_angular_speed_limit_rad_s"] = "0";
  auto mecanum =
      loader_.createSharedInstance("mentor_pi/MecanumSimulationHardware");
  EXPECT_EQ(mecanum->on_init(invalid_limit), CallbackReturn::ERROR);

  auto invalid_interface = AckermannInfo("ackermann_invalid_interface_test");
  invalid_interface.joints[0] = VelocityJoint("wheel_left_front_joint");
  auto ackermann =
      loader_.createSharedInstance("mentor_pi/AckermannSimulationHardware");
  EXPECT_EQ(ackermann->on_init(invalid_interface), CallbackReturn::ERROR);
}

}  // namespace
}  // namespace mentor_pi::simulation
