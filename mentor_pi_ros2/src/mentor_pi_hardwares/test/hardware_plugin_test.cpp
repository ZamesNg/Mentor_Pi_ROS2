#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <memory>
#include <mentor_pi_interfaces/msg/heartbeat.hpp>
#include <mentor_pi_interfaces/msg/motor_command.hpp>
#include <mentor_pi_interfaces/msg/motor_state.hpp>
#include <mentor_pi_interfaces/msg/pwm_servo_command.hpp>
#include <mentor_pi_interfaces/msg/pwm_servo_state.hpp>
#include <optional>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <string>
#include <thread>
#include <vector>

namespace mentor_pi::test {
namespace {

using namespace std::chrono_literals;
using CallbackReturn = hardware_interface::CallbackReturn;
using ReturnType = hardware_interface::return_type;

hardware_interface::InterfaceInfo Interface(const std::string& name) {
  hardware_interface::InterfaceInfo interface;
  interface.name = name;
  return interface;
}

hardware_interface::ComponentInfo WheelJoint(const std::string& name) {
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

hardware_interface::HardwareInfo MecanumInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "mentor_pi_hardware";
  info.type = "system";
  info.hardware_class_type = "mentor_pi/MecanumHardware";
  info.hardware_parameters = {{"robot_name", "mentor_pi_test"},
                              {"feedback_timeout_ms", "100"}};
  info.joints = {WheelJoint("wheel_left_front_joint"),
                 WheelJoint("wheel_right_front_joint"),
                 WheelJoint("wheel_left_rear_joint"),
                 WheelJoint("wheel_right_rear_joint")};
  return info;
}

hardware_interface::HardwareInfo AckermannInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "mentor_pi_hardware";
  info.type = "system";
  info.hardware_class_type = "mentor_pi/AckermannHardware";
  info.hardware_parameters = {
      {"robot_name", "mentor_pi_test"},   {"feedback_timeout_ms", "100"},
      {"steering_pwm_channel", "3"},      {"steering_pwm_min_us", "500"},
      {"steering_pwm_center_us", "1500"}, {"steering_pwm_max_us", "2500"},
      {"steering_angle_min_rad", "-1.5"}, {"steering_angle_max_rad", "1.5"},
      {"steering_inverted", "true"},      {"steering_duration_ms", "20"}};
  info.joints = {SteeringJoint("wheel_left_front_joint"),
                 SteeringJoint("wheel_right_front_joint"),
                 WheelJoint("wheel_left_rear_joint"),
                 WheelJoint("wheel_right_rear_joint")};
  return info;
}

template <typename Predicate, typename Action>
bool WaitUntil(rclcpp::executors::SingleThreadedExecutor* executor,
               Predicate predicate, Action action,
               std::chrono::milliseconds timeout = 1000ms) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    action();
    executor->spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return false;
}

class HardwarePluginTest : public ::testing::Test {
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

TEST_F(HardwarePluginTest, MecanumMapsUnitsConnectorsAndSafetyZeros) {
  auto hardware = loader_.createSharedInstance("mentor_pi/MecanumHardware");
  ASSERT_EQ(hardware->on_init(MecanumInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  auto commands = hardware->export_command_interfaces();
  auto states = hardware->export_state_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  ASSERT_EQ(states.size(), 8U);

  auto node = std::make_shared<rclcpp::Node>("mecanum_hardware_test");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  auto state_publisher =
      node->create_publisher<mentor_pi_interfaces::msg::MotorState>(
          "/mentor_pi/motors/state", qos);
  std::optional<mentor_pi_interfaces::msg::MotorCommand> received;
  auto command_subscription =
      node->create_subscription<mentor_pi_interfaces::msg::MotorCommand>(
          "/mentor_pi/motors/command", qos,
          [&received](mentor_pi_interfaces::msg::MotorCommand::SharedPtr msg) {
            received = *msg;
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto supervisor =
      std::make_shared<rclcpp::Node>("configuration_supervisor", "/mentor_pi");
  auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/mentor_pi/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/mentor_pi/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  executor.add_node(supervisor);

  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  mentor_pi_interfaces::msg::MotorState feedback;
  feedback.motor_model = mentor_pi_interfaces::msg::MotorState::MODEL_JGA27;
  feedback.measured_rps = {1.0F, 2.0F, 3.0F, 4.0F};
  feedback.encoder_count = {1040, 2080, 3120, 4160};
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &states]() {
        if (hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)) !=
            ReturnType::OK) {
          return false;
        }
        return std::fabs(states[1].get_value()) > 1.0;
      },
      [&state_publisher, &feedback]() { state_publisher->publish(feedback); }));

  constexpr double kTwoPi = 6.28318530717958647692;
  EXPECT_NEAR(states[0].get_value(), kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[1].get_value(), kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[2].get_value(), 3.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[3].get_value(), 3.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[4].get_value(), 2.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[5].get_value(), 2.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[6].get_value(), 4.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[7].get_value(), 4.0 * kTwoPi, 1.0e-9);

  for (std::size_t index = 0U; index < commands.size(); ++index) {
    commands[index].set_value(static_cast<double>(index + 1U) * kTwoPi);
  }
  received.reset();
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::ERROR);
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 42U;
  heartbeat.sequence = 100U;
  heartbeat.uptime_ms = 1000U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  std_msgs::msg::UInt64 authorization;
  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(42);
  received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::OK;
      },
      [&heartbeat_publisher, &authorization_publisher, &heartbeat,
       &authorization]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
      }));
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->update_mask,
            mentor_pi_interfaces::msg::MotorCommand::ALL_MOTORS);
  EXPECT_EQ(received->target_rps,
            (std::array<float, 4U>{1.0F, 3.0F, 2.0F, 4.0F}));

  authorization.data = UINT64_C(42);
  received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::ERROR;
      },
      [&authorization_publisher, &authorization, &state_publisher,
       &feedback]() {
        authorization_publisher->publish(authorization);
        state_publisher->publish(feedback);
      }));
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(43);
  received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::ERROR;
      },
      [&authorization_publisher, &authorization, &state_publisher,
       &feedback]() {
        authorization_publisher->publish(authorization);
        state_publisher->publish(feedback);
      }));
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(42);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::OK;
      },
      [&authorization_publisher, &authorization, &state_publisher,
       &feedback]() {
        authorization_publisher->publish(authorization);
        state_publisher->publish(feedback);
      }));

  authorization.data = 0U;
  received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::ERROR;
      },
      [&authorization_publisher, &authorization]() {
        authorization_publisher->publish(authorization);
      }));
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(42);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::OK;
      },
      [&authorization_publisher, &authorization]() {
        authorization_publisher->publish(authorization);
      }));

  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  received.reset();
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::ERROR);
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  std::this_thread::sleep_for(110ms);
  received.reset();
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::ERROR);
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  received.reset();
  ASSERT_EQ(hardware->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));
  EXPECT_EQ(hardware->on_cleanup(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  static_cast<void>(command_subscription);
}

TEST_F(HardwarePluginTest, AckermannUsesRearConnectorsAndSteeringCalibration) {
  auto hardware = loader_.createSharedInstance("mentor_pi/AckermannHardware");
  ASSERT_EQ(hardware->on_init(AckermannInfo()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  auto commands = hardware->export_command_interfaces();
  auto states = hardware->export_state_interfaces();
  ASSERT_EQ(commands.size(), 4U);
  ASSERT_EQ(states.size(), 6U);

  auto node = std::make_shared<rclcpp::Node>("ackermann_hardware_test");
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  auto motor_state_publisher =
      node->create_publisher<mentor_pi_interfaces::msg::MotorState>(
          "/mentor_pi/motors/state", qos);
  auto pwm_state_publisher =
      node->create_publisher<mentor_pi_interfaces::msg::PwmServoState>(
          "/mentor_pi/pwm_servos/state", qos);
  std::optional<mentor_pi_interfaces::msg::MotorCommand> motor_received;
  std::optional<mentor_pi_interfaces::msg::PwmServoCommand> pwm_received;
  auto motor_subscription =
      node->create_subscription<mentor_pi_interfaces::msg::MotorCommand>(
          "/mentor_pi/motors/command", qos,
          [&motor_received](
              mentor_pi_interfaces::msg::MotorCommand::SharedPtr msg) {
            motor_received = *msg;
          });
  auto pwm_subscription =
      node->create_subscription<mentor_pi_interfaces::msg::PwmServoCommand>(
          "/mentor_pi/pwm_servos/command", qos,
          [&pwm_received](
              mentor_pi_interfaces::msg::PwmServoCommand::SharedPtr msg) {
            pwm_received = *msg;
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto supervisor =
      std::make_shared<rclcpp::Node>("configuration_supervisor", "/mentor_pi");
  auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/mentor_pi/heartbeat", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/mentor_pi/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  executor.add_node(supervisor);

  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  mentor_pi_interfaces::msg::MotorState motor_feedback;
  motor_feedback.motor_model =
      mentor_pi_interfaces::msg::MotorState::MODEL_JGA27;
  motor_feedback.measured_rps = {0.0F, 2.0F, 0.0F, 4.0F};
  motor_feedback.encoder_count = {0, 2080, 0, 4160};
  mentor_pi_interfaces::msg::PwmServoState pwm_feedback;
  pwm_feedback.output_pulse_width_us.fill(1500U);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &states]() {
        if (hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)) !=
            ReturnType::OK) {
          return false;
        }
        return std::fabs(states[3].get_value()) > 1.0;
      },
      [&motor_state_publisher, &pwm_state_publisher, &motor_feedback,
       &pwm_feedback]() {
        motor_state_publisher->publish(motor_feedback);
        pwm_state_publisher->publish(pwm_feedback);
      }));

  constexpr double kTwoPi = 6.28318530717958647692;
  commands[0].set_value(0.3);
  commands[1].set_value(0.3);
  commands[2].set_value(2.0 * kTwoPi);
  commands[3].set_value(4.0 * kTwoPi);
  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 84U;
  heartbeat.sequence = 200U;
  heartbeat.uptime_ms = 2000U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  std_msgs::msg::UInt64 authorization;
  authorization.data = (UINT64_C(2) << 32U) | UINT64_C(84);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::OK;
      },
      [&heartbeat_publisher, &authorization_publisher, &heartbeat,
       &authorization]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
      }));
  motor_received.reset();
  pwm_received.reset();
  ASSERT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&motor_received, &pwm_received]() {
        return motor_received.has_value() && pwm_received.has_value();
      },
      []() {}));
  EXPECT_EQ(motor_received->update_mask, 0x0aU);
  EXPECT_EQ(motor_received->target_rps,
            (std::array<float, 4U>{0.0F, 2.0F, 0.0F, 4.0F}));
  EXPECT_EQ(pwm_received->update_mask, 0x04U);
  EXPECT_EQ(pwm_received->pulse_width_us[2], 1300U);

  motor_received.reset();
  pwm_received.reset();
  ASSERT_EQ(hardware->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&motor_received, &pwm_received]() {
        return motor_received.has_value() && pwm_received.has_value();
      },
      []() {}));
  EXPECT_EQ(motor_received->target_rps, (std::array<float, 4U>{}));
  EXPECT_EQ(pwm_received->pulse_width_us[2], 1500U);
  EXPECT_EQ(hardware->on_cleanup(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  static_cast<void>(motor_subscription);
  static_cast<void>(pwm_subscription);
}

}  // namespace
}  // namespace mentor_pi::test
