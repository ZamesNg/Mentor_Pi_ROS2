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
#include <mentor_pi_interfaces/msg/imu_state.hpp>
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

hardware_interface::HardwareInfo MecanumInfo(
    const std::string& robot_name = "mentor_pi_test") {
  hardware_interface::HardwareInfo info;
  info.name = "mentor_pi_hardware";
  info.type = "system";
  info.hardware_class_type = "mentor_pi/MecanumHardware";
  info.hardware_parameters = {{"robot_name", robot_name}};
  info.joints = {WheelJoint("wheel_left_front_joint"),
                 WheelJoint("wheel_right_front_joint"),
                 WheelJoint("wheel_left_rear_joint"),
                 WheelJoint("wheel_right_rear_joint")};
  return info;
}

hardware_interface::HardwareInfo AckermannInfo(
    const std::string& robot_name = "mentor_pi_test") {
  hardware_interface::HardwareInfo info;
  info.name = "mentor_pi_hardware";
  info.type = "system";
  info.hardware_class_type = "mentor_pi/AckermannHardware";
  info.hardware_parameters = {
      {"robot_name", robot_name},        {"steering_pwm_channel", "3"},
      {"steering_pwm_min_us", "500"},    {"steering_pwm_center_us", "1500"},
      {"steering_pwm_max_us", "2500"},   {"steering_angle_min_rad", "-0.6"},
      {"steering_angle_max_rad", "0.6"}, {"steering_inverted", "true"},
      {"steering_duration_ms", "20"}};
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

TEST_F(HardwarePluginTest, RejectsInvalidMeasurementFilterCutoffs) {
  auto mecanum_info = MecanumInfo("mecanum_invalid_lpf_test");
  mecanum_info.hardware_parameters["linear_adrc_measurement_lpf_cutoff_hz"] =
      "0.0";
  auto mecanum = loader_.createSharedInstance("mentor_pi/MecanumHardware");
  EXPECT_EQ(mecanum->on_init(mecanum_info), CallbackReturn::ERROR);

  mecanum_info = MecanumInfo("mecanum_invalid_yaw_lpf_test");
  mecanum_info.hardware_parameters["yaw_adrc_measurement_lpf_cutoff_hz"] =
      "inf";
  mecanum = loader_.createSharedInstance("mentor_pi/MecanumHardware");
  EXPECT_EQ(mecanum->on_init(mecanum_info), CallbackReturn::ERROR);

  auto ackermann_info = AckermannInfo("ackermann_invalid_lpf_test");
  ackermann_info.hardware_parameters["linear_adrc_measurement_lpf_cutoff_hz"] =
      "-1.0";
  auto ackermann = loader_.createSharedInstance("mentor_pi/AckermannHardware");
  EXPECT_EQ(ackermann->on_init(ackermann_info), CallbackReturn::ERROR);

  ackermann_info = AckermannInfo("ackermann_invalid_yaw_lpf_test");
  ackermann_info.hardware_parameters["yaw_adrc_measurement_lpf_cutoff_hz"] =
      "nan";
  ackermann = loader_.createSharedInstance("mentor_pi/AckermannHardware");
  EXPECT_EQ(ackermann->on_init(ackermann_info), CallbackReturn::ERROR);
}

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
          "/mentor_pi_test/motors/state", qos);
  auto imu_publisher =
      node->create_publisher<mentor_pi_interfaces::msg::ImuState>(
          "/mentor_pi_test/imu", qos);
  std::optional<mentor_pi_interfaces::msg::MotorCommand> received;
  auto command_subscription =
      node->create_subscription<mentor_pi_interfaces::msg::MotorCommand>(
          "/mentor_pi_test/motors/command", qos,
          [&received](mentor_pi_interfaces::msg::MotorCommand::SharedPtr msg) {
            received = *msg;
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto supervisor = std::make_shared<rclcpp::Node>("configuration_supervisor",
                                                   "/mentor_pi_test");
  auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/mentor_pi_test/heartbeat",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/mentor_pi_test/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  executor.add_node(supervisor);

  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  std::this_thread::sleep_for(110ms);
  received.reset();
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  mentor_pi_interfaces::msg::MotorState feedback;
  feedback.motor_model = mentor_pi_interfaces::msg::MotorState::MODEL_JGA27;
  feedback.measured_rps = {-1.0F, -2.0F, 3.0F, 4.0F};
  feedback.encoder_count = {-1040, -2080, 3120, 4160};
  mentor_pi_interfaces::msg::ImuState imu;
  imu.valid = true;
  imu.angular_velocity_rad_s.fill(0.0F);
  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 42U;
  heartbeat.sequence = 100U;
  heartbeat.uptime_ms = 1000U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  std_msgs::msg::UInt64 authorization;
  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(42);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &states]() {
        if (hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)) !=
            ReturnType::OK) {
          return false;
        }
        return std::fabs(states[1].get_value()) > 1.0;
      },
      [&heartbeat_publisher, &authorization_publisher, &state_publisher,
       &imu_publisher, &heartbeat, &authorization, &feedback, &imu]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
        state_publisher->publish(feedback);
        imu_publisher->publish(imu);
      }));

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
  const auto received_motion_command = [&received]() {
    return received.has_value() && received->target_rps[0] < 0.0F &&
           received->target_rps[1] < 0.0F && received->target_rps[2] > 0.0F &&
           received->target_rps[3] > 0.0F;
  };
  const auto received_zero_command = [&received]() {
    return received.has_value() &&
           received->target_rps == std::array<float, 4U>{};
  };
  const auto publish_state_and_write = [&]() {
    heartbeat_publisher->publish(heartbeat);
    authorization_publisher->publish(authorization);
    state_publisher->publish(feedback);
    imu_publisher->publish(imu);
    static_cast<void>(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)));
  };
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));
  EXPECT_EQ(received->update_mask,
            mentor_pi_interfaces::msg::MotorCommand::ALL_MOTORS);
  for (const float value : received->target_rps) {
    EXPECT_LE(std::fabs(value), 6.0F);
  }

  imu.valid = false;
  received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &received_zero_command]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
                   ReturnType::OK &&
               received_zero_command();
      },
      [&imu_publisher, &imu]() { imu_publisher->publish(imu); }));
  imu.valid = true;
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));

  authorization.data = UINT64_C(42);
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_zero_command, publish_state_and_write));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(43);
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_zero_command, publish_state_and_write));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(42);
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));

  authorization.data = 0U;
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_zero_command, publish_state_and_write));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));

  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(42);
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));

  heartbeat.agent_session_id = 43U;
  heartbeat.uptime_ms = 10U;
  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(43);
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_zero_command, publish_state_and_write));
  for (int iteration = 0; iteration < 5; ++iteration) {
    publish_state_and_write();
    executor.spin_some();
    EXPECT_TRUE(received_zero_command());
  }
  authorization.data = (UINT64_C(2) << 32U) | UINT64_C(43);
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));

  heartbeat.uptime_ms = 1U;
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_zero_command, publish_state_and_write));
  authorization.data = (UINT64_C(3) << 32U) | UINT64_C(43);
  heartbeat.uptime_ms = 2U;
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));

  std::array<double, 4U> frozen_positions{};
  for (std::size_t wheel = 0U; wheel < frozen_positions.size(); ++wheel) {
    frozen_positions[wheel] = states[wheel * 2U].get_value();
  }
  std::this_thread::sleep_for(110ms);
  received.reset();
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  for (std::size_t index = 1U; index < states.size(); index += 2U) {
    EXPECT_DOUBLE_EQ(states[index].get_value(), 0.0);
  }
  for (std::size_t wheel = 0U; wheel < frozen_positions.size(); ++wheel) {
    EXPECT_DOUBLE_EQ(states[wheel * 2U].get_value(), frozen_positions[wheel]);
  }
  ASSERT_TRUE(WaitUntil(&executor, received_zero_command, []() {}));
  received.reset();
  ASSERT_TRUE(
      WaitUntil(&executor, received_motion_command, publish_state_and_write));

  commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  received.reset();
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::ERROR);
  ASSERT_TRUE(WaitUntil(
      &executor, [&received]() { return received.has_value(); }, []() {}));
  EXPECT_EQ(received->target_rps, (std::array<float, 4U>{}));
  ASSERT_EQ(hardware->on_error(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware->on_cleanup(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  static_cast<void>(command_subscription);
}

TEST_F(HardwarePluginTest, MecanumRemainsInhibitedWithoutFeedback) {
  constexpr char kRobotName[] = "mecanum_settling_test";
  auto hardware = loader_.createSharedInstance("mentor_pi/MecanumHardware");
  ASSERT_EQ(hardware->on_init(MecanumInfo(kRobotName)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  auto supervisor = std::make_shared<rclcpp::Node>(
      "configuration_supervisor", "/" + std::string(kRobotName));
  auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/" + std::string(kRobotName) + "/heartbeat",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/" + std::string(kRobotName) + "/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(supervisor);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&heartbeat_publisher, &authorization_publisher]() {
        return heartbeat_publisher->get_subscription_count() == 1U &&
               authorization_publisher->get_subscription_count() == 1U;
      },
      []() {}));

  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 55U;
  heartbeat.sequence = 1U;
  heartbeat.uptime_ms = 1000U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  std_msgs::msg::UInt64 authorization;
  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(55);

  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 50ms) {
    heartbeat_publisher->publish(heartbeat);
    authorization_publisher->publish(authorization);
    executor.spin_some();
    EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
              ReturnType::OK);
    EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
              ReturnType::OK);
    std::this_thread::sleep_for(5ms);
  }

  std::this_thread::sleep_for(110ms);
  heartbeat_publisher->publish(heartbeat);
  authorization_publisher->publish(authorization);
  executor.spin_some();
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);

  EXPECT_EQ(hardware->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware->on_cleanup(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
}

TEST_F(HardwarePluginTest, AckermannRemainsInhibitedWithoutFeedback) {
  constexpr char kRobotName[] = "ackermann_settling_test";
  auto hardware = loader_.createSharedInstance("mentor_pi/AckermannHardware");
  ASSERT_EQ(hardware->on_init(AckermannInfo(kRobotName)),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  auto supervisor = std::make_shared<rclcpp::Node>(
      "configuration_supervisor", "/" + std::string(kRobotName));
  auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/" + std::string(kRobotName) + "/heartbeat",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/" + std::string(kRobotName) + "/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(supervisor);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&heartbeat_publisher, &authorization_publisher]() {
        return heartbeat_publisher->get_subscription_count() == 1U &&
               authorization_publisher->get_subscription_count() == 1U;
      },
      []() {}));

  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 66U;
  heartbeat.sequence = 1U;
  heartbeat.uptime_ms = 1000U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  std_msgs::msg::UInt64 authorization;
  authorization.data = (UINT64_C(1) << 32U) | UINT64_C(66);

  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 50ms) {
    heartbeat_publisher->publish(heartbeat);
    authorization_publisher->publish(authorization);
    executor.spin_some();
    EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
              ReturnType::OK);
    EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
              ReturnType::OK);
    std::this_thread::sleep_for(5ms);
  }

  std::this_thread::sleep_for(110ms);
  heartbeat_publisher->publish(heartbeat);
  authorization_publisher->publish(authorization);
  executor.spin_some();
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);

  EXPECT_EQ(hardware->on_deactivate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware->on_cleanup(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
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
          "/mentor_pi_test/motors/state", qos);
  auto pwm_state_publisher =
      node->create_publisher<mentor_pi_interfaces::msg::PwmServoState>(
          "/mentor_pi_test/pwm_servos/state", qos);
  auto imu_publisher =
      node->create_publisher<mentor_pi_interfaces::msg::ImuState>(
          "/mentor_pi_test/imu", qos);
  std::optional<mentor_pi_interfaces::msg::MotorCommand> motor_received;
  std::optional<mentor_pi_interfaces::msg::PwmServoCommand> pwm_received;
  auto motor_subscription =
      node->create_subscription<mentor_pi_interfaces::msg::MotorCommand>(
          "/mentor_pi_test/motors/command", qos,
          [&motor_received](
              mentor_pi_interfaces::msg::MotorCommand::SharedPtr msg) {
            motor_received = *msg;
          });
  auto pwm_subscription =
      node->create_subscription<mentor_pi_interfaces::msg::PwmServoCommand>(
          "/mentor_pi_test/pwm_servos/command", qos,
          [&pwm_received](
              mentor_pi_interfaces::msg::PwmServoCommand::SharedPtr msg) {
            pwm_received = *msg;
          });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto supervisor = std::make_shared<rclcpp::Node>("configuration_supervisor",
                                                   "/mentor_pi_test");
  auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/mentor_pi_test/heartbeat",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/mentor_pi_test/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  executor.add_node(supervisor);

  ASSERT_EQ(hardware->on_activate(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);

  std::this_thread::sleep_for(110ms);
  motor_received.reset();
  pwm_received.reset();
  EXPECT_EQ(hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  EXPECT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&motor_received, &pwm_received]() {
        return motor_received.has_value() && pwm_received.has_value();
      },
      []() {}));
  EXPECT_EQ(motor_received->target_rps, (std::array<float, 4U>{}));
  EXPECT_EQ(pwm_received->pulse_width_us[2], 1500U);

  mentor_pi_interfaces::msg::MotorState motor_feedback;
  motor_feedback.motor_model =
      mentor_pi_interfaces::msg::MotorState::MODEL_JGA27;
  motor_feedback.measured_rps = {0.0F, -2.0F, 0.0F, 4.0F};
  motor_feedback.encoder_count = {0, -2080, 0, 4160};
  mentor_pi_interfaces::msg::PwmServoState pwm_feedback;
  pwm_feedback.output_pulse_width_us.fill(1500U);
  mentor_pi_interfaces::msg::ImuState imu;
  imu.valid = true;
  imu.angular_velocity_rad_s.fill(0.0F);
  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 84U;
  heartbeat.sequence = 200U;
  heartbeat.uptime_ms = 2000U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  std_msgs::msg::UInt64 authorization;
  authorization.data = (UINT64_C(2) << 32U) | UINT64_C(84);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &states]() {
        if (hardware->read(rclcpp::Time(0), rclcpp::Duration(0, 1)) !=
            ReturnType::OK) {
          return false;
        }
        return std::fabs(states[3].get_value()) > 1.0;
      },
      [&heartbeat_publisher, &authorization_publisher, &motor_state_publisher,
       &pwm_state_publisher, &imu_publisher, &heartbeat, &authorization,
       &motor_feedback, &pwm_feedback, &imu]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
        motor_state_publisher->publish(motor_feedback);
        pwm_state_publisher->publish(pwm_feedback);
        imu_publisher->publish(imu);
      }));

  constexpr double kTwoPi = 6.28318530717958647692;
  EXPECT_NEAR(states[2].get_value(), 2.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[3].get_value(), 2.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[4].get_value(), 4.0 * kTwoPi, 1.0e-9);
  EXPECT_NEAR(states[5].get_value(), 4.0 * kTwoPi, 1.0e-9);
  commands[0].set_value(0.3);
  commands[1].set_value(0.3);
  commands[2].set_value(2.0 * kTwoPi);
  commands[3].set_value(4.0 * kTwoPi);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
               ReturnType::OK;
      },
      [&heartbeat_publisher, &authorization_publisher, &motor_state_publisher,
       &pwm_state_publisher, &imu_publisher, &heartbeat, &authorization,
       &motor_feedback, &pwm_feedback, &imu]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
        motor_state_publisher->publish(motor_feedback);
        pwm_state_publisher->publish(pwm_feedback);
        imu_publisher->publish(imu);
      }));
  motor_received.reset();
  pwm_received.reset();
  ASSERT_EQ(hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)),
            ReturnType::OK);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&motor_received, &pwm_received]() {
        return motor_received.has_value() && pwm_received.has_value() &&
               motor_received->target_rps[1] < 0.0F &&
               motor_received->target_rps[3] > 0.0F &&
               pwm_received->pulse_width_us[2] != 1500U;
      },
      [&heartbeat_publisher, &authorization_publisher, &motor_state_publisher,
       &pwm_state_publisher, &imu_publisher, &heartbeat, &authorization,
       &motor_feedback, &pwm_feedback, &imu, &hardware]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
        motor_state_publisher->publish(motor_feedback);
        pwm_state_publisher->publish(pwm_feedback);
        imu_publisher->publish(imu);
        static_cast<void>(
            hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)));
      }));
  EXPECT_EQ(motor_received->update_mask, 0x0aU);
  EXPECT_NEAR(motor_received->target_rps[1], -2.0F, 1.0e-5F);
  EXPECT_NEAR(motor_received->target_rps[3], 4.0F, 1.0e-5F);
  EXPECT_LE(std::fabs(motor_received->target_rps[1]), 6.0F);
  EXPECT_LE(std::fabs(motor_received->target_rps[3]), 6.0F);
  EXPECT_EQ(pwm_received->update_mask, 0x04U);
  EXPECT_NE(pwm_received->pulse_width_us[2], 1500U);

  motor_feedback.measured_rps.fill(0.0F);
  motor_received.reset();
  pwm_received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &motor_received, &pwm_received]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
                   ReturnType::OK &&
               motor_received.has_value() && pwm_received.has_value() &&
               pwm_received->pulse_width_us[2] != 1500U;
      },
      [&motor_state_publisher, &pwm_state_publisher, &imu_publisher,
       &motor_feedback, &pwm_feedback, &imu]() {
        motor_state_publisher->publish(motor_feedback);
        pwm_state_publisher->publish(pwm_feedback);
        imu_publisher->publish(imu);
      }));
  EXPECT_NE(pwm_received->pulse_width_us[2], 1500U);

  imu.valid = false;
  motor_received.reset();
  pwm_received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &motor_received, &pwm_received]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
                   ReturnType::OK &&
               motor_received.has_value() && pwm_received.has_value() &&
               motor_received->target_rps == std::array<float, 4U>{} &&
               pwm_received->pulse_width_us[2] == 1500U;
      },
      [&imu_publisher, &imu]() { imu_publisher->publish(imu); }));
  EXPECT_EQ(motor_received->target_rps, (std::array<float, 4U>{}));
  EXPECT_EQ(pwm_received->pulse_width_us[2], 1500U);

  imu.valid = true;
  motor_received.reset();
  pwm_received.reset();
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&hardware, &motor_received, &pwm_received]() {
        return hardware->write(rclcpp::Time(0), rclcpp::Duration(0, 1)) ==
                   ReturnType::OK &&
               motor_received.has_value() && pwm_received.has_value() &&
               motor_received->target_rps != std::array<float, 4U>{} &&
               pwm_received->pulse_width_us[2] != 1500U;
      },
      [&heartbeat_publisher, &authorization_publisher, &motor_state_publisher,
       &pwm_state_publisher, &imu_publisher, &heartbeat, &authorization,
       &motor_feedback, &pwm_feedback, &imu]() {
        heartbeat_publisher->publish(heartbeat);
        authorization_publisher->publish(authorization);
        motor_state_publisher->publish(motor_feedback);
        pwm_state_publisher->publish(pwm_feedback);
        imu_publisher->publish(imu);
      }));

  motor_received.reset();
  pwm_received.reset();
  ASSERT_EQ(hardware->on_error(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  ASSERT_TRUE(WaitUntil(
      &executor,
      [&motor_received, &pwm_received]() {
        return motor_received.has_value() && pwm_received.has_value();
      },
      []() {}));
  EXPECT_EQ(motor_received->target_rps, (std::array<float, 4U>{}));
  EXPECT_EQ(pwm_received->pulse_width_us[2], 1500U);
  ASSERT_EQ(hardware->on_configure(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware->on_cleanup(rclcpp_lifecycle::State()),
            CallbackReturn::SUCCESS);
  static_cast<void>(motor_subscription);
  static_cast<void>(pwm_subscription);
}

}  // namespace
}  // namespace mentor_pi::test
