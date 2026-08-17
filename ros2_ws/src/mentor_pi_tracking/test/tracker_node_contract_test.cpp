// SPDX-License-Identifier: GPL-2.0-or-later

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "mentor_pi_tracking/tracker_node.hpp"
#include "mentor_pi_tracking/tracker_plugin.hpp"
#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace mentor_pi::tracking {
namespace {

class RclcppEnvironment final : public testing::Environment {
 public:
  void SetUp() override {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  void TearDown() override { rclcpp::shutdown(); }
};

const testing::Environment* const kEnvironment =
    testing::AddGlobalTestEnvironment(new RclcppEnvironment());

rclcpp::NodeOptions OptionsFor(const std::string& vehicle,
                               const std::string& algorithm,
                               const std::string& plugin) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("vehicle_type", vehicle);
  options.append_parameter_override("tracking_algorithm", algorithm);
  options.append_parameter_override("controller_plugin", plugin);
  return options;
}

std::shared_ptr<const PolynomialTrajectory> StraightTrajectory(
    double yaw = 0.0, double yaw_rate = 0.0) {
  mentor_pi_tracking_interfaces::msg::PolynomialTrajectory message;
  message.header.frame_id = "odom";
  message.trajectory_id = "adrc-test-" + std::to_string(yaw);
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 2;
  segment.x_coefficients[1] = 0.1;
  segment.yaw_coefficients[0] = yaw;
  segment.yaw_coefficients[1] = yaw_rate;
  message.segments.push_back(segment);
  std::string error;
  const auto trajectory = PolynomialTrajectory::FromMessage(message, &error);
  EXPECT_TRUE(trajectory.has_value()) << error;
  return std::make_shared<const PolynomialTrajectory>(*trajectory);
}

TrackerConfiguration AdrcConfiguration(VehicleType vehicle) {
  TrackerConfiguration configuration;
  configuration.mpc.vehicle = vehicle;
  configuration.mpc.wheelbase = 0.135;
  configuration.mpc.geometry_center_offset = 0.0675;
  configuration.mpc.mecanum_radius_sum = 0.14;
  configuration.mpc.max_linear_speed = 0.2;
  configuration.mpc.max_lateral_speed = 0.2;
  configuration.mpc.max_yaw_rate = 1.0;
  configuration.mpc.max_steering_angle = 0.6;
  return configuration;
}

TrackerRequest AdrcRequest(
    const std::shared_ptr<const PolynomialTrajectory>& trajectory,
    double period) {
  TrackerRequest request;
  request.trajectory = trajectory;
  request.mpc = {{0.0, 0.0, 0.0}, trajectory.get(), 0.0};
  request.live_configuration = AdrcConfiguration(VehicleType::kMecanum).mpc;
  request.measured_period_seconds = period;
  return request;
}

TEST(TrackerNodeContractTest, AllFourPluginsAreDiscoverable) {
  (void)kEnvironment;
  pluginlib::ClassLoader<TrackerPlugin> loader(
      "mentor_pi_tracking", "mentor_pi::tracking::TrackerPlugin");
  const std::array<std::string, 4> names{
      {"mentor_pi_tracking/MecanumMpc", "mentor_pi_tracking/AckermannMpc",
       "mentor_pi_tracking/MecanumAdrc", "mentor_pi_tracking/AckermannAdrc"}};
  for (const std::string& name : names) {
    EXPECT_TRUE(loader.isClassAvailable(name)) << name;
    EXPECT_NE(loader.createSharedInstance(name), nullptr) << name;
  }
}

TEST(TrackerNodeContractTest, LadrcUsesReferenceDerivativeAndRejectsActualDt) {
  pluginlib::ClassLoader<TrackerPlugin> loader(
      "mentor_pi_tracking", "mentor_pi::tracking::TrackerPlugin");
  const auto plugin =
      loader.createSharedInstance("mentor_pi_tracking/MecanumAdrc");
  plugin->Configure(AdrcConfiguration(VehicleType::kMecanum));
  const auto trajectory = StraightTrajectory();
  TrackerRequest request = AdrcRequest(trajectory, 1.0 / 30.0);
  const MpcCommand command = plugin->Compute(request);
  ASSERT_TRUE(command.solved);
  EXPECT_TRUE(command.detail.empty());
  EXPECT_NEAR(command.linear_x, 0.1, 1.0e-12);
  EXPECT_NEAR(command.linear_y, 0.0, 1.0e-12);
  plugin->SetAppliedCommand({true, 0.0, 0.0, 0.0, {}});
  EXPECT_TRUE(plugin->Compute(request).solved);

  request.measured_period_seconds = 0.2;  // wo * dt = 0.6 > 0.5.
  EXPECT_FALSE(plugin->Compute(request).solved);
}

TEST(TrackerNodeContractTest, AckermannLadrcIgnoresCommonYawPolynomial) {
  pluginlib::ClassLoader<TrackerPlugin> loader(
      "mentor_pi_tracking", "mentor_pi::tracking::TrackerPlugin");
  const auto first_plugin =
      loader.createSharedInstance("mentor_pi_tracking/AckermannAdrc");
  const auto second_plugin =
      loader.createSharedInstance("mentor_pi_tracking/AckermannAdrc");
  const auto configuration = AdrcConfiguration(VehicleType::kAckermann);
  first_plugin->Configure(configuration);
  second_plugin->Configure(configuration);
  auto first_request = AdrcRequest(StraightTrajectory(0.0, 0.0), 1.0 / 30.0);
  auto second_request = AdrcRequest(StraightTrajectory(2.0, -3.0), 1.0 / 30.0);
  first_request.live_configuration = configuration.mpc;
  second_request.live_configuration = configuration.mpc;
  const MpcCommand first = first_plugin->Compute(first_request);
  const MpcCommand second = second_plugin->Compute(second_request);
  ASSERT_TRUE(first.solved);
  ASSERT_TRUE(second.solved);
  EXPECT_DOUBLE_EQ(first.linear_x, second.linear_x);
  EXPECT_DOUBLE_EQ(first.linear_y, second.linear_y);
  EXPECT_DOUBLE_EQ(first.angular_z, second.angular_z);
}

TEST(TrackerNodeContractTest, AckermannLadrcUsesMeasuredYawAsKinematicState) {
  pluginlib::ClassLoader<TrackerPlugin> loader(
      "mentor_pi_tracking", "mentor_pi::tracking::TrackerPlugin");
  const auto first_plugin =
      loader.createSharedInstance("mentor_pi_tracking/AckermannAdrc");
  const auto second_plugin =
      loader.createSharedInstance("mentor_pi_tracking/AckermannAdrc");
  const auto configuration = AdrcConfiguration(VehicleType::kAckermann);
  first_plugin->Configure(configuration);
  second_plugin->Configure(configuration);
  const auto trajectory = StraightTrajectory();
  auto aligned = AdrcRequest(trajectory, 1.0 / 30.0);
  auto quarter_turn = aligned;
  aligned.live_configuration = configuration.mpc;
  quarter_turn.live_configuration = configuration.mpc;
  quarter_turn.mpc.state[2] = std::acos(-1.0) / 2.0;

  const MpcCommand aligned_command = first_plugin->Compute(aligned);
  const MpcCommand quarter_turn_command = second_plugin->Compute(quarter_turn);
  ASSERT_TRUE(aligned_command.solved);
  ASSERT_TRUE(quarter_turn_command.solved);
  EXPECT_GT(aligned_command.linear_x, 0.09);
  EXPECT_NEAR(aligned_command.angular_z, 0.0, 1.0e-12);
  EXPECT_NEAR(quarter_turn_command.linear_x, 0.0, 1.0e-12);
  EXPECT_LT(quarter_turn_command.angular_z, -1.0);
}

TEST(TrackerNodeContractTest, GenericNodeUsesOnlyGenericEndpoints) {
  const auto tracker = MakeTrackerNode(
      OptionsFor("mecanum", "mpc", "mentor_pi_tracking/MecanumMpc"));
  const auto topics = tracker->get_topic_names_and_types();
  EXPECT_EQ(topics.count("/mentor_pi/trajectory_tracker/reference_trajectory"),
            1U);
  EXPECT_EQ(topics.count("/mentor_pi/vehicle/reference"), 1U);
  EXPECT_EQ(topics.count("/mentor_pi/mecanum_drive_controller/reference"), 0U);
  EXPECT_EQ(topics.count("/mentor_pi/ackermann_steering_controller/reference"),
            0U);
  EXPECT_EQ(topics.count("/mentor_pi/mecanum_mpc_tracker/reference_trajectory"),
            0U);
  const auto services = tracker->get_service_names_and_types();
  EXPECT_EQ(services.count("/mentor_pi/trajectory_tracker/cancel"), 1U);
  EXPECT_EQ(services.count("/mentor_pi/mecanum_mpc_tracker/cancel"), 0U);
}

TEST(TrackerNodeContractTest, DefaultsToAdrc) {
  const auto tracker = MakeTrackerNode(rclcpp::NodeOptions());
  EXPECT_EQ(tracker->get_parameter("tracking_algorithm").as_string(), "adrc");
  EXPECT_EQ(tracker->get_parameter("controller_plugin").as_string(),
            "mentor_pi_tracking/MecanumAdrc");
}

TEST(TrackerNodeContractTest, OdometryPoseIsAlreadyTheGeometryCenterState) {
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = 1.25;
  odometry.pose.pose.position.y = -0.75;
  odometry.pose.pose.orientation.z = std::sin(0.3);
  odometry.pose.pose.orientation.w = std::cos(0.3);

  const auto state = GeometryCenterPoseState(odometry);
  ASSERT_TRUE(state.has_value());
  EXPECT_DOUBLE_EQ((*state)[0], 1.25);
  EXPECT_DOUBLE_EQ((*state)[1], -0.75);
  EXPECT_NEAR((*state)[2], 0.6, 1.0e-12);

  odometry.pose.pose.orientation.x = 0.0;
  odometry.pose.pose.orientation.y = 0.0;
  odometry.pose.pose.orientation.z = 0.0;
  odometry.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(GeometryCenterPoseState(odometry).has_value());
}

TEST(TrackerNodeContractTest, AllPluginsUseTheSameNodeAndEndpoints) {
  struct TrackerSpec {
    const char* vehicle;
    const char* algorithm;
    const char* plugin;
  };
  const std::array<TrackerSpec, 4> specs{{
      {"mecanum", "mpc", "mentor_pi_tracking/MecanumMpc"},
      {"mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc"},
      {"ackermann", "mpc", "mentor_pi_tracking/AckermannMpc"},
      {"ackermann", "adrc", "mentor_pi_tracking/AckermannAdrc"},
  }};

  auto baseline = MakeTrackerNode(
      OptionsFor(specs[0].vehicle, specs[0].algorithm, specs[0].plugin));
  const auto expected_topics = baseline->get_topic_names_and_types();
  const auto expected_services = baseline->get_service_names_and_types();
  baseline.reset();

  for (const auto& spec : specs) {
    const auto tracker =
        MakeTrackerNode(OptionsFor(spec.vehicle, spec.algorithm, spec.plugin));
    EXPECT_STREQ(tracker->get_name(), "trajectory_tracker");
    EXPECT_STREQ(tracker->get_namespace(), "/mentor_pi");
    EXPECT_EQ(tracker->get_topic_names_and_types(), expected_topics);
    EXPECT_EQ(tracker->get_service_names_and_types(), expected_services);
    EXPECT_EQ(expected_topics.count(
                  "/mentor_pi/trajectory_tracker/reference_trajectory"),
              1U);
    EXPECT_EQ(expected_topics.count("/mentor_pi/vehicle/reference"), 1U);
    EXPECT_EQ(expected_topics.count("/mentor_pi/vehicle/odometry"), 1U);
    EXPECT_EQ(expected_services.count("/mentor_pi/trajectory_tracker/cancel"),
              1U);
  }
}

TEST(TrackerNodeContractTest,
     RejectsMismatchedPluginAndUnsafeAdrcConfiguration) {
  EXPECT_THROW(MakeTrackerNode(OptionsFor("mecanum", "mpc",
                                          "mentor_pi_tracking/AckermannMpc")),
               std::invalid_argument);

  auto options =
      OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc");
  options.append_parameter_override("position_adrc_input_gain", 0.0);
  EXPECT_THROW(MakeTrackerNode(options), std::invalid_argument);

  options = OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc");
  options.append_parameter_override("position_adrc_controller_bandwidth_rad_s",
                                    4.0);
  options.append_parameter_override("position_adrc_observer_bandwidth_rad_s",
                                    3.0);
  EXPECT_THROW(MakeTrackerNode(options), std::invalid_argument);

  options = OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc");
  options.append_parameter_override("position_adrc_observer_bandwidth_rad_s",
                                    16.0);
  EXPECT_THROW(MakeTrackerNode(options), std::invalid_argument);
}

TEST(TrackerNodeContractTest, StaleHeartbeatInhibitsOutput) {
  using namespace std::chrono_literals;
  const auto tracker = MakeTrackerNode(
      OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc"));
  const auto peer = std::make_shared<rclcpp::Node>("heartbeat_expiry_peer");
  const auto supervisor =
      std::make_shared<rclcpp::Node>("configuration_supervisor", "/mentor_pi");
  geometry_msgs::msg::TwistStamped::SharedPtr command;
  const auto command_subscription =
      peer->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/mentor_pi/vehicle/reference", rclcpp::QoS(10).reliable(),
          [&command](geometry_msgs::msg::TwistStamped::SharedPtr message) {
            command = std::move(message);
          });
  const auto trajectory_publisher = peer->create_publisher<
      mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>(
      "/mentor_pi/trajectory_tracker/reference_trajectory",
      rclcpp::QoS(1).reliable());
  const auto odometry_publisher =
      peer->create_publisher<nav_msgs::msg::Odometry>(
          "/mentor_pi/vehicle/odometry", rclcpp::SensorDataQoS());
  const auto motor_publisher =
      peer->create_publisher<mentor_pi_interfaces::msg::MotorState>(
          "/mentor_pi/motors/state", rclcpp::SensorDataQoS());
  const auto authorization_publisher =
      supervisor->create_publisher<std_msgs::msg::UInt64>(
          "/mentor_pi/configuration/motion_authorization",
          rclcpp::QoS(1).reliable().transient_local());
  const auto heartbeat_publisher =
      supervisor->create_publisher<mentor_pi_interfaces::msg::Heartbeat>(
          "/mentor_pi/heartbeat", rclcpp::QoS(1).reliable());
  (void)command_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(peer);
  executor.add_node(supervisor);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (trajectory_publisher->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(trajectory_publisher->get_subscription_count(), 0U);

  mentor_pi_tracking_interfaces::msg::PolynomialTrajectory trajectory;
  trajectory.header.frame_id = "odom";
  trajectory.header.stamp = peer->now() + rclcpp::Duration::from_seconds(0.3);
  trajectory.trajectory_id = "heartbeat-expiry";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 5;
  segment.x_coefficients[1] = 0.1;
  trajectory.segments.push_back(segment);
  trajectory_publisher->publish(trajectory);
  std_msgs::msg::UInt64 authorization;
  authorization.data = (std::uint64_t{1} << 32U) | std::uint64_t{42};
  mentor_pi_interfaces::msg::Heartbeat heartbeat;
  heartbeat.agent_session_id = 42U;
  heartbeat.state = mentor_pi_interfaces::msg::Heartbeat::READY;
  mentor_pi_interfaces::msg::MotorState motor;
  motor.motor_model = mentor_pi_interfaces::msg::MotorState::MODEL_JGA27;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;

  bool nonzero = false;
  const auto active_deadline = std::chrono::steady_clock::now() + 2s;
  while (!nonzero && std::chrono::steady_clock::now() < active_deadline) {
    authorization_publisher->publish(authorization);
    heartbeat_publisher->publish(heartbeat);
    motor_publisher->publish(motor);
    odometry_publisher->publish(odometry);
    executor.spin_some();
    nonzero = command != nullptr && command->twist.linear.x > 0.01;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(nonzero);

  command.reset();
  bool zero = false;
  const auto stale_deadline = std::chrono::steady_clock::now() + 1800ms;
  while (!zero && std::chrono::steady_clock::now() < stale_deadline) {
    authorization_publisher->publish(authorization);
    motor_publisher->publish(motor);
    odometry_publisher->publish(odometry);
    executor.spin_some();
    zero = command != nullptr && command->twist.linear.x == 0.0 &&
           command->twist.linear.y == 0.0 && command->twist.angular.z == 0.0;
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(zero);
}

}  // namespace
}  // namespace mentor_pi::tracking
