// SPDX-License-Identifier: GPL-2.0-or-later

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "mentor_pi_tracking/tracker_node.hpp"
#include "mentor_pi_tracking/tracker_plugin.hpp"
#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

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
  message.header.frame_id = "map";
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
  request.bounded_configuration = AdrcConfiguration(VehicleType::kMecanum).mpc;
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

TEST(TrackerNodeContractTest, LadrcHoldsTerminalPoseForBothVehicles) {
  pluginlib::ClassLoader<TrackerPlugin> loader(
      "mentor_pi_tracking", "mentor_pi::tracking::TrackerPlugin");
  const auto trajectory = StraightTrajectory(0.25, 0.2);

  const auto mecanum =
      loader.createSharedInstance("mentor_pi_tracking/MecanumAdrc");
  mecanum->Configure(AdrcConfiguration(VehicleType::kMecanum));
  auto mecanum_request = AdrcRequest(trajectory, 1.0 / 30.0);
  mecanum_request.mpc.elapsed_seconds = trajectory->duration();
  mecanum_request.mpc.state = {{0.2, 0.0, 0.65}};
  const MpcCommand mecanum_at_endpoint = mecanum->Compute(mecanum_request);
  ASSERT_TRUE(mecanum_at_endpoint.solved);
  EXPECT_NEAR(mecanum_at_endpoint.linear_x, 0.0, 1.0e-12);
  EXPECT_NEAR(mecanum_at_endpoint.linear_y, 0.0, 1.0e-12);
  EXPECT_NEAR(mecanum_at_endpoint.angular_z, 0.0, 1.0e-12);

  const auto mecanum_behind =
      loader.createSharedInstance("mentor_pi_tracking/MecanumAdrc");
  mecanum_behind->Configure(AdrcConfiguration(VehicleType::kMecanum));
  mecanum_request.mpc.state = {{0.0, 0.0, 0.65}};
  const MpcCommand correction = mecanum_behind->Compute(mecanum_request);
  ASSERT_TRUE(correction.solved);
  EXPECT_GT(correction.linear_x, 0.0);

  const auto ackermann =
      loader.createSharedInstance("mentor_pi_tracking/AckermannAdrc");
  const auto ackermann_configuration =
      AdrcConfiguration(VehicleType::kAckermann);
  ackermann->Configure(ackermann_configuration);
  auto ackermann_request = AdrcRequest(trajectory, 1.0 / 30.0);
  ackermann_request.bounded_configuration = ackermann_configuration.mpc;
  ackermann_request.mpc.elapsed_seconds = trajectory->duration();
  ackermann_request.mpc.state = {{0.2, 0.0, -2.0}};
  const MpcCommand ackermann_at_endpoint =
      ackermann->Compute(ackermann_request);
  ASSERT_TRUE(ackermann_at_endpoint.solved);
  EXPECT_NEAR(ackermann_at_endpoint.linear_x, 0.0, 1.0e-12);
  EXPECT_NEAR(ackermann_at_endpoint.linear_y, 0.0, 1.0e-12);
  EXPECT_NEAR(ackermann_at_endpoint.angular_z, 0.0, 1.0e-12);
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
  first_request.bounded_configuration = configuration.mpc;
  second_request.bounded_configuration = configuration.mpc;
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
  aligned.bounded_configuration = configuration.mpc;
  quarter_turn.bounded_configuration = configuration.mpc;
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
  EXPECT_EQ(topics.count("/mentor_pi/trajectory_tracker/diagnostics"), 1U);
  EXPECT_EQ(topics.count("/mentor_pi/vehicle/reference"), 1U);
  EXPECT_EQ(topics.count("/mentor_pi/vehicle/pose"), 1U);
  EXPECT_EQ(topics.count("/mentor_pi/vehicle/odometry"), 0U);
  EXPECT_EQ(topics.count("/mentor_pi/motors/state"), 0U);
  EXPECT_EQ(topics.count("/mentor_pi/heartbeat"), 0U);
  EXPECT_EQ(topics.count("/mentor_pi/configuration/motion_authorization"), 0U);
  EXPECT_EQ(topics.count("/mentor_pi/diagnostics"), 0U);
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
  EXPECT_DOUBLE_EQ(
      tracker->get_parameter("driven_wheel_angular_speed_limit_rad_s")
          .as_double(),
      37.69911184307752);
}

TEST(TrackerNodeContractTest, MapPoseIsAlreadyTheGeometryCenterState) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 1.25;
  pose.pose.position.y = -0.75;
  pose.pose.orientation.z = std::sin(0.3);
  pose.pose.orientation.w = std::cos(0.3);

  const auto state = GeometryCenterPoseState(pose);
  ASSERT_TRUE(state.has_value());
  EXPECT_DOUBLE_EQ((*state)[0], 1.25);
  EXPECT_DOUBLE_EQ((*state)[1], -0.75);
  EXPECT_NEAR((*state)[2], 0.6, 1.0e-12);

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(GeometryCenterPoseState(pose).has_value());

  pose.pose.orientation.w = 1.0;
  pose.header.frame_id = "odom";
  EXPECT_FALSE(GeometryCenterPoseState(pose).has_value());
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
    EXPECT_EQ(expected_topics.count("/mentor_pi/vehicle/pose"), 1U);
    EXPECT_EQ(expected_topics.count("/mentor_pi/vehicle/odometry"), 0U);
    EXPECT_EQ(
        expected_topics.count("/mentor_pi/trajectory_tracker/diagnostics"), 1U);
    EXPECT_EQ(expected_topics.count("/mentor_pi/motors/state"), 0U);
    EXPECT_EQ(expected_topics.count("/mentor_pi/heartbeat"), 0U);
    EXPECT_EQ(
        expected_topics.count("/mentor_pi/configuration/motion_authorization"),
        0U);
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

  for (const double limit : {0.0, 37.69911184307752 + 1.0e-9,
                             std::numeric_limits<double>::quiet_NaN()}) {
    options = OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc");
    options.append_parameter_override("driven_wheel_angular_speed_limit_rad_s",
                                      limit);
    EXPECT_THROW(MakeTrackerNode(options), std::invalid_argument);
  }
}

TEST(TrackerNodeContractTest,
     LateTrajectoryActivatesImmediatelyAndStalePoseInhibitsOutput) {
  using namespace std::chrono_literals;
  const auto tracker = MakeTrackerNode(
      OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc"));
  const auto peer = std::make_shared<rclcpp::Node>("pose_expiry_peer");
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
  const auto pose_publisher =
      peer->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/mentor_pi/vehicle/pose", rclcpp::SensorDataQoS());
  (void)command_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(peer);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while (trajectory_publisher->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(trajectory_publisher->get_subscription_count(), 0U);

  mentor_pi_tracking_interfaces::msg::PolynomialTrajectory trajectory;
  trajectory.header.frame_id = "map";
  // Transport delay must not reject a synchronized absolute-time command.
  // A start in the recent past activates on the next tracker control tick.
  trajectory.header.stamp = peer->now() - rclcpp::Duration::from_seconds(0.1);
  trajectory.trajectory_id = "pose-expiry";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 5;
  segment.x_coefficients[1] = 0.1;
  trajectory.segments.push_back(segment);
  trajectory_publisher->publish(trajectory);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;

  bool nonzero = false;
  const auto active_deadline = std::chrono::steady_clock::now() + 2s;
  while (!nonzero && std::chrono::steady_clock::now() < active_deadline) {
    pose_publisher->publish(pose);
    executor.spin_some();
    nonzero = command != nullptr && command->twist.linear.x > 0.01;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(nonzero);

  command.reset();
  bool zero = false;
  const auto stale_deadline = std::chrono::steady_clock::now() + 1s;
  while (!zero && std::chrono::steady_clock::now() < stale_deadline) {
    executor.spin_some();
    zero = command != nullptr && command->twist.linear.x == 0.0 &&
           command->twist.linear.y == 0.0 && command->twist.angular.z == 0.0;
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(zero);
}

TEST(TrackerNodeContractTest,
     TerminalHoldSurvivesStalePoseUntilReplacementOrCancel) {
  using namespace std::chrono_literals;
  const auto tracker = MakeTrackerNode(
      OptionsFor("mecanum", "adrc", "mentor_pi_tracking/MecanumAdrc"));
  const auto peer = std::make_shared<rclcpp::Node>("terminal_hold_peer");
  geometry_msgs::msg::TwistStamped::SharedPtr command;
  std::size_t terminal_hold_diagnostics = 0U;
  const auto command_subscription =
      peer->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/mentor_pi/vehicle/reference", rclcpp::QoS(10).reliable(),
          [&command](geometry_msgs::msg::TwistStamped::SharedPtr message) {
            command = std::move(message);
          });
  const auto diagnostic_subscription =
      peer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
          "/mentor_pi/trajectory_tracker/diagnostics",
          rclcpp::QoS(10).reliable(),
          [&terminal_hold_diagnostics](
              const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
            for (const auto& status : message->status) {
              if (status.message.find("terminal hold active") !=
                  std::string::npos) {
                ++terminal_hold_diagnostics;
              }
            }
          });
  const auto trajectory_publisher = peer->create_publisher<
      mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>(
      "/mentor_pi/trajectory_tracker/reference_trajectory",
      rclcpp::QoS(1).reliable());
  const auto pose_publisher =
      peer->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/mentor_pi/vehicle/pose", rclcpp::SensorDataQoS());
  const auto cancel_client = peer->create_client<std_srvs::srv::Trigger>(
      "/mentor_pi/trajectory_tracker/cancel");
  (void)command_subscription;
  (void)diagnostic_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(peer);
  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while ((trajectory_publisher->get_subscription_count() == 0U ||
          !cancel_client->service_is_ready()) &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_GT(trajectory_publisher->get_subscription_count(), 0U);
  ASSERT_TRUE(cancel_client->service_is_ready());

  auto make_trajectory = [&peer](const std::string& id, double terminal_x) {
    mentor_pi_tracking_interfaces::msg::PolynomialTrajectory trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = peer->now() + rclcpp::Duration::from_seconds(0.3);
    trajectory.trajectory_id = id;
    mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
    segment.duration.nanosec = 200'000'000U;
    segment.x_coefficients[0] = terminal_x < 0.0 ? terminal_x : 0.0;
    segment.x_coefficients[1] = terminal_x < 0.0 ? 0.0 : terminal_x / 0.2;
    trajectory.segments.push_back(segment);
    return trajectory;
  };
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;

  trajectory_publisher->publish(make_trajectory("first-hold", 0.2));
  bool correcting_first_endpoint = false;
  const auto first_hold_deadline = std::chrono::steady_clock::now() + 2s;
  while (!correcting_first_endpoint &&
         std::chrono::steady_clock::now() < first_hold_deadline) {
    pose_publisher->publish(pose);
    executor.spin_some();
    correcting_first_endpoint = terminal_hold_diagnostics == 1U &&
                                command != nullptr &&
                                command->twist.linear.x > 0.01;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(correcting_first_endpoint);

  command.reset();
  bool stale_zero = false;
  const auto stale_deadline = std::chrono::steady_clock::now() + 1s;
  while (!stale_zero && std::chrono::steady_clock::now() < stale_deadline) {
    executor.spin_some();
    stale_zero = command != nullptr && command->twist.linear.x == 0.0 &&
                 command->twist.linear.y == 0.0 &&
                 command->twist.angular.z == 0.0;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(stale_zero);

  command.reset();
  bool resumed_hold = false;
  const auto resume_deadline = std::chrono::steady_clock::now() + 1s;
  while (!resumed_hold && std::chrono::steady_clock::now() < resume_deadline) {
    pose_publisher->publish(pose);
    executor.spin_some();
    resumed_hold = command != nullptr && command->twist.linear.x > 0.01;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(resumed_hold);
  EXPECT_EQ(terminal_hold_diagnostics, 1U);

  trajectory_publisher->publish(make_trajectory("replacement-hold", -0.2));
  command.reset();
  bool replacement_active = false;
  const auto replacement_deadline = std::chrono::steady_clock::now() + 2s;
  while (!replacement_active &&
         std::chrono::steady_clock::now() < replacement_deadline) {
    pose_publisher->publish(pose);
    executor.spin_some();
    replacement_active = terminal_hold_diagnostics == 2U &&
                         command != nullptr && command->twist.linear.x < -0.01;
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(replacement_active);

  command.reset();
  auto cancel_future = cancel_client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  const auto cancel_deadline = std::chrono::steady_clock::now() + 1s;
  while (cancel_future.wait_for(0ms) != std::future_status::ready &&
         std::chrono::steady_clock::now() < cancel_deadline) {
    pose_publisher->publish(pose);
    executor.spin_some();
    std::this_thread::sleep_for(5ms);
  }
  ASSERT_EQ(cancel_future.wait_for(0ms), std::future_status::ready);
  ASSERT_TRUE(cancel_future.get()->success);

  bool cancelled_zero = false;
  const auto zero_deadline = std::chrono::steady_clock::now() + 500ms;
  while (!cancelled_zero && std::chrono::steady_clock::now() < zero_deadline) {
    pose_publisher->publish(pose);
    executor.spin_some();
    cancelled_zero = command != nullptr && command->twist.linear.x == 0.0 &&
                     command->twist.linear.y == 0.0 &&
                     command->twist.angular.z == 0.0;
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_TRUE(cancelled_zero);
}

}  // namespace
}  // namespace mentor_pi::tracking
