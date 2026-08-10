// SPDX-License-Identifier: GPL-2.0-or-later

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "gtest/gtest.h"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "mentor_pi_tracking/tracker_node.hpp"
#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
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

void ExerciseTracker(VehicleType vehicle, const std::string& controller,
                     const std::string& tracker_name) {
  const auto tracker = MakeTrackerNode(vehicle, rclcpp::NodeOptions{});
  const auto peer = std::make_shared<rclcpp::Node>(tracker_name + "_peer");
  geometry_msgs::msg::TwistStamped::SharedPtr command;
  const auto command_subscription =
      peer->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/mentor_pi/" + controller + "/reference",
          rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
          [&command](geometry_msgs::msg::TwistStamped::SharedPtr message) {
            command = std::move(message);
          });
  const auto trajectory_publisher = peer->create_publisher<
      mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>(
      "/mentor_pi/" + tracker_name + "/reference_trajectory",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
  const auto odometry_publisher =
      peer->create_publisher<nav_msgs::msg::Odometry>(
          "/mentor_pi/" + controller + "/odometry", rclcpp::SensorDataQoS());
  const auto motor_publisher =
      peer->create_publisher<mentor_pi_interfaces::msg::MotorState>(
          "/mentor_pi/motors/state", rclcpp::SensorDataQoS());
  const auto authorization_publisher =
      peer->create_publisher<std_msgs::msg::UInt64>(
          "/mentor_pi/configuration/motion_authorization",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  const auto cancel_client = peer->create_client<std_srvs::srv::Trigger>(
      "/mentor_pi/" + tracker_name + "/cancel");
  (void)command_subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(peer);
  const auto discovery_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (trajectory_publisher->get_subscription_count() == 0U &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(trajectory_publisher->get_subscription_count(), 0U);

  mentor_pi_tracking_interfaces::msg::PolynomialTrajectory trajectory;
  trajectory.header.frame_id = "odom";
  trajectory.header.stamp = peer->now() + rclcpp::Duration::from_seconds(0.4);
  trajectory.trajectory_id = tracker_name + "-integration";
  mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
  segment.duration.sec = 2;
  segment.x_coefficients[1] = 0.1;
  trajectory.segments.push_back(segment);
  trajectory_publisher->publish(trajectory);

  std_msgs::msg::UInt64 authorization;
  authorization.data = 1U;
  mentor_pi_interfaces::msg::MotorState motor;
  motor.motor_model = mentor_pi_interfaces::msg::MotorState::MODEL_JGA27;
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  bool received_nonzero = false;
  const auto command_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!received_nonzero &&
         std::chrono::steady_clock::now() < command_deadline) {
    authorization_publisher->publish(authorization);
    motor_publisher->publish(motor);
    odometry_publisher->publish(odometry);
    executor.spin_some();
    received_nonzero = command != nullptr && command->twist.linear.x > 0.01;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received_nonzero);
  EXPECT_DOUBLE_EQ(command->twist.linear.y, 0.0);

  auto replacement = trajectory;
  replacement.header.stamp = peer->now() + rclcpp::Duration::from_seconds(0.4);
  replacement.trajectory_id = tracker_name + "-replacement";
  replacement.segments[0].x_coefficients[1] = -0.1;
  trajectory_publisher->publish(replacement);
  bool switched_early = false;
  const auto before_replacement =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (std::chrono::steady_clock::now() < before_replacement) {
    authorization_publisher->publish(authorization);
    motor_publisher->publish(motor);
    odometry_publisher->publish(odometry);
    executor.spin_some();
    switched_early = switched_early ||
                     (command != nullptr && command->twist.linear.x < -0.01);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(switched_early);

  bool received_replacement = false;
  const auto replacement_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!received_replacement &&
         std::chrono::steady_clock::now() < replacement_deadline) {
    authorization_publisher->publish(authorization);
    motor_publisher->publish(motor);
    odometry_publisher->publish(odometry);
    executor.spin_some();
    received_replacement =
        command != nullptr && command->twist.linear.x < -0.01;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received_replacement);

  command.reset();
  bool stale_zero = false;
  const auto stale_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
  while (!stale_zero && std::chrono::steady_clock::now() < stale_deadline) {
    authorization_publisher->publish(authorization);
    motor_publisher->publish(motor);
    executor.spin_some();
    stale_zero = command != nullptr && command->twist.linear.x == 0.0 &&
                 command->twist.linear.y == 0.0 &&
                 command->twist.angular.z == 0.0;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(stale_zero);

  received_nonzero = false;
  const auto recovery_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
  while (!received_nonzero &&
         std::chrono::steady_clock::now() < recovery_deadline) {
    motor_publisher->publish(motor);
    odometry_publisher->publish(odometry);
    executor.spin_some();
    received_nonzero =
        command != nullptr && std::abs(command->twist.linear.x) > 0.01;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(received_nonzero);

  ASSERT_TRUE(cancel_client->wait_for_service(std::chrono::seconds(1)));
  auto cancel_future = cancel_client->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  const auto cancel_result = executor.spin_until_future_complete(
      cancel_future, std::chrono::seconds(1));
  ASSERT_EQ(cancel_result, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(cancel_future.get()->success);
  command.reset();
  const auto zero_deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (command == nullptr &&
         std::chrono::steady_clock::now() < zero_deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_NE(command, nullptr);
  EXPECT_DOUBLE_EQ(command->twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command->twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command->twist.angular.z, 0.0);
}

TEST(TrackerNodeContractTest, MecanumExposesStableTopicsAndPublishesSafeZero) {
  (void)kEnvironment;
  const auto tracker =
      MakeTrackerNode(VehicleType::kMecanum, rclcpp::NodeOptions{});
  const auto observer = std::make_shared<rclcpp::Node>("tracking_observer");
  geometry_msgs::msg::TwistStamped::SharedPtr received;
  const auto subscription =
      observer->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/mentor_pi/mecanum_drive_controller/reference",
          rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
          [&received](geometry_msgs::msg::TwistStamped::SharedPtr message) {
            received = std::move(message);
          });
  (void)subscription;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(observer);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
  while (received == nullptr && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_NE(received, nullptr);
  EXPECT_DOUBLE_EQ(received->twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(received->twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(received->twist.angular.z, 0.0);

  const auto topics = tracker->get_topic_names_and_types();
  EXPECT_EQ(topics.count("/mentor_pi/mecanum_mpc_tracker/reference_trajectory"),
            1U);
  const auto services = tracker->get_service_names_and_types();
  EXPECT_EQ(services.count("/mentor_pi/mecanum_mpc_tracker/cancel"), 1U);
}

TEST(TrackerNodeContractTest, AckermannUsesAckermannControllerContract) {
  const auto tracker =
      MakeTrackerNode(VehicleType::kAckermann, rclcpp::NodeOptions{});
  const auto topics = tracker->get_topic_names_and_types();
  EXPECT_EQ(
      topics.count("/mentor_pi/ackermann_mpc_tracker/reference_trajectory"),
      1U);
  EXPECT_EQ(topics.count("/mentor_pi/ackermann_steering_controller/reference"),
            1U);
  const auto services = tracker->get_service_names_and_types();
  EXPECT_EQ(services.count("/mentor_pi/ackermann_mpc_tracker/cancel"), 1U);
}

TEST(TrackerNodeContractTest, TracksSyntheticMecanumInputsAndCancelsToZero) {
  ExerciseTracker(VehicleType::kMecanum, "mecanum_drive_controller",
                  "mecanum_mpc_tracker");
}

TEST(TrackerNodeContractTest, TracksSyntheticAckermannInputsAndCancelsToZero) {
  ExerciseTracker(VehicleType::kAckermann, "ackermann_steering_controller",
                  "ackermann_mpc_tracker");
}

}  // namespace
}  // namespace mentor_pi::tracking
