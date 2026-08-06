// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_monitor_node.h"

#include <atomic>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "mentor_pi_interfaces/msg/controller_diagnostics.hpp"
#include "mentor_pi_interfaces/msg/motor_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

int g_failures = 0;

using ControllerDiagnostics = mentor_pi_interfaces::msg::ControllerDiagnostics;
using MotorCommand = mentor_pi_interfaces::msg::MotorCommand;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

void TestDefaultIsStrictAndImmutable() {
  const auto monitor = mentor_pi_bringup::MakeQualificationMonitorNode();
  Expect(!monitor.node->get_parameter("imu_characterization_mode").as_bool(),
         "IMU characterization mode defaults false");
  const auto result = monitor.node->set_parameter(
      rclcpp::Parameter("imu_characterization_mode", true));
  Expect(!result.successful,
         "IMU characterization mode is immutable after startup");
}

void TestExplicitCharacterizationOverride() {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("imu_characterization_mode", true)});
  const auto monitor = mentor_pi_bringup::MakeQualificationMonitorNode(options);
  Expect(monitor.node->get_parameter("imu_characterization_mode").as_bool(),
         "explicit IMU characterization mode override is applied");
  Expect(!monitor.outcome->complete.load(std::memory_order_acquire),
         "constructing the monitor does not claim evidence");
  Expect(!monitor.outcome->passed.load(std::memory_order_acquire),
         "constructing the monitor does not claim a pass");
}

template <typename Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor* executor,
               Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  executor->spin_some();
  return predicate();
}

void SpinFor(rclcpp::executors::SingleThreadedExecutor* executor,
             std::chrono::milliseconds duration) {
  static_cast<void>(SpinUntil(executor, []() { return false; }, duration));
}

void TestZeroCommandsWaitForCleanDiagnosticsBaseline() {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("publish_zero_motor_commands", true),
       rclcpp::Parameter("zero_command_rate_hz", 100.0)});
  const auto monitor = mentor_pi_bringup::MakeQualificationMonitorNode(options);
  const auto peer = std::make_shared<rclcpp::Node>("qualification_gate_peer");

  std::atomic<std::uint32_t> command_count{0U};
  const auto command_subscription = peer->create_subscription<MotorCommand>(
      "/mentor_pi/motors/command",
      rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
          .best_effort()
          .durability_volatile(),
      [&command_count](MotorCommand::ConstSharedPtr) {
        command_count.fetch_add(1U, std::memory_order_relaxed);
      });
  const auto diagnostics_publisher =
      peer->create_publisher<ControllerDiagnostics>(
          "/mentor_pi/diagnostics",
          rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
              .reliable()
              .durability_volatile());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(monitor.node);
  executor.add_node(peer);
  const bool graph_ready = SpinUntil(
      &executor,
      [&diagnostics_publisher, &peer]() {
        return diagnostics_publisher->get_subscription_count() == 1U &&
               peer->count_publishers("/mentor_pi/motors/command") == 1U;
      },
      std::chrono::seconds{2});
  Expect(graph_ready, "qualification baseline-gate test graph is discovered");

  SpinFor(&executor, std::chrono::milliseconds{150});
  Expect(command_count.load(std::memory_order_relaxed) == 0U,
         "zero-command timer publishes nothing before diagnostics baseline");

  ControllerDiagnostics baseline;
  baseline.session_generation = 42U;
  baseline.session_state = ControllerDiagnostics::SESSION_ACTIVE;
  diagnostics_publisher->publish(baseline);
  const bool command_received = SpinUntil(
      &executor,
      [&command_count]() {
        return command_count.load(std::memory_order_relaxed) > 0U;
      },
      std::chrono::seconds{2});
  Expect(command_received,
         "a clean zero diagnostics baseline opens the command timer gate");

  executor.remove_node(peer);
  executor.remove_node(monitor.node);
  static_cast<void>(command_subscription);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    TestDefaultIsStrictAndImmutable();
    TestExplicitCharacterizationOverride();
    TestZeroCommandsWaitForCleanDiagnosticsBaseline();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    ++g_failures;
  }
  rclcpp::shutdown();
  if (g_failures == 0) {
    std::cout << "qualification monitor ROS node tests passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
