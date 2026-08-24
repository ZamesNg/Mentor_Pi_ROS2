// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/motor_commissioning_node.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "mentor_pi_bringup/motor_commissioning_core.h"
#include "mentor_pi_interfaces/msg/controller_diagnostics.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/motor_command.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace {

using ControllerDiagnostics = mentor_pi_interfaces::msg::ControllerDiagnostics;
using Heartbeat = mentor_pi_interfaces::msg::Heartbeat;
using MotorCommand = mentor_pi_interfaces::msg::MotorCommand;
using MotorState = mentor_pi_interfaces::msg::MotorState;
using mentor_pi_bringup::MotorCommissioningFailure;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

rclcpp::QoS BestEffortDepthOneQos() {
  return rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
      .best_effort()
      .durability_volatile();
}

rclcpp::QoS ReliableDepthOneQos() {
  return rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
      .reliable()
      .durability_volatile();
}

template <typename Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor* executor,
               Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
  return predicate();
}

void SpinFor(rclcpp::executors::SingleThreadedExecutor* executor,
             std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }
}

class FakeCommissioningController final : public rclcpp::Node {
 public:
  explicit FakeCommissioningController(
      const std::string& authorization_node_name = "configuration_supervisor",
      const std::string& node_namespace = "/mentor_pi")
      : rclcpp::Node(authorization_node_name, node_namespace) {
    motion_authorization_publisher_ = create_publisher<std_msgs::msg::UInt64>(
        "configuration/motion_authorization",
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .transient_local());
    heartbeat_publisher_ = create_publisher<Heartbeat>("heartbeat",
                                                       ReliableDepthOneQos());
    diagnostics_publisher_ = create_publisher<ControllerDiagnostics>(
        "diagnostics", ReliableDepthOneQos());
    motor_state_publisher_ = create_publisher<MotorState>(
        "motors/state", BestEffortDepthOneQos());
    motor_command_subscription_ = create_subscription<MotorCommand>(
        "motors/command", BestEffortDepthOneQos(),
        [this](const MotorCommand::SharedPtr message) {
          ObserveMotorCommand(*message);
        });

    PublishAuthorization(1U, 42U);
    status_timer_ = create_wall_timer(std::chrono::milliseconds{100},
                                      [this]() { PublishStatus(); });
    motor_state_timer_ = create_wall_timer(std::chrono::milliseconds{20},
                                           [this]() { PublishMotorState(); });
  }

  bool command_policy_valid() const { return command_policy_valid_; }
  std::uint32_t drive_command_count() const { return drive_command_count_; }
  std::uint32_t stop_command_count() const { return stop_command_count_; }
  bool targets_are_zero() const {
    for (const float target : commanded_targets_) {
      if (target != 0.0F) {
        return false;
      }
    }
    return true;
  }

  void PublishAuthorization(std::uint32_t configuration_generation,
                            std::uint32_t session_id) {
    std_msgs::msg::UInt64 authorization;
    authorization.data =
        (static_cast<std::uint64_t>(configuration_generation) << 32U) |
        static_cast<std::uint64_t>(session_id);
    motion_authorization_publisher_->publish(authorization);
  }

 private:
  void ObserveMotorCommand(const MotorCommand& message) {
    if (message.update_mask != MotorCommand::ALL_MOTORS) {
      command_policy_valid_ = false;
    }
    for (std::size_t index = 0U; index < message.target_rps.size(); ++index) {
      if (!std::isfinite(message.target_rps[index]) ||
          (index != 1U && message.target_rps[index] != 0.0F)) {
        command_policy_valid_ = false;
      }
    }
    commanded_targets_ = message.target_rps;
    if (commanded_targets_[1] == 0.0F) {
      ++stop_command_count_;
    } else {
      ++drive_command_count_;
    }
  }

  void PublishStatus() {
    Heartbeat heartbeat;
    heartbeat.sequence = sequence_;
    ++sequence_;
    heartbeat.uptime_ms = sequence_ * 100U;
    heartbeat.agent_session_id = 42U;
    heartbeat.state = Heartbeat::READY;
    heartbeat_publisher_->publish(heartbeat);

    ControllerDiagnostics diagnostics;
    diagnostics.uptime_ms = heartbeat.uptime_ms;
    diagnostics.session_generation = 42U;
    diagnostics.session_state = ControllerDiagnostics::SESSION_ACTIVE;
    diagnostics_publisher_->publish(diagnostics);
  }

  void PublishMotorState() {
    MotorState state;
    state.target_rps = commanded_targets_;
    state.measured_rps[1] = commanded_targets_[1] * 0.8F;
    if (commanded_targets_[1] != 0.0F) {
      ++encoder_count_;
    }
    state.encoder_count[1] = encoder_count_;
    motor_state_publisher_->publish(state);
  }

  std::array<float, 4> commanded_targets_{};
  std::int64_t encoder_count_ = 0;
  std::uint32_t sequence_ = 0U;
  std::uint32_t drive_command_count_ = 0U;
  std::uint32_t stop_command_count_ = 0U;
  bool command_policy_valid_ = true;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
      motion_authorization_publisher_;
  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<ControllerDiagnostics>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<MotorState>::SharedPtr motor_state_publisher_;
  rclcpp::Subscription<MotorCommand>::SharedPtr motor_command_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr motor_state_timer_;
};

rclcpp::NodeOptions ValidOptions(
    std::int64_t duration_ms = 100,
    const std::string& node_namespace = "/mentor_pi") {
  rclcpp::NodeOptions options;
  if (node_namespace != "/mentor_pi") {
    options.arguments(
        {"--ros-args", "--remap", "__ns:=" + node_namespace});
  }
  options.parameter_overrides(
      {rclcpp::Parameter(
           "acknowledgement",
           std::string(mentor_pi_bringup::kMotorCommissioningAcknowledgement)),
       rclcpp::Parameter("motor_id", 2), rclcpp::Parameter("target_rps", 0.05),
       rclcpp::Parameter("duration_ms", duration_ms)});
  return options;
}

void TestInvalidAcknowledgementCreatesNoNode() {
  rclcpp::NodeOptions options = ValidOptions();
  options.parameter_overrides(
      {rclcpp::Parameter("acknowledgement", "MOTORS_RAISED"),
       rclcpp::Parameter("motor_id", 2), rclcpp::Parameter("target_rps", 0.05),
       rclcpp::Parameter("duration_ms", 100)});
  bool threw = false;
  try {
    static_cast<void>(mentor_pi_bringup::MakeMotorCommissioningNode(options));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  Expect(threw, "inexact runtime acknowledgement is rejected at startup");
}

void TestSuccessfulCommissioningAndCommandPolicy() {
  auto controller = std::make_shared<FakeCommissioningController>();
  const auto commissioning =
      mentor_pi_bringup::MakeMotorCommissioningNode(ValidOptions());
  const auto immutable_result =
      commissioning.node->set_parameter(rclcpp::Parameter("target_rps", 0.06));
  Expect(!immutable_result.successful,
         "commissioning parameters are immutable after startup");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(commissioning.node);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds{5};
  while (!commissioning.outcome->complete.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
  }

  Expect(commissioning.outcome->complete.load(std::memory_order_acquire),
         "commissioning node finishes within the bounded test time");
  Expect(commissioning.outcome->passed.load(std::memory_order_acquire),
         "controller acknowledgements produce a successful outcome");
  Expect(controller->command_policy_valid(),
         "every command uses ALL_MOTORS with only motor 2 nonzero");
  Expect(controller->drive_command_count() >= 2U,
         "100 ms drive emits repeated 20 Hz targets");
  Expect(controller->stop_command_count() >= 20U,
         "pre-stop and post-stop each emit a repeated zero burst");

  executor.remove_node(commissioning.node);
  executor.remove_node(controller);
}

void TestNamespaceOverrideScopesCommissioning() {
  constexpr char kNamespace[] = "/mecanum_1";
  auto controller = std::make_shared<FakeCommissioningController>(
      "configuration_supervisor", kNamespace);
  const auto commissioning = mentor_pi_bringup::MakeMotorCommissioningNode(
      ValidOptions(100, kNamespace));
  Expect(std::string{commissioning.node->get_namespace()} == kNamespace,
         "namespace remapping selects the mecanum_1 commissioning graph");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(commissioning.node);
  Expect(SpinUntil(
             &executor,
             [&commissioning]() {
               return commissioning.outcome->complete.load(
                   std::memory_order_acquire);
             },
             std::chrono::seconds{5}),
         "mecanum_1 commissioning finishes within the bounded test time");
  Expect(commissioning.outcome->passed.load(std::memory_order_acquire),
         "mecanum_1 supervisor identity and relative endpoints are accepted");
  Expect(controller->command_policy_valid(),
         "mecanum_1 commands retain the all-motor update policy");
  Expect(controller->drive_command_count() >= 2U,
         "mecanum_1 receives repeated drive commands");
  Expect(commissioning.node->count_publishers(
             "/mentor_pi/motors/command") == 0U,
         "the mecanum_1 override does not publish on the default motor topic");

  executor.remove_node(commissioning.node);
  executor.remove_node(controller);
}

void TestAuthorizationPublisherIdentityAndMultiplicity() {
  {
    auto wrong_controller =
        std::make_shared<FakeCommissioningController>("wrong_supervisor");
    const auto commissioning =
        mentor_pi_bringup::MakeMotorCommissioningNode(ValidOptions(5000));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(wrong_controller);
    executor.add_node(commissioning.node);
    SpinFor(&executor, std::chrono::milliseconds{750});
    Expect(wrong_controller->drive_command_count() == 0U,
           "a sole authorization publisher with the wrong node identity cannot "
           "start motion");
    static_cast<void>(commissioning.stop_control->RequestStop());
    Expect(SpinUntil(
               &executor,
               [&commissioning]() {
                 return commissioning.outcome->complete.load(
                     std::memory_order_acquire);
               },
               std::chrono::seconds{2}),
           "wrong-publisher refusal completes after an explicit stop request");
    executor.remove_node(commissioning.node);
    executor.remove_node(wrong_controller);
  }

  {
    auto controller = std::make_shared<FakeCommissioningController>();
    auto duplicate = std::make_shared<rclcpp::Node>(
        "duplicate_configuration_supervisor", "/mentor_pi");
    auto duplicate_publisher =
        duplicate->create_publisher<std_msgs::msg::UInt64>(
            "/mentor_pi/configuration/motion_authorization",
            rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
                .reliable()
                .transient_local());
    std_msgs::msg::UInt64 duplicate_authorization;
    duplicate_authorization.data = (UINT64_C(2) << 32U) | UINT64_C(42);
    duplicate_publisher->publish(duplicate_authorization);

    const auto commissioning =
        mentor_pi_bringup::MakeMotorCommissioningNode(ValidOptions(5000));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller);
    executor.add_node(duplicate);
    executor.add_node(commissioning.node);
    SpinFor(&executor, std::chrono::milliseconds{750});
    Expect(controller->drive_command_count() == 0U,
           "two authorization publishers fail closed regardless of retained "
           "value order");
    static_cast<void>(commissioning.stop_control->RequestStop());
    static_cast<void>(SpinUntil(
        &executor,
        [&commissioning]() {
          return commissioning.outcome->complete.load(
              std::memory_order_acquire);
        },
        std::chrono::seconds{2}));
    executor.remove_node(commissioning.node);
    executor.remove_node(duplicate);
    executor.remove_node(controller);
  }
}

void TestSessionBoundAuthorizationAndStopControl() {
  {
    auto controller = std::make_shared<FakeCommissioningController>();
    const auto commissioning =
        mentor_pi_bringup::MakeMotorCommissioningNode(ValidOptions(5000));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller);
    executor.add_node(commissioning.node);
    Expect(
        SpinUntil(
            &executor,
            [&controller]() { return controller->drive_command_count() > 0U; },
            std::chrono::seconds{3}),
        "long commissioning run reaches drive before authorization test");
    controller->PublishAuthorization(2U, 42U);
    Expect(SpinUntil(
               &executor,
               [&commissioning]() {
                 return commissioning.outcome->complete.load(
                     std::memory_order_acquire);
               },
               std::chrono::seconds{2}),
           "changed authorization generation produces a bounded stop outcome");
    Expect(commissioning.outcome->failure.load(std::memory_order_acquire) ==
               static_cast<std::uint8_t>(
                   MotorCommissioningFailure::kMotionAuthorizationLost),
           "authorization generation is locked for the complete run");
    Expect(controller->targets_are_zero(),
           "authorization loss leaves every target at zero");
    executor.remove_node(commissioning.node);
    executor.remove_node(controller);
  }

  {
    auto controller = std::make_shared<FakeCommissioningController>();
    const auto commissioning =
        mentor_pi_bringup::MakeMotorCommissioningNode(ValidOptions(5000));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller);
    executor.add_node(commissioning.node);
    Expect(
        SpinUntil(
            &executor,
            [&controller]() { return controller->drive_command_count() > 0U; },
            std::chrono::seconds{3}),
        "long commissioning run reaches drive before stop-control test");
    Expect(commissioning.stop_control->RequestStop(),
           "abnormal-exit stop control publishes an immediate zero");
    Expect(SpinUntil(
               &executor,
               [&commissioning]() {
                 return commissioning.outcome->complete.load(
                     std::memory_order_acquire);
               },
               std::chrono::seconds{2}),
           "stop control also requests the bounded post-stop state machine");
    Expect(
        commissioning.outcome->failure.load(std::memory_order_acquire) ==
            static_cast<std::uint8_t>(MotorCommissioningFailure::kInterrupted),
        "external stop control reports interruption");
    Expect(controller->targets_are_zero(),
           "external stop control leaves every target at zero");
    executor.remove_node(commissioning.node);
    executor.remove_node(controller);
  }
}

void TestCommandPublisherConflictFailsClosed() {
  auto controller = std::make_shared<FakeCommissioningController>();
  auto conflict_node =
      std::make_shared<rclcpp::Node>("conflicting_motor_publisher");
  const auto conflict_publisher = conflict_node->create_publisher<MotorCommand>(
      "/mentor_pi/motors/command", BestEffortDepthOneQos());
  static_cast<void>(conflict_publisher);
  const auto commissioning =
      mentor_pi_bringup::MakeMotorCommissioningNode(ValidOptions(5000));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(controller);
  executor.add_node(conflict_node);
  executor.add_node(commissioning.node);
  Expect(SpinUntil(
             &executor,
             [&commissioning]() {
               return commissioning.outcome->complete.load(
                   std::memory_order_acquire);
             },
             std::chrono::seconds{2}),
         "a second motor-command publisher produces a bounded refusal");
  Expect(controller->drive_command_count() == 0U,
         "publisher conflict never emits a nonzero command");
  Expect(commissioning.outcome->failure.load(std::memory_order_acquire) ==
             static_cast<std::uint8_t>(
                 MotorCommissioningFailure::kCommandPublisherConflict),
         "publisher conflict has an explicit outcome");
  executor.remove_node(commissioning.node);
  executor.remove_node(conflict_node);
  executor.remove_node(controller);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    TestInvalidAcknowledgementCreatesNoNode();
    TestSuccessfulCommissioningAndCommandPolicy();
    TestNamespaceOverrideScopesCommissioning();
    TestAuthorizationPublisherIdentityAndMultiplicity();
    TestSessionBoundAuthorizationAndStopControl();
    TestCommandPublisherConflictFailsClosed();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    ++g_failures;
  }
  rclcpp::shutdown();
  if (g_failures == 0) {
    std::cout << "motor commissioning ROS node tests passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
