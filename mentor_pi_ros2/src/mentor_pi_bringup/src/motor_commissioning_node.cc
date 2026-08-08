// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/motor_commissioning_node.h"

#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mentor_pi_bringup/motor_commissioning_core.h"
#include "mentor_pi_interfaces/msg/controller_diagnostics.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/motor_command.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace mentor_pi_bringup {
namespace {

using ControllerDiagnostics = mentor_pi_interfaces::msg::ControllerDiagnostics;
using Heartbeat = mentor_pi_interfaces::msg::Heartbeat;
using MotorCommand = mentor_pi_interfaces::msg::MotorCommand;
using MotorState = mentor_pi_interfaces::msg::MotorState;

constexpr char kMotionAuthorizationTopic[] =
    "/mentor_pi/configuration/motion_authorization";
constexpr char kConfigurationSupervisorName[] = "configuration_supervisor";
constexpr char kConfigurationSupervisorNamespace[] = "/mentor_pi";
constexpr char kMotorCommandTopic[] = "/mentor_pi/motors/command";
constexpr char kMotorStateTopic[] = "/mentor_pi/motors/state";
constexpr char kHeartbeatTopic[] = "/mentor_pi/heartbeat";
constexpr char kDiagnosticsTopic[] = "/mentor_pi/diagnostics";

std::int64_t MonotonicNowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

rclcpp::NodeOptions MakeStrictOptions(const rclcpp::NodeOptions& options) {
  rclcpp::NodeOptions strict_options{options};
  strict_options.allow_undeclared_parameters(false);
  strict_options.automatically_declare_parameters_from_overrides(false);
  return strict_options;
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

rclcpp::QoS TransientMotionGateQos() {
  return rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
      .reliable()
      .transient_local();
}

class MotorCommissioningNode final : public rclcpp::Node,
                                     public MotorCommissioningStopControl {
 public:
  MotorCommissioningNode(const rclcpp::NodeOptions& options,
                         std::shared_ptr<MotorCommissioningOutcome> outcome,
                         const volatile std::sig_atomic_t* stop_requested)
      : rclcpp::Node("motor_commissioning", "/mentor_pi",
                     MakeStrictOptions(options)),
        outcome_(std::move(outcome)),
        stop_requested_(stop_requested) {
    const std::string acknowledgement =
        declare_parameter<std::string>("acknowledgement", "");
    const std::int64_t motor_id =
        declare_parameter<std::int64_t>("motor_id", 0);
    const double target_rps = declare_parameter<double>("target_rps", 0.0);
    const std::int64_t duration_ms =
        declare_parameter<std::int64_t>("duration_ms", 0);
    configuration_ =
        MakeConfiguration(acknowledgement, motor_id, target_rps, duration_ms);
    core_ = std::make_unique<MotorCommissioningCore>(configuration_);

    parameter_callback_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>&) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason =
              "motor commissioning parameters are immutable after startup";
          return result;
        });

    command_publisher_ = create_publisher<MotorCommand>(
        kMotorCommandTopic, BestEffortDepthOneQos());
    CreateSubscriptions();
    timer_ = create_wall_timer(
        std::chrono::milliseconds{kMotorCommissioningPeriodMs},
        [this]() { RunCycle(); });

    RCLCPP_WARN(
        get_logger(),
        "guarded motor commissioning armed: session not yet locked, motor=%u "
        "target=%.6f RPS duration=%" PRId64
        " ms; all four targets are explicitly updated at 20 Hz",
        static_cast<unsigned int>(configuration_.motor_id),
        static_cast<double>(configuration_.target_rps), duration_ms);
  }

  bool RequestStop() noexcept override {
    external_stop_requested_.store(true, std::memory_order_release);
    try {
      PublishCommand(MotorCommissioningCommand::kStop);
      return true;
    } catch (...) {
      return false;
    }
  }

 private:
  MotorCommissioningConfiguration MakeConfiguration(
      const std::string& acknowledgement, std::int64_t motor_id,
      double target_rps, std::int64_t duration_ms) const {
    if (motor_id < 1 || motor_id > 4) {
      throw std::invalid_argument("motor_id must be in [1, 4]");
    }
    if (!std::isfinite(target_rps) || std::fabs(target_rps) < 0.01 ||
        std::fabs(target_rps) > 0.25) {
      throw std::invalid_argument(
          "target_rps must be finite and have magnitude in [0.01, 0.25]");
    }
    if (duration_ms < 100 || duration_ms > 5000) {
      throw std::invalid_argument("duration_ms must be in [100, 5000]");
    }

    MotorCommissioningConfiguration configuration;
    configuration.acknowledgement = acknowledgement;
    configuration.motor_id = static_cast<std::uint8_t>(motor_id);
    configuration.target_rps = static_cast<float>(target_rps);
    configuration.duration_ms = static_cast<std::uint32_t>(duration_ms);
    configuration.start_time_ms = MonotonicNowMs();
    const MotorCommissioningFailure failure =
        ValidateMotorCommissioningConfiguration(configuration);
    if (failure != MotorCommissioningFailure::kNone) {
      throw std::invalid_argument(std::string("invalid commissioning input: ") +
                                  MotorCommissioningFailureName(failure));
    }
    return configuration;
  }

  void CreateSubscriptions() {
    motion_authorization_subscription_ =
        create_subscription<std_msgs::msg::UInt64>(
            kMotionAuthorizationTopic, TransientMotionGateQos(),
            [this](const std_msgs::msg::UInt64::SharedPtr message) {
              CommissioningMotionAuthorizationObservation observation;
              observation.arrival_time_ms = MonotonicNowMs();
              observation.configuration_generation =
                  static_cast<std::uint32_t>(message->data >> 32U);
              observation.agent_session_id = static_cast<std::uint32_t>(
                  message->data & UINT64_C(0xffffffff));
              core_->ObserveMotionAuthorization(observation);
            });
    heartbeat_subscription_ = create_subscription<Heartbeat>(
        kHeartbeatTopic, ReliableDepthOneQos(),
        [this](const Heartbeat::SharedPtr message) {
          CommissioningHeartbeatObservation observation;
          observation.arrival_time_ms = MonotonicNowMs();
          observation.sequence = message->sequence;
          observation.uptime_ms = message->uptime_ms;
          observation.agent_session_id = message->agent_session_id;
          observation.state = message->state;
          core_->ObserveHeartbeat(observation);
        });
    diagnostics_subscription_ = create_subscription<ControllerDiagnostics>(
        kDiagnosticsTopic, ReliableDepthOneQos(),
        [this](const ControllerDiagnostics::SharedPtr message) {
          CommissioningDiagnosticsObservation observation;
          observation.arrival_time_ms = MonotonicNowMs();
          observation.uptime_ms = message->uptime_ms;
          observation.session_generation = message->session_generation;
          observation.command_rejections = message->command_rejections;
          observation.motor_command_rejections =
              message->motor_command_rejections;
          observation.motor_lease_expiries = message->motor_lease_expiries;
          observation.motor_watchdog_trips = message->motor_watchdog_trips;
          observation.session_state = message->session_state;
          core_->ObserveDiagnostics(observation);
        });
    motor_state_subscription_ = create_subscription<MotorState>(
        kMotorStateTopic, BestEffortDepthOneQos(),
        [this](const MotorState::SharedPtr message) {
          CommissioningMotorStateObservation observation;
          observation.arrival_time_ms = MonotonicNowMs();
          observation.target_rps = message->target_rps;
          observation.measured_rps = message->measured_rps;
          observation.encoder_count = message->encoder_count;
          observation.watchdog_stop_mask = message->watchdog_stop_mask;
          core_->ObserveMotorState(observation);
        });
  }

  bool HasExclusiveConfigurationSupervisor() {
    const auto publisher_information =
        get_publishers_info_by_topic(kMotionAuthorizationTopic);
    if (publisher_information.size() != std::size_t{1}) {
      return false;
    }
    return publisher_information.front().node_name() ==
               kConfigurationSupervisorName &&
           publisher_information.front().node_namespace() ==
               kConfigurationSupervisorNamespace;
  }

  void RunCycle() {
    if (outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    const std::int64_t now_ms = MonotonicNowMs();
    core_->ObserveMotionAuthorizationPublisher(
        HasExclusiveConfigurationSupervisor());
    core_->ObserveCommandPublisherConflict(
        count_publishers(kMotorCommandTopic) != 1U);
    if ((stop_requested_ != nullptr && *stop_requested_ != 0) ||
        external_stop_requested_.load(std::memory_order_acquire)) {
      core_->RequestAbort(now_ms);
    }

    const MotorCommissioningAction action = core_->Tick(now_ms);
    if (action.command != MotorCommissioningCommand::kNone) {
      PublishCommand(action.command);
      core_->RecordCommandPublished(action.command, MonotonicNowMs());
    }
    if (core_->complete()) {
      Complete();
    }
  }

  void PublishCommand(MotorCommissioningCommand command) {
    MotorCommand message;
    message.update_mask = MotorCommand::ALL_MOTORS;
    message.target_rps.fill(0.0F);
    if (command == MotorCommissioningCommand::kDrive) {
      const std::size_t selected_index =
          static_cast<std::size_t>(configuration_.motor_id - 1U);
      message.target_rps[selected_index] = configuration_.target_rps;
    }
    command_publisher_->publish(message);
  }

  void Complete() {
    const MotorCommissioningSummary summary = core_->summary();
    timer_->cancel();
    const char* const outcome_text = summary.passed ? "PASS" : "FAIL";
    const char* const failure_text =
        MotorCommissioningFailureName(summary.failure);
    if (summary.passed) {
      RCLCPP_INFO(
          get_logger(),
          "motor commissioning %s: session=%" PRIu32
          " motor=%u target=%.6f duration_ms=%" PRIu32 " encoder_delta=%" PRId64
          " peak_abs_measured_rps=%.6f final_measured_rps=%.6f "
          "target_observed=%s physical_response_observed=%s "
          "zero_confirmed=%s commands=%" PRIu32 "/%" PRIu32 "/%" PRIu32
          " failure=%s",
          outcome_text, summary.agent_session_id,
          static_cast<unsigned int>(summary.motor_id),
          static_cast<double>(summary.target_rps), summary.duration_ms,
          summary.encoder_delta,
          static_cast<double>(summary.peak_absolute_measured_rps),
          static_cast<double>(summary.final_measured_rps),
          summary.target_observed ? "true" : "false",
          summary.physical_response_observed ? "true" : "false",
          summary.zero_confirmed ? "true" : "false", summary.pre_stop_commands,
          summary.drive_commands, summary.post_stop_commands, failure_text);
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "motor commissioning %s: session=%" PRIu32
          " motor=%u target=%.6f duration_ms=%" PRIu32 " encoder_delta=%" PRId64
          " peak_abs_measured_rps=%.6f final_measured_rps=%.6f "
          "target_observed=%s physical_response_observed=%s "
          "zero_confirmed=%s commands=%" PRIu32 "/%" PRIu32 "/%" PRIu32
          " failure=%s",
          outcome_text, summary.agent_session_id,
          static_cast<unsigned int>(summary.motor_id),
          static_cast<double>(summary.target_rps), summary.duration_ms,
          summary.encoder_delta,
          static_cast<double>(summary.peak_absolute_measured_rps),
          static_cast<double>(summary.final_measured_rps),
          summary.target_observed ? "true" : "false",
          summary.physical_response_observed ? "true" : "false",
          summary.zero_confirmed ? "true" : "false", summary.pre_stop_commands,
          summary.drive_commands, summary.post_stop_commands, failure_text);
    }
    outcome_->failure.store(static_cast<std::uint8_t>(summary.failure),
                            std::memory_order_release);
    outcome_->passed.store(summary.passed, std::memory_order_release);
    outcome_->complete.store(true, std::memory_order_release);
  }

  MotorCommissioningConfiguration configuration_{};
  std::unique_ptr<MotorCommissioningCore> core_;
  std::shared_ptr<MotorCommissioningOutcome> outcome_;
  const volatile std::sig_atomic_t* stop_requested_ = nullptr;
  std::atomic<bool> external_stop_requested_{false};

  rclcpp::Publisher<MotorCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      motion_authorization_subscription_;
  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Subscription<ControllerDiagnostics>::SharedPtr
      diagnostics_subscription_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

MotorCommissioningInstance MakeMotorCommissioningNode(
    const rclcpp::NodeOptions& options,
    const volatile std::sig_atomic_t* stop_requested) {
  MotorCommissioningInstance instance;
  instance.outcome = std::make_shared<MotorCommissioningOutcome>();
  const auto node = std::make_shared<MotorCommissioningNode>(
      options, instance.outcome, stop_requested);
  instance.node = node;
  instance.stop_control = node;
  return instance;
}

}  // namespace mentor_pi_bringup
