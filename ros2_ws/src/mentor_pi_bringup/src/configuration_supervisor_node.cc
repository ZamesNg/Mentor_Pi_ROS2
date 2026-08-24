// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/configuration_supervisor_node.h"

#include <chrono>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mentor_pi_bringup/configuration.h"
#include "mentor_pi_bringup/supervisor_core.h"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/result.hpp"
#include "mentor_pi_interfaces/srv/set_battery_threshold.hpp"
#include "mentor_pi_interfaces/srv/set_motor_adrc.hpp"
#include "mentor_pi_interfaces/srv/set_motor_model.hpp"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace mentor_pi_bringup {
namespace {

using HeartbeatMessage = mentor_pi_interfaces::msg::Heartbeat;
using ResultMessage = mentor_pi_interfaces::msg::Result;
using SetBatteryThreshold = mentor_pi_interfaces::srv::SetBatteryThreshold;
using SetMotorModel = mentor_pi_interfaces::srv::SetMotorModel;
using SetMotorAdrc = mentor_pi_interfaces::srv::SetMotorAdrc;
using SetPwmServoOffsets = mentor_pi_interfaces::srv::SetPwmServoOffsets;
using BatteryClient = rclcpp::Client<SetBatteryThreshold>;
using MotorClient = rclcpp::Client<SetMotorModel>;
using MotorAdrcClient = rclcpp::Client<SetMotorAdrc>;
using PwmClient = rclcpp::Client<SetPwmServoOffsets>;

constexpr std::chrono::milliseconds kStateMachinePeriod{10};
constexpr std::chrono::milliseconds kGraphPollPeriod{100};
// Heartbeat is reliable at 2 Hz. Three missed periods constitute heartbeat
// disappearance for the host sequencing gate; the MCU motor lease remains the
// independent 200 ms safety mechanism.
constexpr std::chrono::milliseconds kHeartbeatStaleAfter{1500};

rclcpp::NodeOptions MakeStrictOptions(const rclcpp::NodeOptions& options) {
  rclcpp::NodeOptions strict_options{options};
  strict_options.allow_undeclared_parameters(false);
  strict_options.automatically_declare_parameters_from_overrides(true);
  return strict_options;
}

ConfigurationValue ConvertParameterValue(const rclcpp::ParameterValue& value) {
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_STRING:
      return value.get<std::string>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return value.get<std::int64_t>();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return value.get<std::vector<std::int64_t>>();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return value.get<std::vector<double>>();
    default:
      return std::monostate{};
  }
}

ConfigurationMap ReadParameterOverrides(rclcpp::Node& node) {
  ConfigurationMap configuration;
  const auto& overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  for (const auto& parameter : overrides) {
    configuration.emplace(parameter.first,
                          ConvertParameterValue(parameter.second));
  }
  return configuration;
}

HeartbeatState ConvertHeartbeatState(std::uint8_t state) {
  switch (state) {
    case HeartbeatMessage::BOOTING:
      return HeartbeatState::kBooting;
    case HeartbeatMessage::READY:
      return HeartbeatState::kReady;
    case HeartbeatMessage::DEGRADED:
      return HeartbeatState::kDegraded;
    case HeartbeatMessage::FAULT:
      return HeartbeatState::kFault;
    default:
      return HeartbeatState::kFault;
  }
}

ResultCode ConvertResultCode(std::uint8_t result) {
  return static_cast<ResultCode>(result);
}

struct PendingRosCall {
  RequestToken token{};
  bool sent = false;
  std::int64_t request_id = 0;
};

class ConfigurationSupervisorNode final : public rclcpp::Node {
 public:
  explicit ConfigurationSupervisorNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("configuration_supervisor", "/mentor_pi",
                     MakeStrictOptions(options)) {
    const auto validation =
        ValidateConfiguration(ReadParameterOverrides(*this));

    motion_gate_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "configuration/motion_enabled",
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .transient_local());
    motion_authorization_publisher_ = create_publisher<std_msgs::msg::UInt64>(
        "configuration/motion_authorization",
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .transient_local());
    PublishMotionGate(false, 0U, 0U);

    parameter_callback_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>&) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "deployment configuration is immutable after startup";
          return result;
        });

    if (!validation.ok) {
      RCLCPP_ERROR(get_logger(), "invalid deployment configuration: %s",
                   validation.error.c_str());
    } else {
      core_.emplace(validation.configuration);
      RCLCPP_INFO(
          get_logger(),
          "validated configuration: model=%s offsets=[%d,%d,%d,%d] "
          "battery_threshold=%u mV",
          MotorModelName(validation.configuration.motor_model),
          static_cast<int>(validation.configuration.pwm_servo_offsets_us[0]),
          static_cast<int>(validation.configuration.pwm_servo_offsets_us[1]),
          static_cast<int>(validation.configuration.pwm_servo_offsets_us[2]),
          static_cast<int>(validation.configuration.pwm_servo_offsets_us[3]),
          static_cast<unsigned int>(
              validation.configuration.battery_low_threshold_mv));
    }

    const auto service_qos = rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
                                 .reliable()
                                 .durability_volatile();
    const auto& service_qos_profile = service_qos.get_rmw_qos_profile();
    motor_client_ =
        create_client<SetMotorModel>("motors/set_model", service_qos_profile);
    motor_adrc_client_ =
        create_client<SetMotorAdrc>("motors/set_adrc", service_qos_profile);
    pwm_client_ = create_client<SetPwmServoOffsets>("pwm_servos/set_offsets",
                                                    service_qos_profile);
    battery_client_ = create_client<SetBatteryThreshold>(
        "battery/set_low_threshold", service_qos_profile);

    const auto heartbeat_qos = rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
                                   .reliable()
                                   .durability_volatile();
    heartbeat_subscription_ = create_subscription<HeartbeatMessage>(
        "heartbeat", heartbeat_qos,
        [this](HeartbeatMessage::ConstSharedPtr message) {
          HandleHeartbeat(*message);
        });

    state_machine_timer_ =
        create_wall_timer(kStateMachinePeriod, [this]() { RunStateMachine(); });
    graph_timer_ = create_wall_timer(kGraphPollPeriod,
                                     [this]() { CheckControllerPresence(); });
  }

 private:
  void HandleHeartbeat(const HeartbeatMessage& message) {
    const auto now = SupervisorCore::Clock::now();
    last_heartbeat_time_ = now;
    heartbeat_received_ = true;
    heartbeat_timed_out_ = false;
    if (!core_.has_value()) {
      return;
    }

    HeartbeatSample heartbeat;
    heartbeat.uptime_ms = message.uptime_ms;
    heartbeat.agent_session_id = message.agent_session_id;
    heartbeat.state = ConvertHeartbeatState(message.state);
    core_->OnHeartbeat(heartbeat, now);
    ReconcilePendingCall();
    PublishStatusIfChanged();
  }

  void CheckControllerPresence() {
    const bool graph_present =
        count_publishers("heartbeat") > std::size_t{0};
    const auto now = SupervisorCore::Clock::now();
    if (graph_present != graph_present_) {
      graph_present_ = graph_present;
      if (!graph_present_) {
        heartbeat_received_ = false;
        heartbeat_timed_out_ = false;
      }
      if (core_.has_value()) {
        core_->OnControllerPresence(graph_present_, now);
      }
    }

    if (graph_present_ && heartbeat_received_ && !heartbeat_timed_out_ &&
        now - last_heartbeat_time_ >= kHeartbeatStaleAfter) {
      heartbeat_timed_out_ = true;
      if (core_.has_value()) {
        core_->OnControllerPresence(false, now);
      }
    }

    ReconcilePendingCall();
    PublishStatusIfChanged();
  }

  void RunStateMachine() {
    if (!core_.has_value()) {
      return;
    }

    const auto call = core_->Tick(SupervisorCore::Clock::now());
    ReconcilePendingCall();
    if (call.has_value()) {
      if (pending_call_.has_value()) {
        RCLCPP_ERROR(get_logger(),
                     "internal error: issued a call while one is pending");
      } else {
        pending_call_ = PendingRosCall{call->token, false, 0};
      }
    }
    TryDispatchPendingCall();
    PublishStatusIfChanged();
  }

  void TryDispatchPendingCall() {
    if (!pending_call_.has_value() || pending_call_->sent ||
        !core_.has_value() || !core_->IsCurrentToken(pending_call_->token)) {
      return;
    }

    const RequestToken token = pending_call_->token;
    try {
      switch (token.operation) {
        case ApplyOperation::kMotorModel:
          DispatchMotorModel(token);
          return;
        case ApplyOperation::kMotorAdrc:
          DispatchMotorAdrc(token);
          return;
        case ApplyOperation::kPwmServoOffsets:
          DispatchPwmOffsets(token);
          return;
        case ApplyOperation::kBatteryThreshold:
          DispatchBatteryThreshold(token);
          return;
      }
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "failed to dispatch %s: %s",
                   ApplyOperationName(token.operation), error.what());
      pending_call_.reset();
      core_->OnServiceResult(token, ResultCode::kIoError, 0,
                             SupervisorCore::Clock::now());
    }
  }

  void DispatchMotorModel(const RequestToken& token) {
    if (!motor_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<SetMotorModel::Request>();
    const MotorModel requested_model = core_->configuration().motor_model;
    request->model = MotorModelWireValue(requested_model);
    const auto future = motor_client_->async_send_request(
        request, [this, token,
                  requested_model](MotorClient::SharedFuture response_future) {
          try {
            const auto response = response_future.get();
            if (response->result.code == ResultMessage::OK) {
              const auto profile_error = ValidateMotorProfileResponse(
                  requested_model, response->active_model,
                  response->ticks_per_revolution, response->max_rps);
              if (profile_error != MotorProfileResponseError::kNone) {
                RCLCPP_ERROR(
                    get_logger(),
                    "motors/set_model returned inconsistent OK profile: %s; "
                    "requested=%s, active=%u, ticks=%" PRIu32 ", max_rps=%g",
                    MotorProfileResponseErrorName(profile_error),
                    MotorModelName(requested_model),
                    static_cast<unsigned int>(response->active_model),
                    response->ticks_per_revolution,
                    static_cast<double>(response->max_rps));
                HandleServiceResponse(
                    token, ResultMessage::IO_ERROR,
                    static_cast<std::uint16_t>(profile_error));
                return;
              }
            }
            HandleServiceResponse(token, response->result.code,
                                  response->result.detail);
          } catch (const std::exception& error) {
            HandleServiceException(token, error);
          }
        });
    pending_call_->sent = true;
    pending_call_->request_id = future.request_id;
  }

  void DispatchPwmOffsets(const RequestToken& token) {
    if (!pwm_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<SetPwmServoOffsets::Request>();
    request->update_mask = std::uint8_t{0x0F};
    request->offset_us = core_->configuration().pwm_servo_offsets_us;
    const auto future = pwm_client_->async_send_request(
        request, [this, token](PwmClient::SharedFuture response_future) {
          try {
            const auto response = response_future.get();
            HandleServiceResponse(token, response->result.code,
                                  response->result.detail);
          } catch (const std::exception& error) {
            HandleServiceException(token, error);
          }
        });
    pending_call_->sent = true;
    pending_call_->request_id = future.request_id;
  }

  void DispatchMotorAdrc(const RequestToken& token) {
    if (!motor_adrc_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<SetMotorAdrc::Request>();
    request->update_mask = SetMotorAdrc::Request::ALL_MOTORS;
    request->known_velocity_decay_rate_s_inverse =
        core_->configuration().known_velocity_decay_rate_s_inverse;
    request->input_gain_rps_per_second_per_permille =
        core_->configuration().input_gain_rps_per_second_per_permille;
    request->controller_bandwidth_rad_s =
        core_->configuration().controller_bandwidth_rad_s;
    request->controller_fal_exponent =
        core_->configuration().controller_fal_exponent;
    request->controller_fal_threshold_rps =
        core_->configuration().controller_fal_threshold_rps;
    request->observer_bandwidth_rad_s =
        core_->configuration().observer_bandwidth_rad_s;
    request->observer_velocity_fal_exponent =
        core_->configuration().observer_velocity_fal_exponent;
    request->observer_disturbance_fal_exponent =
        core_->configuration().observer_disturbance_fal_exponent;
    request->observer_fal_threshold_rps =
        core_->configuration().observer_fal_threshold_rps;
    request->disturbance_leakage_s_inverse =
        core_->configuration().disturbance_leakage_s_inverse;
    request->disturbance_estimate_limit_rps_per_second =
        core_->configuration().disturbance_estimate_limit_rps_per_second;
    request->velocity_filter_new_weight =
        core_->configuration().velocity_filter_new_weight;
    request->positive_minimum_drive_permille =
        core_->configuration().positive_minimum_drive_permille;
    request->negative_minimum_drive_permille =
        core_->configuration().negative_minimum_drive_permille;
    const auto future = motor_adrc_client_->async_send_request(
        request, [this, token](MotorAdrcClient::SharedFuture response_future) {
          try {
            const auto response = response_future.get();
            const std::uint8_t expected_mask =
                response->result.code == ResultMessage::OK
                    ? SetMotorAdrc::Request::ALL_MOTORS
                    : std::uint8_t{0U};
            if (response->applied_mask != expected_mask) {
              RCLCPP_ERROR(
                  get_logger(),
                  "motors/set_adrc returned code=%u with applied_mask=%u",
                  static_cast<unsigned int>(response->result.code),
                  static_cast<unsigned int>(response->applied_mask));
              HandleServiceResponse(token, ResultMessage::IO_ERROR,
                                    response->applied_mask);
              return;
            }
            HandleServiceResponse(token, response->result.code,
                                  response->result.detail);
          } catch (const std::exception& error) {
            HandleServiceException(token, error);
          }
        });
    pending_call_->sent = true;
    pending_call_->request_id = future.request_id;
  }

  void DispatchBatteryThreshold(const RequestToken& token) {
    if (!battery_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<SetBatteryThreshold::Request>();
    request->threshold_mv = core_->configuration().battery_low_threshold_mv;
    const auto future = battery_client_->async_send_request(
        request, [this, token](BatteryClient::SharedFuture response_future) {
          try {
            const auto response = response_future.get();
            HandleServiceResponse(token, response->result.code,
                                  response->result.detail);
          } catch (const std::exception& error) {
            HandleServiceException(token, error);
          }
        });
    pending_call_->sent = true;
    pending_call_->request_id = future.request_id;
  }

  void HandleServiceResponse(const RequestToken& token, std::uint8_t code,
                             std::uint16_t detail) {
    if (!core_.has_value()) {
      return;
    }
    if (pending_call_.has_value() && pending_call_->token == token) {
      pending_call_.reset();
    }
    const auto disposition = core_->OnServiceResult(
        token, ConvertResultCode(code), detail, SupervisorCore::Clock::now());
    if (disposition == ResponseDisposition::kStale) {
      RCLCPP_WARN(get_logger(),
                  "ignored stale response for %s, generation=%" PRIu64
                  ", "
                  "session=%" PRIu32 ", attempt=%u",
                  ApplyOperationName(token.operation),
                  token.configuration_generation, token.agent_session_id,
                  static_cast<unsigned int>(token.attempt));
    }
    PublishStatusIfChanged();
  }

  void HandleServiceException(const RequestToken& token,
                              const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "service future for %s failed: %s",
                 ApplyOperationName(token.operation), error.what());
    if (!core_.has_value()) {
      return;
    }
    if (pending_call_.has_value() && pending_call_->token == token) {
      pending_call_.reset();
    }
    static_cast<void>(core_->OnServiceResult(token, ResultCode::kIoError, 0,
                                             SupervisorCore::Clock::now()));
    PublishStatusIfChanged();
  }

  void ReconcilePendingCall() {
    if (!pending_call_.has_value() || !core_.has_value() ||
        core_->IsCurrentToken(pending_call_->token)) {
      return;
    }
    if (pending_call_->sent) {
      RemovePendingRequest(*pending_call_);
    }
    pending_call_.reset();
  }

  void RemovePendingRequest(const PendingRosCall& call) {
    switch (call.token.operation) {
      case ApplyOperation::kMotorModel:
        static_cast<void>(
            motor_client_->remove_pending_request(call.request_id));
        return;
      case ApplyOperation::kMotorAdrc:
        static_cast<void>(
            motor_adrc_client_->remove_pending_request(call.request_id));
        return;
      case ApplyOperation::kPwmServoOffsets:
        static_cast<void>(pwm_client_->remove_pending_request(call.request_id));
        return;
      case ApplyOperation::kBatteryThreshold:
        static_cast<void>(
            battery_client_->remove_pending_request(call.request_id));
        return;
    }
  }

  void PublishMotionGate(bool enabled, std::uint64_t configuration_generation,
                         std::uint32_t agent_session_id) {
    std_msgs::msg::Bool message;
    message.data = enabled;
    motion_gate_publisher_->publish(message);

    std_msgs::msg::UInt64 authorization;
    if (enabled) {
      const std::uint64_t generation =
          static_cast<std::uint32_t>(configuration_generation);
      authorization.data =
          (generation << 32U) | static_cast<std::uint64_t>(agent_session_id);
    }
    motion_authorization_publisher_->publish(authorization);
  }

  void PublishStatusIfChanged() {
    if (!core_.has_value()) {
      return;
    }
    const SupervisorStatus& status = core_->status();
    const bool changed =
        !last_status_.has_value() || last_status_->phase != status.phase ||
        last_status_->motion_enabled != status.motion_enabled ||
        last_status_->configuration_generation !=
            status.configuration_generation ||
        last_status_->agent_session_id != status.agent_session_id ||
        last_status_->description != status.description;
    if (!changed) {
      return;
    }

    if (!last_status_.has_value() ||
        last_status_->motion_enabled != status.motion_enabled) {
      PublishMotionGate(status.motion_enabled, status.configuration_generation,
                        status.agent_session_id);
    }
    RCLCPP_INFO(get_logger(),
                "configuration state=%s gate=%s generation=%" PRIu64
                " session=%" PRIu32 ": %s",
                SupervisorPhaseName(status.phase),
                status.motion_enabled ? "enabled" : "disabled",
                status.configuration_generation, status.agent_session_id,
                status.description.c_str());
    last_status_ = status;
  }

  std::optional<SupervisorCore> core_;
  MotorClient::SharedPtr motor_client_;
  MotorAdrcClient::SharedPtr motor_adrc_client_;
  PwmClient::SharedPtr pwm_client_;
  BatteryClient::SharedPtr battery_client_;
  rclcpp::Subscription<HeartbeatMessage>::SharedPtr heartbeat_subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_gate_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr
      motion_authorization_publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  std::optional<PendingRosCall> pending_call_;
  std::optional<SupervisorStatus> last_status_;
  SupervisorCore::TimePoint last_heartbeat_time_{};
  bool heartbeat_received_ = false;
  bool heartbeat_timed_out_ = false;
  bool graph_present_ = false;
};

}  // namespace

std::shared_ptr<rclcpp::Node> MakeConfigurationSupervisorNode(
    const rclcpp::NodeOptions& options) {
  return std::make_shared<ConfigurationSupervisorNode>(options);
}

}  // namespace mentor_pi_bringup
