// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_campaign_node.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mentor_pi_bringup/configuration.h"
#include "mentor_pi_bringup/qualification_campaign_core.h"
#include "mentor_pi_bringup/qualification_monitor_core.h"
#include "mentor_pi_interfaces/msg/battery_state.hpp"
#include "mentor_pi_interfaces/msg/bus_servo_command.hpp"
#include "mentor_pi_interfaces/msg/button_event.hpp"
#include "mentor_pi_interfaces/msg/buzzer_command.hpp"
#include "mentor_pi_interfaces/msg/controller_diagnostics.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/imu_state.hpp"
#include "mentor_pi_interfaces/msg/led_command.hpp"
#include "mentor_pi_interfaces/msg/motor_command.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "mentor_pi_interfaces/msg/oled_command.hpp"
#include "mentor_pi_interfaces/msg/pwm_servo_command.hpp"
#include "mentor_pi_interfaces/msg/pwm_servo_state.hpp"
#include "mentor_pi_interfaces/msg/result.hpp"
#include "mentor_pi_interfaces/msg/rgb_command.hpp"
#include "mentor_pi_interfaces/srv/configure_bus_servo.hpp"
#include "mentor_pi_interfaces/srv/get_bus_servo_state.hpp"
#include "mentor_pi_interfaces/srv/set_battery_threshold.hpp"
#include "mentor_pi_interfaces/srv/set_motor_model.hpp"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.hpp"
#include "mentor_pi_interfaces/srv/stop_bus_servos.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentor_pi_bringup {
namespace {

using BatteryState = mentor_pi_interfaces::msg::BatteryState;
using BusServoCommand = mentor_pi_interfaces::msg::BusServoCommand;
using ButtonEvent = mentor_pi_interfaces::msg::ButtonEvent;
using BuzzerCommand = mentor_pi_interfaces::msg::BuzzerCommand;
using ConfigureBusServo = mentor_pi_interfaces::srv::ConfigureBusServo;
using ControllerDiagnostics = mentor_pi_interfaces::msg::ControllerDiagnostics;
using GetBusServoState = mentor_pi_interfaces::srv::GetBusServoState;
using Heartbeat = mentor_pi_interfaces::msg::Heartbeat;
using ImuState = mentor_pi_interfaces::msg::ImuState;
using LedCommand = mentor_pi_interfaces::msg::LedCommand;
using MotorCommand = mentor_pi_interfaces::msg::MotorCommand;
using MotorState = mentor_pi_interfaces::msg::MotorState;
using OledCommand = mentor_pi_interfaces::msg::OledCommand;
using PwmServoCommand = mentor_pi_interfaces::msg::PwmServoCommand;
using PwmServoState = mentor_pi_interfaces::msg::PwmServoState;
using Result = mentor_pi_interfaces::msg::Result;
using RgbCommand = mentor_pi_interfaces::msg::RgbCommand;
using SetBatteryThreshold = mentor_pi_interfaces::srv::SetBatteryThreshold;
using SetMotorModel = mentor_pi_interfaces::srv::SetMotorModel;
using SetPwmServoOffsets = mentor_pi_interfaces::srv::SetPwmServoOffsets;
using StopBusServos = mentor_pi_interfaces::srv::StopBusServos;

using MotorModelClient = rclcpp::Client<SetMotorModel>;
using PwmOffsetsClient = rclcpp::Client<SetPwmServoOffsets>;
using BusStateClient = rclcpp::Client<GetBusServoState>;
using BusConfigureClient = rclcpp::Client<ConfigureBusServo>;
using BusStopClient = rclcpp::Client<StopBusServos>;
using BatteryThresholdClient = rclcpp::Client<SetBatteryThreshold>;

constexpr std::chrono::milliseconds kSchedulerPeriod{1};
constexpr std::chrono::milliseconds kGraphPeriod{100};
constexpr std::int64_t kNanosecondsPerSecond = INT64_C(1000000000);
constexpr std::int64_t kServiceTimeoutNs = INT64_C(250000000);
constexpr double kDefaultDiscoveryTimeoutSeconds = 30.0;
constexpr double kDefaultCampaignTimeoutSeconds = 7200.0;
constexpr double kMaximumTimeoutSeconds = 172800.0;
constexpr char kRosDistribution[] = "humble";
constexpr std::uint16_t kBusVolatileConfigurationMask =
    ConfigureBusServo::Request::SET_OFFSET |
    ConfigureBusServo::Request::SET_TORQUE;

static_assert((kBusVolatileConfigurationMask &
               (ConfigureBusServo::Request::SET_ID |
                ConfigureBusServo::Request::SAVE_OFFSET |
                ConfigureBusServo::Request::SET_POSITION_LIMITS |
                ConfigureBusServo::Request::SET_VOLTAGE_LIMITS |
                ConfigureBusServo::Request::SET_TEMPERATURE_LIMIT)) == 0U,
              "qualification campaign must never issue persistent bus writes");

std::int64_t MonotonicNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::int64_t SecondsToNanoseconds(double seconds) {
  return static_cast<std::int64_t>(
      std::llround(seconds * static_cast<double>(kNanosecondsPerSecond)));
}

bool IsForwardCounter(std::uint32_t current, std::uint32_t previous) {
  const std::uint32_t delta = current - previous;
  return delta != 0U && delta < UINT32_C(0x80000000);
}

std::string UtcNow() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  if (gmtime_r(&now, &utc) == nullptr) {
    throw std::runtime_error("cannot convert current UTC time");
  }
  std::array<char, 32U> text{};
  if (std::strftime(text.data(), text.size(), "%Y-%m-%dT%H:%M:%SZ", &utc) ==
      0U) {
    throw std::runtime_error("cannot format current UTC time");
  }
  return text.data();
}

rclcpp::NodeOptions MakeStrictOptions(const rclcpp::NodeOptions& options) {
  rclcpp::NodeOptions strict{options};
  strict.allow_undeclared_parameters(false);
  strict.automatically_declare_parameters_from_overrides(false);
  return strict;
}

rclcpp::QoS BestEffortDepthOneQos() {
  return rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
      .best_effort()
      .durability_volatile();
}

rclcpp::QoS ReliableQos(std::size_t depth) {
  return rclcpp::QoS{rclcpp::KeepLast{depth}}.reliable().durability_volatile();
}

template <typename Value, std::size_t Size>
bool AllFinite(const std::array<Value, Size>& values) {
  for (const Value value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

template <typename Value, std::size_t Size>
std::uint64_t SumCounters(const std::array<Value, Size>& values) {
  std::uint64_t total = 0U;
  for (const Value value : values) {
    total += static_cast<std::uint64_t>(value);
  }
  return total;
}

template <typename Value>
void AppendCampaignCounter(Value value,
                           CampaignDiagnosticsObservation* observation,
                           std::size_t* next_index) {
  if (*next_index < observation->monotonic_counters.size()) {
    observation->monotonic_counters[*next_index] =
        static_cast<std::uint64_t>(value);
  }
  ++(*next_index);
}

template <typename Value, std::size_t Size>
void AppendCampaignCounters(const std::array<Value, Size>& values,
                            CampaignDiagnosticsObservation* observation,
                            std::size_t* next_index) {
  for (const Value value : values) {
    AppendCampaignCounter(value, observation, next_index);
  }
}

bool IsMotorStateValid(const MotorState& message) {
  if (!AllFinite(message.target_rps) || !AllFinite(message.measured_rps) ||
      message.motor_model > MotorState::MODEL_JGB528 ||
      (message.watchdog_stop_mask & UINT8_C(0xF0)) != 0U) {
    return false;
  }
  return std::all_of(message.target_rps.begin(), message.target_rps.end(),
                     [](float target) { return target == 0.0F; });
}

bool IsPwmStateValid(const PwmServoState& message) {
  for (std::size_t index = 0U; index < message.target_pulse_width_us.size();
       ++index) {
    if (message.target_pulse_width_us[index] < 500U ||
        message.target_pulse_width_us[index] > 2500U ||
        message.output_pulse_width_us[index] < 500U ||
        message.output_pulse_width_us[index] > 2500U ||
        message.offset_us[index] < -100 || message.offset_us[index] > 100) {
      return false;
    }
  }
  return (message.moving_mask & UINT8_C(0xF0)) == 0U;
}

bool IsButtonEventValid(const ButtonEvent& message) {
  if (message.button_id < 1U || message.button_id > 2U) {
    return false;
  }
  switch (message.event) {
    case ButtonEvent::PRESSED:
    case ButtonEvent::LONG_PRESS:
    case ButtonEvent::LONG_PRESS_REPEAT:
    case ButtonEvent::RELEASE_FROM_LONG_PRESS:
    case ButtonEvent::RELEASE_FROM_SHORT_PRESS:
    case ButtonEvent::CLICK:
    case ButtonEvent::DOUBLE_CLICK:
    case ButtonEvent::TRIPLE_CLICK:
      return true;
    default:
      return false;
  }
}

CampaignDiagnosticsObservation ConvertDiagnostics(
    const ControllerDiagnostics& message, std::int64_t arrival_time_ns) {
  CampaignDiagnosticsObservation observation;
  observation.arrival_time_ns = arrival_time_ns;
  observation.transport_rx_bytes = message.transport_rx_bytes;
  observation.transport_tx_bytes = message.transport_tx_bytes;
  observation.uptime_ms = message.uptime_ms;
  observation.session_generation = message.session_generation;
  observation.motor_command_consumptions = message.motor_command_consumptions;
  observation.motor_command_age_over_20_ms =
      message.motor_command_age_over_20_ms;
  observation.motor_command_max_age_us = message.motor_command_max_age_us;
  observation.post_seal_allocation_attempts =
      message.post_seal_allocation_attempts;
  observation.session_state = message.session_state;
  observation.reset_reason = message.last_reset_reason;
  std::size_t next_index = 0U;
  AppendCampaignCounter(message.transport_rx_bytes, &observation, &next_index);
  AppendCampaignCounter(message.transport_tx_bytes, &observation, &next_index);
  AppendCampaignCounter(message.agent_reconnects, &observation, &next_index);
  AppendCampaignCounter(message.command_messages, &observation, &next_index);
  AppendCampaignCounter(message.command_rejections, &observation, &next_index);
  AppendCampaignCounters(message.mailbox_overwrites, &observation, &next_index);
  AppendCampaignCounter(message.button_event_drops, &observation, &next_index);
  AppendCampaignCounter(message.publication_errors, &observation, &next_index);
  AppendCampaignCounter(message.service_requests, &observation, &next_index);
  AppendCampaignCounter(message.service_completions, &observation, &next_index);
  AppendCampaignCounter(message.service_busy_rejections, &observation,
                        &next_index);
  AppendCampaignCounter(message.service_timeouts, &observation, &next_index);
  AppendCampaignCounter(message.service_partial_results, &observation,
                        &next_index);
  AppendCampaignCounter(message.late_response_drops, &observation, &next_index);
  AppendCampaignCounters(message.motor_lease_expiries, &observation,
                         &next_index);
  AppendCampaignCounters(message.motor_command_rejections, &observation,
                         &next_index);
  AppendCampaignCounter(message.motor_watchdog_trips, &observation,
                        &next_index);
  AppendCampaignCounter(message.motor_command_consumptions, &observation,
                        &next_index);
  AppendCampaignCounter(message.motor_command_age_over_20_ms, &observation,
                        &next_index);
  AppendCampaignCounter(message.motor_command_max_age_us, &observation,
                        &next_index);
  AppendCampaignCounter(message.executor_overruns, &observation, &next_index);
  AppendCampaignCounters(message.peripheral_errors, &observation, &next_index);
  AppendCampaignCounters(message.peripheral_timeouts, &observation,
                         &next_index);
  AppendCampaignCounters(message.usart1_errors, &observation, &next_index);
  AppendCampaignCounter(message.transport_rx_overruns, &observation,
                        &next_index);
  AppendCampaignCounter(message.transport_tx_timeouts, &observation,
                        &next_index);
  AppendCampaignCounters(message.task_missed_releases, &observation,
                         &next_index);
  AppendCampaignCounter(message.post_seal_allocation_attempts, &observation,
                        &next_index);
  AppendCampaignCounter(message.usart1_rx_dma_high_water_bytes, &observation,
                        &next_index);
  AppendCampaignCounter(message.maximum_transport_wait_us, &observation,
                        &next_index);
  AppendCampaignCounters(message.task_max_execution_us, &observation,
                         &next_index);
  if (next_index != observation.monotonic_counters.size()) {
    throw std::logic_error("campaign diagnostic counter mapping is incomplete");
  }
  observation.hard_error_total = message.command_rejections;
  observation.hard_error_total += message.button_event_drops;
  observation.hard_error_total += message.publication_errors;
  observation.hard_error_total += message.service_busy_rejections;
  observation.hard_error_total += message.service_timeouts;
  observation.hard_error_total += message.service_partial_results;
  observation.hard_error_total += message.late_response_drops;
  observation.hard_error_total += SumCounters(message.motor_lease_expiries);
  observation.hard_error_total += SumCounters(message.motor_command_rejections);
  observation.hard_error_total += message.motor_watchdog_trips;
  observation.hard_error_total += message.executor_overruns;
  observation.hard_error_total += SumCounters(message.peripheral_errors);
  observation.hard_error_total += SumCounters(message.peripheral_timeouts);
  observation.hard_error_total += SumCounters(message.usart1_errors);
  observation.hard_error_total += message.transport_rx_overruns;
  observation.hard_error_total += message.transport_tx_timeouts;
  observation.hard_error_total += SumCounters(message.task_missed_releases);
  observation.hard_error_total += message.post_seal_allocation_attempts;
  return observation;
}

struct PendingServiceCall {
  CampaignService service = CampaignService::kMotorModel;
  std::int64_t request_id = 0;
  std::int64_t deadline_ns = 0;
  std::uint64_t token = 0U;
};

bool ValidTestOverrides(const QualificationCampaignTestOverrides& overrides,
                        const CampaignProfile& canonical_profile) {
  if (overrides.minimum_telemetry_gap_ns < 0 ||
      overrides.profile.mode != canonical_profile.mode ||
      overrides.profile.canonical_duration_ns !=
          canonical_profile.canonical_duration_ns ||
      overrides.profile.service_round_period_ns !=
          canonical_profile.service_round_period_ns ||
      overrides.profile.expected_cycles != canonical_profile.expected_cycles ||
      overrides.profile.continuous_session_required !=
          canonical_profile.continuous_session_required) {
    return false;
  }
  return std::all_of(
      overrides.profile.command_rates.begin(),
      overrides.profile.command_rates.end(), [](CampaignProfile::Rate rate) {
        return rate.numerator > 0U && rate.denominator_seconds > 0U;
      });
}

class QualificationCampaignNode final : public rclcpp::Node {
 public:
  QualificationCampaignNode(
      const rclcpp::NodeOptions& options,
      std::shared_ptr<QualificationCampaignOutcome> outcome,
      std::optional<QualificationCampaignTestOverrides> test_overrides)
      : rclcpp::Node("qualification_campaign", "/mentor_pi",
                     MakeStrictOptions(options)),
        outcome_(std::move(outcome)),
        construction_time_ns_(MonotonicNowNs()),
        process_start_utc_(UtcNow()),
        test_overrides_(std::move(test_overrides)) {
    ParseAndValidateParameters();
    CreateInterfaces();
    parameter_callback_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>&) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason =
              "qualification parameters are immutable after startup";
          return result;
        });
    graph_timer_ = create_wall_timer(kGraphPeriod, [this]() { CheckGraph(); });
    scheduler_timer_ =
        create_wall_timer(kSchedulerPeriod, [this]() { RunScheduler(); });
    RCLCPP_WARN(
        get_logger(),
        "campaign is armed for a guarded fixture; motors remain hard-coded "
        "zero-only, physical disconnect/reset actions remain operator-driven");
  }

 private:
  void ParseAndValidateParameters() {
    const std::string mode_text =
        declare_parameter<std::string>("mode", "load500");
    const std::optional<CampaignMode> parsed_mode =
        ParseCampaignMode(mode_text);
    if (!parsed_mode.has_value()) {
      throw std::invalid_argument("unsupported qualification campaign mode");
    }
    mode_ = *parsed_mode;
    const CampaignProfile canonical_profile = CampaignProfileForMode(mode_);
    profile_ = canonical_profile;
    if (test_overrides_.has_value()) {
      if (!ValidTestOverrides(*test_overrides_, canonical_profile)) {
        throw std::invalid_argument(
            "test overrides must preserve campaign mode semantics");
      }
      profile_ = test_overrides_->profile;
    }

    const double requested_duration =
        declare_parameter<double>("duration_sec", -1.0);
    const double discovery_timeout_seconds = declare_parameter<double>(
        "discovery_timeout_sec", kDefaultDiscoveryTimeoutSeconds);
    const double campaign_timeout_seconds = declare_parameter<double>(
        "campaign_timeout_sec", kDefaultCampaignTimeoutSeconds);
    require_button_stimulus_ =
        declare_parameter<bool>("require_button_stimulus", false);
    require_valid_imu_ = declare_parameter<bool>("require_valid_imu", true);

    metadata_.campaign_mode = mode_text;
    evidence_directory_ =
        declare_parameter<std::string>("evidence_directory", "");
    metadata_.run_id = declare_parameter<std::string>("run_id", "");
    metadata_.source_revision =
        declare_parameter<std::string>("source_revision", "");
    metadata_.firmware_sha256 =
        declare_parameter<std::string>("firmware_sha256", "");
    metadata_.host_revision =
        declare_parameter<std::string>("host_revision", "");
    metadata_.ros_distribution =
        declare_parameter<std::string>("ros_distribution", kRosDistribution);
    metadata_.board_serial = declare_parameter<std::string>("board_serial", "");
    metadata_.fixture_revision =
        declare_parameter<std::string>("fixture_revision", "");
    const std::string fixture_acknowledgement =
        declare_parameter<std::string>("fixture_acknowledgement", "");
    bus_servo_id_ = declare_parameter<std::int64_t>("bus_servo_id", 0);
    bus_hold_position_ =
        declare_parameter<std::int64_t>("bus_hold_position", -1);
    bus_position_tolerance_ =
        declare_parameter<std::int64_t>("bus_position_tolerance", 20);
    bus_current_offset_ =
        declare_parameter<std::int64_t>("bus_current_offset", 0);
    bus_torque_enabled_ = declare_parameter<bool>("bus_torque_enabled", false);

    if (fixture_acknowledgement != kGuardedFixtureAcknowledgement) {
      throw std::invalid_argument(
          "exact guarded-fixture acknowledgement is required before any "
          "campaign publisher is created");
    }
    if (bus_servo_id_ < 1 || bus_servo_id_ > 253 || bus_hold_position_ < 0 ||
        bus_hold_position_ > 1000 || bus_position_tolerance_ < 0 ||
        bus_position_tolerance_ > 100 || bus_current_offset_ < -125 ||
        bus_current_offset_ > 125) {
      throw std::invalid_argument("bus fixture parameters are out of range");
    }
    if (!std::isfinite(requested_duration) ||
        !std::isfinite(discovery_timeout_seconds) ||
        !std::isfinite(campaign_timeout_seconds) ||
        discovery_timeout_seconds <= 0.0 || campaign_timeout_seconds <= 0.0 ||
        discovery_timeout_seconds > kMaximumTimeoutSeconds ||
        campaign_timeout_seconds > kMaximumTimeoutSeconds) {
      throw std::invalid_argument(
          "campaign timeouts must be finite and in (0, 172800]");
    }
    discovery_timeout_ns_ = SecondsToNanoseconds(discovery_timeout_seconds);
    campaign_timeout_ns_ = SecondsToNanoseconds(campaign_timeout_seconds);
    if (profile_.continuous_session_required) {
      if (requested_duration != -1.0 &&
          (requested_duration < 2.0 || requested_duration > 86400.0)) {
        throw std::invalid_argument("duration_sec must be -1 or in [2, 86400]");
      }
      duration_ns_ = requested_duration == -1.0
                         ? profile_.canonical_duration_ns
                         : SecondsToNanoseconds(requested_duration);
    } else {
      if (requested_duration != -1.0) {
        throw std::invalid_argument(
            "duration_sec is not used by operator-driven cycle campaigns");
      }
      duration_ns_ = 0;
    }
    if (metadata_.ros_distribution != kRosDistribution ||
        !IsValidSha256(metadata_.firmware_sha256) ||
        !IsValidCampaignEvidenceToken(metadata_.source_revision, 128U) ||
        !IsValidCampaignEvidenceToken(metadata_.host_revision, 128U) ||
        !IsValidCampaignEvidenceToken(metadata_.board_serial, 96U) ||
        !IsValidCampaignEvidenceToken(metadata_.fixture_revision, 96U) ||
        metadata_.run_id.empty() || metadata_.run_id.size() > 96U ||
        metadata_.run_id.find('/') != std::string::npos ||
        metadata_.run_id.find("..") != std::string::npos ||
        !std::filesystem::path(evidence_directory_).is_absolute()) {
      throw std::invalid_argument("evidence metadata or path is invalid");
    }
    if (std::filesystem::exists(evidence_directory_)) {
      throw std::invalid_argument(
          "evidence_directory must name a new, nonexistent directory");
    }

    CampaignConfiguration configuration;
    configuration.profile = profile_;
    configuration.duration_ns = duration_ns_;
    configuration.minimum_telemetry_gap_ns =
        test_overrides_.has_value() ? test_overrides_->minimum_telemetry_gap_ns
                                    : 0;
    configuration.require_button_stimulus = require_button_stimulus_;
    configuration.require_valid_imu = require_valid_imu_;
    core_.emplace(configuration);
  }

  void CreateInterfaces() {
    motor_publisher_ = create_publisher<MotorCommand>("motors/command",
                                                      BestEffortDepthOneQos());
    pwm_publisher_ = create_publisher<PwmServoCommand>("pwm_servos/command",
                                                       BestEffortDepthOneQos());
    bus_publisher_ = create_publisher<BusServoCommand>("bus_servos/command",
                                                       ReliableQos(1U));
    led_publisher_ =
        create_publisher<LedCommand>("leds/command", ReliableQos(1U));
    buzzer_publisher_ =
        create_publisher<BuzzerCommand>("buzzer/command", ReliableQos(1U));
    rgb_publisher_ =
        create_publisher<RgbCommand>("rgb/command", BestEffortDepthOneQos());
    oled_publisher_ =
        create_publisher<OledCommand>("oled/command", ReliableQos(1U));

    heartbeat_subscription_ = create_subscription<Heartbeat>(
        "heartbeat", ReliableQos(1U),
        [this](Heartbeat::ConstSharedPtr message) {
          ObserveHeartbeat(*message);
        });
    diagnostics_subscription_ = create_subscription<ControllerDiagnostics>(
        "diagnostics", ReliableQos(1U),
        [this](ControllerDiagnostics::ConstSharedPtr message) {
          ObserveControllerDiagnostics(*message);
        });
    motor_state_subscription_ = create_subscription<MotorState>(
        "motors/state", BestEffortDepthOneQos(),
        [this](MotorState::ConstSharedPtr message) {
          const bool valid = IsMotorStateValid(*message);
          if (valid) {
            motor_state_seen_ = true;
            motor_model_ = message->motor_model;
          }
          const std::int64_t now_ns = MonotonicNowNs();
          core_->ObserveTelemetry(CampaignTelemetry::kMotorState, now_ns,
                                  valid);
        });
    pwm_state_subscription_ = create_subscription<PwmServoState>(
        "pwm_servos/state", BestEffortDepthOneQos(),
        [this](PwmServoState::ConstSharedPtr message) {
          const bool valid = IsPwmStateValid(*message);
          if (valid) {
            pwm_state_seen_ = true;
            pwm_targets_ = message->target_pulse_width_us;
            pwm_offsets_ = message->offset_us;
          }
          const std::int64_t now_ns = MonotonicNowNs();
          core_->ObserveTelemetry(CampaignTelemetry::kPwmServoState, now_ns,
                                  valid);
        });
    imu_subscription_ = create_subscription<ImuState>(
        "imu", BestEffortDepthOneQos(),
        [this](ImuState::ConstSharedPtr message) {
          const bool valid = AllFinite(message->angular_velocity_rad_s) &&
                             AllFinite(message->linear_acceleration_m_s2) &&
                             (!require_valid_imu_ || message->valid);
          core_->ObserveTelemetry(CampaignTelemetry::kImu, MonotonicNowNs(),
                                  valid);
        });
    battery_subscription_ = create_subscription<BatteryState>(
        "battery/state", ReliableQos(1U),
        [this](BatteryState::ConstSharedPtr message) {
          const bool valid = IsValidQualificationBatteryState(
              message->voltage_mv, message->low_threshold_mv, message->valid);
          if (valid) {
            battery_seen_ = true;
            battery_threshold_mv_ = message->low_threshold_mv;
          }
          core_->ObserveTelemetry(CampaignTelemetry::kBattery, MonotonicNowNs(),
                                  valid);
        });
    button_subscription_ = create_subscription<ButtonEvent>(
        "buttons/events", ReliableQos(8U),
        [this](ButtonEvent::ConstSharedPtr message) {
          core_->ObserveTelemetry(CampaignTelemetry::kButtonEvents,
                                  MonotonicNowNs(),
                                  IsButtonEventValid(*message));
        });

    motor_model_client_ = create_client<SetMotorModel>("motors/set_model");
    pwm_offsets_client_ =
        create_client<SetPwmServoOffsets>("pwm_servos/set_offsets");
    bus_state_client_ = create_client<GetBusServoState>("bus_servos/get_state");
    bus_configure_client_ =
        create_client<ConfigureBusServo>("bus_servos/configure");
    bus_stop_client_ = create_client<StopBusServos>("bus_servos/stop");
    battery_threshold_client_ =
        create_client<SetBatteryThreshold>("battery/set_low_threshold");
  }

  void ObserveHeartbeat(const Heartbeat& message) {
    const std::int64_t now_ns = MonotonicNowNs();
    if (heartbeat_seen_ &&
        (message.agent_session_id != last_heartbeat_session_id_ ||
         !IsForwardCounter(message.uptime_ms, last_heartbeat_uptime_ms_))) {
      HandleSessionBoundary();
    }
    heartbeat_seen_ = true;
    heartbeat_outage_handled_ = false;
    last_heartbeat_arrival_ns_ = now_ns;
    last_heartbeat_session_id_ = message.agent_session_id;
    last_heartbeat_uptime_ms_ = message.uptime_ms;
    CampaignHeartbeatObservation observation;
    observation.arrival_time_ns = now_ns;
    observation.sequence = message.sequence;
    observation.uptime_ms = message.uptime_ms;
    observation.session_id = message.agent_session_id;
    observation.state = message.state;
    core_->ObserveHeartbeat(observation);
    const bool valid_state =
        message.state == Heartbeat::READY ||
        (!require_valid_imu_ && message.state == Heartbeat::DEGRADED);
    core_->ObserveTelemetry(CampaignTelemetry::kHeartbeat, now_ns, valid_state);
    if (core_->failed()) {
      BeginAbort("heartbeat/session failure", now_ns);
    }
  }

  void ObserveControllerDiagnostics(const ControllerDiagnostics& message) {
    const std::int64_t now_ns = MonotonicNowNs();
    if (diagnostics_seen_ &&
        (message.session_generation != last_diagnostics_generation_ ||
         !IsForwardCounter(message.uptime_ms, last_diagnostics_uptime_ms_))) {
      HandleSessionBoundary();
    }
    diagnostics_seen_ = true;
    last_diagnostics_generation_ = message.session_generation;
    last_diagnostics_uptime_ms_ = message.uptime_ms;
    core_->ObserveDiagnostics(ConvertDiagnostics(message, now_ns));
    core_->ObserveTelemetry(CampaignTelemetry::kDiagnostics, now_ns, true);
    if (core_->failed()) {
      BeginAbort("diagnostics/session failure", now_ns);
    }
  }

  bool GraphComplete() const {
    const std::array<std::string, kCampaignTelemetryCount> telemetry_topics{
        "/mentor_pi/heartbeat",     "/mentor_pi/diagnostics",
        "/mentor_pi/motors/state",  "/mentor_pi/pwm_servos/state",
        "/mentor_pi/imu",           "/mentor_pi/battery/state",
        "/mentor_pi/buttons/events"};
    for (const std::string& topic : telemetry_topics) {
      if (count_publishers(topic) != 1U) {
        return false;
      }
    }
    const std::array<std::string, kCampaignCommandCount> command_topics{
        "/mentor_pi/motors/command",     "/mentor_pi/pwm_servos/command",
        "/mentor_pi/bus_servos/command", "/mentor_pi/leds/command",
        "/mentor_pi/buzzer/command",     "/mentor_pi/rgb/command",
        "/mentor_pi/oled/command"};
    for (const std::string& topic : command_topics) {
      if (count_publishers(topic) != 1U || count_subscribers(topic) != 1U) {
        return false;
      }
    }
    return motor_model_client_->service_is_ready() &&
           pwm_offsets_client_->service_is_ready() &&
           bus_state_client_->service_is_ready() &&
           bus_configure_client_->service_is_ready() &&
           bus_stop_client_->service_is_ready() &&
           battery_threshold_client_->service_is_ready();
  }

  void CheckGraph() {
    if (outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    const std::int64_t now_ns = MonotonicNowNs();
    const bool complete = GraphComplete();
    if (!complete && graph_complete_) {
      HandleSessionBoundary();
    }
    graph_complete_ = complete;
    core_->ObserveEndpointGraph(complete, now_ns);
    if (core_->failed()) {
      BeginAbort("endpoint graph failure", now_ns);
      return;
    }
    if (core_->evidence_started()) {
      if (complete && !bus_admitted_ && !bus_preflight_inflight_ &&
          !pending_service_.has_value()) {
        DispatchBusPreflight();
      }
      MaybeRestartServiceRound();
      return;
    }
    if (now_ns - construction_time_ns_ > discovery_timeout_ns_) {
      Finish("graph/state discovery timeout", now_ns);
      return;
    }
    if (!complete || !heartbeat_seen_ || !diagnostics_seen_ ||
        !motor_state_seen_ || !pwm_state_seen_ || !battery_seen_) {
      return;
    }
    if (!bus_admitted_) {
      if (!bus_preflight_inflight_) {
        DispatchBusPreflight();
      }
      return;
    }
    if (!core_->BeginEvidence(now_ns)) {
      Finish("unclean or invalid evidence baseline", now_ns);
      return;
    }
    evidence_start_estimate_ns_ = now_ns;
    metadata_.start_time_utc = UtcNow();
    RCLCPP_INFO(get_logger(), "evidence window started mode=%s firmware=%s",
                CampaignModeName(mode_), metadata_.firmware_sha256.c_str());
  }

  void RunScheduler() {
    if (outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    const std::int64_t now_ns = MonotonicNowNs();
    if (aborting_) {
      CheckAbortCompletion(now_ns);
      return;
    }
    CheckBusPreflightTimeout(now_ns);
    if (heartbeat_seen_ &&
        now_ns - last_heartbeat_arrival_ns_ >= kNanosecondsPerSecond) {
      if (!heartbeat_outage_handled_) {
        HandleSessionBoundary();
        heartbeat_outage_handled_ = true;
      }
    }
    if (core_->failed()) {
      BeginAbort("latched campaign failure", now_ns);
      return;
    }
    if (!core_->evidence_started()) {
      return;
    }
    CheckServiceTimeout(now_ns);
    if (core_->failed()) {
      BeginAbort("service failure", now_ns);
      return;
    }
    const CampaignDueWork work = core_->Poll(now_ns);
    if (core_->failed()) {
      BeginAbort("schedule/diagnostic failure", now_ns);
      return;
    }
    PublishDueCommands(work.command_mask);
    if (work.service_round) {
      if (!ServiceSessionConnected(now_ns) || !bus_admitted_) {
        SkipServiceRound();
      } else if (service_round_active_ || pending_service_.has_value()) {
        SkipServiceRound();
      } else {
        service_round_active_ = true;
        next_service_ = CampaignService::kMotorModel;
        DispatchNextService();
      }
    }
    MaybeRestartServiceRound();
    if (core_->CampaignComplete(now_ns)) {
      Finish("campaign complete", now_ns);
      return;
    }
    if (!profile_.continuous_session_required &&
        now_ns - core_start_time_ns() >= campaign_timeout_ns_) {
      Finish("operator campaign timeout", now_ns);
    }
  }

  std::int64_t core_start_time_ns() const {
    return core_->evidence_started() ? evidence_start_estimate_ns_
                                     : construction_time_ns_;
  }

  void PublishDueCommands(std::uint8_t command_mask) {
    for (std::uint8_t index = 0U;
         index < static_cast<std::uint8_t>(CampaignCommand::kCount); ++index) {
      if ((command_mask & (UINT8_C(1) << index)) == 0U) {
        continue;
      }
      switch (static_cast<CampaignCommand>(index)) {
        case CampaignCommand::kMotor:
          PublishMotor();
          break;
        case CampaignCommand::kPwmServo:
          PublishPwm();
          break;
        case CampaignCommand::kBusServo:
          PublishBus();
          break;
        case CampaignCommand::kLed:
          PublishLed();
          break;
        case CampaignCommand::kBuzzer:
          PublishBuzzer();
          break;
        case CampaignCommand::kRgb:
          PublishRgb();
          break;
        case CampaignCommand::kOled:
          PublishOled();
          break;
        case CampaignCommand::kCount:
          break;
      }
    }
  }

  void PublishMotor() {
    constexpr std::array<std::uint8_t, 5U> kZeroOnlySequenceMasks{
        MotorCommand::MOTOR_1, MotorCommand::MOTOR_2, MotorCommand::MOTOR_3,
        MotorCommand::MOTOR_4, MotorCommand::ALL_MOTORS};
    MotorCommand command;
    command.update_mask =
        kZeroOnlySequenceMasks[motor_sequence_ % kZeroOnlySequenceMasks.size()];
    command.target_rps.fill(0.0F);
    if (!core_->RecordMotorCommand(command.update_mask, command.target_rps)) {
      Finish("zero-only motor invariant failed", MonotonicNowNs());
      return;
    }
    motor_publisher_->publish(command);
    ++motor_sequence_;
  }

  void PublishPwm() {
    PwmServoCommand command;
    command.update_mask = PwmServoCommand::ALL_SERVOS;
    command.duration_ms = 20U;
    command.pulse_width_us = pwm_targets_;
    pwm_publisher_->publish(command);
    core_->RecordCommand(CampaignCommand::kPwmServo);
  }

  void PublishBus() {
    if (!bus_admitted_) {
      if (profile_.continuous_session_required) {
        core_->LatchHarnessFailure(CampaignFailure::kServiceFailure);
        BeginAbort("bus command attempted without current-session admission",
                   MonotonicNowNs());
      }
      return;
    }
    BusServoCommand command;
    command.count = 1U;
    command.servo_id[0U] = static_cast<std::uint8_t>(bus_servo_id_);
    command.position[0U] = static_cast<std::uint16_t>(bus_hold_position_);
    command.duration_ms = 20U;
    bus_publisher_->publish(command);
    core_->RecordCommand(CampaignCommand::kBusServo);
  }

  void PublishLed() {
    LedCommand command;
    command.led_id = 1U;
    command.on_time_ms = 20U;
    command.off_time_ms = 20U;
    command.repeat = 1U;
    led_publisher_->publish(command);
    core_->RecordCommand(CampaignCommand::kLed);
  }

  void PublishBuzzer() {
    BuzzerCommand command;
    command.frequency_hz = 1000U;
    command.on_time_ms = 20U;
    command.off_time_ms = 20U;
    command.repeat = 1U;
    buzzer_publisher_->publish(command);
    core_->RecordCommand(CampaignCommand::kBuzzer);
  }

  void PublishRgb() {
    RgbCommand command;
    command.update_mask = RgbCommand::PIXEL_1;
    command.red.fill(0U);
    command.green.fill(0U);
    command.blue.fill(0U);
    rgb_publisher_->publish(command);
    core_->RecordCommand(CampaignCommand::kRgb);
  }

  void PublishOled() {
    OledCommand command;
    command.update_mask = OledCommand::ALL_LINES;
    command.line_1 = "RRCLite v2 qualify";
    command.line_2 = CampaignModeName(mode_);
    oled_publisher_->publish(command);
    core_->RecordCommand(CampaignCommand::kOled);
  }

  bool BusStateMatchesFixture(
      const GetBusServoState::Response& response) const {
    const std::uint16_t expected_fields =
        static_cast<std::uint16_t>(GetBusServoState::Request::FIELD_POSITION |
                                   GetBusServoState::Request::FIELD_OFFSET |
                                   GetBusServoState::Request::FIELD_TORQUE);
    const std::int64_t difference =
        std::llabs(static_cast<std::int64_t>(response.state.position) -
                   bus_hold_position_);
    return response.result.code == Result::OK &&
           (response.state.valid_fields & expected_fields) == expected_fields &&
           response.state.reported_id ==
               static_cast<std::uint8_t>(bus_servo_id_) &&
           difference <= bus_position_tolerance_ &&
           response.state.offset ==
               static_cast<std::int8_t>(bus_current_offset_) &&
           response.state.torque_enabled == bus_torque_enabled_;
  }

  bool ServiceSessionConnected(std::int64_t now_ns) const {
    return !aborting_ && graph_complete_ && heartbeat_seen_ &&
           diagnostics_seen_ &&
           last_diagnostics_generation_ == last_heartbeat_session_id_ &&
           now_ns >= last_heartbeat_arrival_ns_ &&
           now_ns - last_heartbeat_arrival_ns_ < kNanosecondsPerSecond;
  }

  bool IsCurrentService(CampaignService service, std::uint64_t token) const {
    return pending_service_.has_value() &&
           pending_service_->service == service &&
           pending_service_->token == token && token == service_token_;
  }

  void CancelServiceRoundForTransition() {
    const bool round_interrupted =
        service_round_active_ || pending_service_.has_value();
    CampaignService first_skipped = next_service_;
    if (pending_service_.has_value()) {
      first_skipped = pending_service_->service;
      RemovePendingRequest(*pending_service_);
      pending_service_.reset();
    }
    ++service_token_;
    if (round_interrupted) {
      for (std::uint8_t index = static_cast<std::uint8_t>(first_skipped);
           index < static_cast<std::uint8_t>(CampaignService::kCount);
           ++index) {
        core_->RecordServiceSkipped(static_cast<CampaignService>(index));
      }
      service_round_restart_pending_ = true;
    }
    service_round_active_ = false;
    next_service_ = CampaignService::kCount;
  }

  void CancelServiceRoundForFinish() {
    if (pending_service_.has_value()) {
      RemovePendingRequest(*pending_service_);
      pending_service_.reset();
    }
    ++service_token_;
    service_round_active_ = false;
    service_round_restart_pending_ = false;
    next_service_ = CampaignService::kCount;
  }

  void HandleSessionBoundary() {
    bus_admitted_ = false;
    bus_state_matches_fixture_ = false;
    CancelBusPreflight();
    CancelServiceRoundForTransition();
  }

  void MaybeRestartServiceRound() {
    const std::int64_t now_ns = MonotonicNowNs();
    if (!service_round_restart_pending_ || service_round_active_ ||
        pending_service_.has_value() || !bus_admitted_ ||
        !ServiceSessionConnected(now_ns) || !core_->evidence_started()) {
      return;
    }
    service_round_restart_pending_ = false;
    service_round_active_ = true;
    next_service_ = CampaignService::kMotorModel;
    DispatchNextService();
  }

  void DispatchBusPreflight() {
    if (!bus_state_client_->service_is_ready() || bus_preflight_inflight_ ||
        pending_service_.has_value() ||
        !ServiceSessionConnected(MonotonicNowNs())) {
      return;
    }
    auto request = std::make_shared<GetBusServoState::Request>();
    request->servo_id = static_cast<std::uint8_t>(bus_servo_id_);
    request->fields =
        static_cast<std::uint16_t>(GetBusServoState::Request::FIELD_POSITION |
                                   GetBusServoState::Request::FIELD_OFFSET |
                                   GetBusServoState::Request::FIELD_TORQUE);
    const std::uint64_t token = ++bus_preflight_token_;
    const std::uint32_t heartbeat_session_id = last_heartbeat_session_id_;
    const std::uint32_t diagnostics_generation = last_diagnostics_generation_;
    const auto future = bus_state_client_->async_send_request(
        request, [this, token, heartbeat_session_id, diagnostics_generation](
                     BusStateClient::SharedFuture response_future) {
          if (!bus_preflight_inflight_ || token != bus_preflight_token_) {
            return;
          }
          const std::int64_t now_ns = MonotonicNowNs();
          if (!ServiceSessionConnected(now_ns) ||
              last_heartbeat_session_id_ != heartbeat_session_id ||
              last_diagnostics_generation_ != diagnostics_generation) {
            bus_preflight_inflight_ = false;
            ++bus_preflight_token_;
            bus_admitted_ = false;
            bus_state_matches_fixture_ = false;
            return;
          }
          bus_preflight_inflight_ = false;
          try {
            const auto response = response_future.get();
            if (!BusStateMatchesFixture(*response)) {
              core_->LatchHarnessFailure(CampaignFailure::kServiceFailure);
              BeginAbort("bus fixture preflight mismatch", now_ns);
              return;
            }
            bus_admitted_ = true;
            bus_state_matches_fixture_ = true;
            MaybeRestartServiceRound();
          } catch (const std::exception&) {
            core_->LatchHarnessFailure(CampaignFailure::kServiceFailure);
            BeginAbort("bus fixture preflight exception", now_ns);
          }
        });
    bus_preflight_request_id_ = future.request_id;
    bus_preflight_deadline_ns_ = MonotonicNowNs() + kServiceTimeoutNs;
    bus_preflight_inflight_ = true;
  }

  void CancelBusPreflight() {
    if (!bus_preflight_inflight_) {
      return;
    }
    static_cast<void>(
        bus_state_client_->remove_pending_request(bus_preflight_request_id_));
    bus_preflight_inflight_ = false;
    ++bus_preflight_token_;
  }

  void CheckBusPreflightTimeout(std::int64_t now_ns) {
    if (!bus_preflight_inflight_ || now_ns < bus_preflight_deadline_ns_) {
      return;
    }
    CancelBusPreflight();
    core_->LatchHarnessFailure(CampaignFailure::kServiceFailure);
    BeginAbort("bus fixture preflight timeout", now_ns);
  }

  void PublishAbortSafeState() {
    MotorCommand motor;
    motor.update_mask = MotorCommand::ALL_MOTORS;
    motor.target_rps.fill(0.0F);
    motor_publisher_->publish(motor);

    if (pwm_state_seen_) {
      PwmServoCommand pwm;
      pwm.update_mask = PwmServoCommand::ALL_SERVOS;
      pwm.duration_ms = 20U;
      pwm.pulse_width_us = pwm_targets_;
      pwm_publisher_->publish(pwm);
    }

    LedCommand led;
    led.led_id = 1U;
    led.on_time_ms = 0U;
    led_publisher_->publish(led);
    BuzzerCommand buzzer;
    buzzer.frequency_hz = 0U;
    buzzer.on_time_ms = 0U;
    buzzer_publisher_->publish(buzzer);
    RgbCommand rgb;
    rgb.update_mask = RgbCommand::PIXEL_1;
    rgb.red.fill(0U);
    rgb.green.fill(0U);
    rgb.blue.fill(0U);
    rgb_publisher_->publish(rgb);
  }

  void BeginAbort(const char* reason, std::int64_t now_ns) {
    if (aborting_ || outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    aborting_ = true;
    abort_reason_ = reason;
    abort_deadline_ns_ = now_ns + kServiceTimeoutNs;
    CancelBusPreflight();
    CancelServiceRoundForFinish();
    PublishAbortSafeState();
    TryAbortBusStop();
  }

  void TryAbortBusStop() {
    if (abort_bus_stop_pending_ || !bus_stop_client_->service_is_ready()) {
      return;
    }
    auto request = std::make_shared<StopBusServos::Request>();
    request->count = 1U;
    request->servo_id[0U] = static_cast<std::uint8_t>(bus_servo_id_);
    const std::uint64_t token = ++abort_bus_stop_token_;
    const auto future = bus_stop_client_->async_send_request(
        request, [this, token](BusStopClient::SharedFuture) {
          if (!aborting_ || !abort_bus_stop_pending_ ||
              token != abort_bus_stop_token_) {
            return;
          }
          abort_bus_stop_pending_ = false;
          Finish(abort_reason_.c_str(), MonotonicNowNs());
        });
    abort_bus_stop_request_id_ = future.request_id;
    abort_bus_stop_pending_ = true;
  }

  void CheckAbortCompletion(std::int64_t now_ns) {
    TryAbortBusStop();
    if (now_ns < abort_deadline_ns_) {
      return;
    }
    if (abort_bus_stop_pending_) {
      static_cast<void>(
          bus_stop_client_->remove_pending_request(abort_bus_stop_request_id_));
      abort_bus_stop_pending_ = false;
    }
    Finish(abort_reason_.c_str(), now_ns);
  }

  void DispatchNextService() {
    if (!ServiceSessionConnected(MonotonicNowNs()) || !bus_admitted_) {
      CancelServiceRoundForTransition();
      return;
    }
    switch (next_service_) {
      case CampaignService::kMotorModel:
        DispatchMotorModel();
        break;
      case CampaignService::kPwmOffsets:
        DispatchPwmOffsets();
        break;
      case CampaignService::kBusState:
        DispatchBusState();
        break;
      case CampaignService::kBusConfigure:
        if (bus_state_matches_fixture_) {
          DispatchBusConfigure();
        } else {
          core_->RecordServiceSkipped(CampaignService::kBusConfigure);
          AdvanceService();
        }
        break;
      case CampaignService::kBusStop:
        DispatchBusStop();
        break;
      case CampaignService::kBatteryThreshold:
        DispatchBatteryThreshold();
        break;
      case CampaignService::kCount:
        service_round_active_ = false;
        break;
    }
  }

  template <typename Future>
  void SetPending(CampaignService service, const Future& future,
                  std::uint64_t token) {
    PendingServiceCall call;
    call.service = service;
    call.request_id = future.request_id;
    call.deadline_ns = MonotonicNowNs() + kServiceTimeoutNs;
    call.token = token;
    pending_service_ = call;
  }

  void DispatchMotorModel() {
    auto request = std::make_shared<SetMotorModel::Request>();
    request->model = motor_model_;
    const std::uint64_t token = ++service_token_;
    const auto future = motor_model_client_->async_send_request(
        request, [this, token](MotorModelClient::SharedFuture response) {
          if (!IsCurrentService(CampaignService::kMotorModel, token)) {
            return;
          }
          try {
            const auto value = response.get();
            std::uint8_t code = value->result.code;
            if (code == Result::OK &&
                ValidateMotorProfileResponse(
                    static_cast<MotorModel>(motor_model_), value->active_model,
                    value->ticks_per_revolution,
                    value->max_rps) != MotorProfileResponseError::kNone) {
              code = Result::IO_ERROR;
            }
            HandleSimpleResponse(CampaignService::kMotorModel, token, code);
          } catch (const std::exception&) {
            HandleSimpleResponse(CampaignService::kMotorModel, token,
                                 Result::IO_ERROR);
          }
        });
    SetPending(CampaignService::kMotorModel, future, token);
  }

  void DispatchPwmOffsets() {
    auto request = std::make_shared<SetPwmServoOffsets::Request>();
    request->update_mask = SetPwmServoOffsets::Request::ALL_SERVOS;
    request->offset_us = pwm_offsets_;
    const std::uint64_t token = ++service_token_;
    const auto future = pwm_offsets_client_->async_send_request(
        request, [this, token](PwmOffsetsClient::SharedFuture response) {
          if (!IsCurrentService(CampaignService::kPwmOffsets, token)) {
            return;
          }
          try {
            const auto value = response.get();
            const std::uint8_t code =
                value->result.code == Result::OK &&
                        value->applied_mask !=
                            SetPwmServoOffsets::Request::ALL_SERVOS
                    ? Result::IO_ERROR
                    : value->result.code;
            HandleSimpleResponse(CampaignService::kPwmOffsets, token, code);
          } catch (const std::exception&) {
            HandleSimpleResponse(CampaignService::kPwmOffsets, token,
                                 Result::IO_ERROR);
          }
        });
    SetPending(CampaignService::kPwmOffsets, future, token);
  }

  void DispatchBusState() {
    bus_state_matches_fixture_ = false;
    auto request = std::make_shared<GetBusServoState::Request>();
    request->servo_id = static_cast<std::uint8_t>(bus_servo_id_);
    request->fields =
        static_cast<std::uint16_t>(GetBusServoState::Request::FIELD_POSITION |
                                   GetBusServoState::Request::FIELD_OFFSET |
                                   GetBusServoState::Request::FIELD_TORQUE);
    const std::uint64_t token = ++service_token_;
    const auto future = bus_state_client_->async_send_request(
        request, [this, token](BusStateClient::SharedFuture response) {
          if (!IsCurrentService(CampaignService::kBusState, token)) {
            return;
          }
          try {
            const auto value = response.get();
            std::uint8_t code = value->result.code;
            bus_state_matches_fixture_ = BusStateMatchesFixture(*value);
            if (!bus_state_matches_fixture_) {
              code = Result::IO_ERROR;
            }
            HandleSimpleResponse(CampaignService::kBusState, token, code);
          } catch (const std::exception&) {
            bus_state_matches_fixture_ = false;
            HandleSimpleResponse(CampaignService::kBusState, token,
                                 Result::IO_ERROR);
          }
        });
    SetPending(CampaignService::kBusState, future, token);
  }

  void DispatchBusConfigure() {
    auto request = std::make_shared<ConfigureBusServo::Request>();
    request->servo_id = static_cast<std::uint8_t>(bus_servo_id_);
    request->update_mask = kBusVolatileConfigurationMask;
    request->offset = static_cast<std::int8_t>(bus_current_offset_);
    request->torque_enabled = bus_torque_enabled_;
    const std::uint64_t token = ++service_token_;
    const auto future = bus_configure_client_->async_send_request(
        request, [this, token](BusConfigureClient::SharedFuture response) {
          if (!IsCurrentService(CampaignService::kBusConfigure, token)) {
            return;
          }
          try {
            const auto value = response.get();
            const std::uint8_t code =
                value->result.code == Result::OK &&
                        (value->applied_mask != kBusVolatileConfigurationMask ||
                         value->effective_id !=
                             static_cast<std::uint8_t>(bus_servo_id_))
                    ? Result::IO_ERROR
                    : value->result.code;
            HandleSimpleResponse(CampaignService::kBusConfigure, token, code);
          } catch (const std::exception&) {
            HandleSimpleResponse(CampaignService::kBusConfigure, token,
                                 Result::IO_ERROR);
          }
        });
    SetPending(CampaignService::kBusConfigure, future, token);
  }

  void DispatchBusStop() {
    auto request = std::make_shared<StopBusServos::Request>();
    request->count = 1U;
    request->servo_id[0U] = static_cast<std::uint8_t>(bus_servo_id_);
    const std::uint64_t token = ++service_token_;
    const auto future = bus_stop_client_->async_send_request(
        request, [this, token](BusStopClient::SharedFuture response) {
          if (!IsCurrentService(CampaignService::kBusStop, token)) {
            return;
          }
          try {
            const auto value = response.get();
            const std::uint8_t code = value->result.code == Result::OK &&
                                              value->commands_transmitted != 1U
                                          ? Result::IO_ERROR
                                          : value->result.code;
            HandleSimpleResponse(CampaignService::kBusStop, token, code);
          } catch (const std::exception&) {
            HandleSimpleResponse(CampaignService::kBusStop, token,
                                 Result::IO_ERROR);
          }
        });
    SetPending(CampaignService::kBusStop, future, token);
  }

  void DispatchBatteryThreshold() {
    auto request = std::make_shared<SetBatteryThreshold::Request>();
    request->threshold_mv = battery_threshold_mv_;
    const std::uint64_t token = ++service_token_;
    const auto future = battery_threshold_client_->async_send_request(
        request, [this, token](BatteryThresholdClient::SharedFuture response) {
          if (!IsCurrentService(CampaignService::kBatteryThreshold, token)) {
            return;
          }
          try {
            const auto value = response.get();
            const std::uint8_t code =
                value->result.code == Result::OK &&
                        value->active_threshold_mv != battery_threshold_mv_
                    ? Result::IO_ERROR
                    : value->result.code;
            HandleSimpleResponse(CampaignService::kBatteryThreshold, token,
                                 code);
          } catch (const std::exception&) {
            HandleSimpleResponse(CampaignService::kBatteryThreshold, token,
                                 Result::IO_ERROR);
          }
        });
    SetPending(CampaignService::kBatteryThreshold, future, token);
  }

  void HandleSimpleResponse(CampaignService service, std::uint64_t token,
                            std::uint8_t result_code) {
    if (!IsCurrentService(service, token)) {
      return;
    }
    const std::int64_t now_ns = MonotonicNowNs();
    if (!ServiceSessionConnected(now_ns)) {
      CancelServiceRoundForTransition();
      return;
    }
    pending_service_.reset();
    core_->RecordServiceResult(service, result_code, true, false);
    if (result_code != Result::OK) {
      core_->LatchHarnessFailure(CampaignFailure::kServiceFailure);
    }
    if (core_->failed()) {
      BeginAbort("service response failure", now_ns);
      return;
    }
    AdvanceService();
  }

  void AdvanceService() {
    if (next_service_ == CampaignService::kBatteryThreshold) {
      next_service_ = CampaignService::kCount;
      service_round_active_ = false;
      return;
    }
    next_service_ = static_cast<CampaignService>(
        static_cast<std::uint8_t>(next_service_) + 1U);
    DispatchNextService();
  }

  void CheckServiceTimeout(std::int64_t now_ns) {
    if (!pending_service_.has_value() ||
        now_ns < pending_service_->deadline_ns) {
      return;
    }
    const PendingServiceCall call = *pending_service_;
    if (!ServiceSessionConnected(now_ns)) {
      CancelServiceRoundForTransition();
      return;
    }
    RemovePendingRequest(call);
    pending_service_.reset();
    ++service_token_;
    if (call.service == CampaignService::kBusState) {
      bus_state_matches_fixture_ = false;
    }
    core_->RecordServiceResult(call.service, Result::TIMEOUT, true, true);
    core_->LatchHarnessFailure(CampaignFailure::kServiceFailure);
    BeginAbort("service timeout", now_ns);
  }

  void RemovePendingRequest(const PendingServiceCall& call) {
    switch (call.service) {
      case CampaignService::kMotorModel:
        static_cast<void>(
            motor_model_client_->remove_pending_request(call.request_id));
        break;
      case CampaignService::kPwmOffsets:
        static_cast<void>(
            pwm_offsets_client_->remove_pending_request(call.request_id));
        break;
      case CampaignService::kBusState:
        static_cast<void>(
            bus_state_client_->remove_pending_request(call.request_id));
        break;
      case CampaignService::kBusConfigure:
        static_cast<void>(
            bus_configure_client_->remove_pending_request(call.request_id));
        break;
      case CampaignService::kBusStop:
        static_cast<void>(
            bus_stop_client_->remove_pending_request(call.request_id));
        break;
      case CampaignService::kBatteryThreshold:
        static_cast<void>(
            battery_threshold_client_->remove_pending_request(call.request_id));
        break;
      case CampaignService::kCount:
        break;
    }
  }

  void SkipServiceRound() {
    for (std::uint8_t index = 0U;
         index < static_cast<std::uint8_t>(CampaignService::kCount); ++index) {
      core_->RecordServiceSkipped(static_cast<CampaignService>(index));
    }
  }

  void Finish(const char* reason, std::int64_t finish_time_ns) {
    if (outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    CancelBusPreflight();
    CancelServiceRoundForFinish();
    if (abort_bus_stop_pending_) {
      static_cast<void>(
          bus_stop_client_->remove_pending_request(abort_bus_stop_request_id_));
      abort_bus_stop_pending_ = false;
      ++abort_bus_stop_token_;
    }
    if (metadata_.start_time_utc.empty()) {
      metadata_.start_time_utc = process_start_utc_;
    }
    metadata_.finish_time_utc = UtcNow();
    CampaignSummary summary = core_->Finish(finish_time_ns);
    std::string write_error;
    const bool evidence_written = WriteCampaignEvidence(
        evidence_directory_, metadata_, summary, &write_error);
    const bool passed = summary.execution_passed && evidence_written;
    outcome_->passed.store(passed, std::memory_order_release);
    outcome_->complete.store(true, std::memory_order_release);
    if (passed) {
      RCLCPP_INFO(
          get_logger(),
          "campaign PASS (%s); release qualification remains INCOMPLETE; "
          "evidence=%s",
          reason, evidence_directory_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(),
                   "campaign FAIL (%s), mask=0x%016" PRIx64 ", evidence=%s%s%s",
                   reason, summary.failure_mask, evidence_directory_.c_str(),
                   evidence_written ? "" : ", writer_error=",
                   evidence_written ? "" : write_error.c_str());
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<QualificationCampaignOutcome> outcome_;
  const std::int64_t construction_time_ns_;
  const std::string process_start_utc_;
  const std::optional<QualificationCampaignTestOverrides> test_overrides_;
  std::optional<QualificationCampaignCore> core_;
  CampaignMode mode_ = CampaignMode::kLoad500;
  CampaignProfile profile_{};
  std::int64_t duration_ns_ = 0;
  std::int64_t discovery_timeout_ns_ = 0;
  std::int64_t campaign_timeout_ns_ = 0;
  std::int64_t evidence_start_estimate_ns_ = construction_time_ns_;
  bool require_button_stimulus_ = false;
  bool require_valid_imu_ = true;
  CampaignEvidenceMetadata metadata_{};
  std::string evidence_directory_;
  std::int64_t bus_servo_id_ = 0;
  std::int64_t bus_hold_position_ = -1;
  std::int64_t bus_position_tolerance_ = 20;
  std::int64_t bus_current_offset_ = 0;
  bool bus_torque_enabled_ = false;
  bool bus_state_matches_fixture_ = false;
  bool bus_admitted_ = false;
  bool bus_preflight_inflight_ = false;
  std::int64_t bus_preflight_request_id_ = 0;
  std::int64_t bus_preflight_deadline_ns_ = 0;
  std::uint64_t bus_preflight_token_ = 0U;
  bool aborting_ = false;
  std::string abort_reason_;
  std::int64_t abort_deadline_ns_ = 0;
  bool abort_bus_stop_pending_ = false;
  std::int64_t abort_bus_stop_request_id_ = 0;
  std::uint64_t abort_bus_stop_token_ = 0U;

  bool heartbeat_seen_ = false;
  bool heartbeat_outage_handled_ = false;
  bool graph_complete_ = false;
  std::uint32_t last_heartbeat_session_id_ = 0U;
  std::uint32_t last_heartbeat_uptime_ms_ = 0U;
  std::int64_t last_heartbeat_arrival_ns_ = 0;
  bool diagnostics_seen_ = false;
  std::uint32_t last_diagnostics_generation_ = 0U;
  std::uint32_t last_diagnostics_uptime_ms_ = 0U;
  bool motor_state_seen_ = false;
  bool pwm_state_seen_ = false;
  bool battery_seen_ = false;
  std::uint8_t motor_model_ = MotorState::MODEL_JGA27;
  std::array<std::uint16_t, 4U> pwm_targets_{1500U, 1500U, 1500U, 1500U};
  std::array<std::int16_t, 4U> pwm_offsets_{};
  std::uint16_t battery_threshold_mv_ = 6300U;
  std::uint64_t motor_sequence_ = 0U;

  bool service_round_active_ = false;
  bool service_round_restart_pending_ = false;
  CampaignService next_service_ = CampaignService::kCount;
  std::uint64_t service_token_ = 0U;
  std::optional<PendingServiceCall> pending_service_;

  rclcpp::Publisher<MotorCommand>::SharedPtr motor_publisher_;
  rclcpp::Publisher<PwmServoCommand>::SharedPtr pwm_publisher_;
  rclcpp::Publisher<BusServoCommand>::SharedPtr bus_publisher_;
  rclcpp::Publisher<LedCommand>::SharedPtr led_publisher_;
  rclcpp::Publisher<BuzzerCommand>::SharedPtr buzzer_publisher_;
  rclcpp::Publisher<RgbCommand>::SharedPtr rgb_publisher_;
  rclcpp::Publisher<OledCommand>::SharedPtr oled_publisher_;
  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Subscription<ControllerDiagnostics>::SharedPtr
      diagnostics_subscription_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscription_;
  rclcpp::Subscription<PwmServoState>::SharedPtr pwm_state_subscription_;
  rclcpp::Subscription<ImuState>::SharedPtr imu_subscription_;
  rclcpp::Subscription<BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<ButtonEvent>::SharedPtr button_subscription_;
  MotorModelClient::SharedPtr motor_model_client_;
  PwmOffsetsClient::SharedPtr pwm_offsets_client_;
  BusStateClient::SharedPtr bus_state_client_;
  BusConfigureClient::SharedPtr bus_configure_client_;
  BusStopClient::SharedPtr bus_stop_client_;
  BatteryThresholdClient::SharedPtr battery_threshold_client_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  rclcpp::TimerBase::SharedPtr scheduler_timer_;
};

}  // namespace

QualificationCampaignInstance MakeQualificationCampaignNode(
    const rclcpp::NodeOptions& options) {
  QualificationCampaignInstance instance;
  instance.outcome = std::make_shared<QualificationCampaignOutcome>();
  instance.node = std::make_shared<QualificationCampaignNode>(
      options, instance.outcome, std::nullopt);
  return instance;
}

QualificationCampaignInstance MakeQualificationCampaignNodeForTest(
    const rclcpp::NodeOptions& options,
    QualificationCampaignTestOverrides overrides) {
  QualificationCampaignInstance instance;
  instance.outcome = std::make_shared<QualificationCampaignOutcome>();
  instance.node = std::make_shared<QualificationCampaignNode>(
      options, instance.outcome, std::move(overrides));
  return instance;
}

}  // namespace mentor_pi_bringup
