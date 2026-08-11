// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_monitor_node.h"

#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mentor_pi_bringup/qualification_monitor_core.h"
#include "mentor_pi_interfaces/msg/battery_state.hpp"
#include "mentor_pi_interfaces/msg/button_event.hpp"
#include "mentor_pi_interfaces/msg/controller_diagnostics.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/imu_state.hpp"
#include "mentor_pi_interfaces/msg/motor_command.hpp"
#include "mentor_pi_interfaces/msg/motor_state.hpp"
#include "mentor_pi_interfaces/msg/pwm_servo_state.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentor_pi_bringup {
namespace {

using BatteryState = mentor_pi_interfaces::msg::BatteryState;
using ButtonEvent = mentor_pi_interfaces::msg::ButtonEvent;
using ControllerDiagnostics = mentor_pi_interfaces::msg::ControllerDiagnostics;
using Heartbeat = mentor_pi_interfaces::msg::Heartbeat;
using ImuState = mentor_pi_interfaces::msg::ImuState;
using MotorCommand = mentor_pi_interfaces::msg::MotorCommand;
using MotorState = mentor_pi_interfaces::msg::MotorState;
using PwmServoState = mentor_pi_interfaces::msg::PwmServoState;

constexpr std::chrono::milliseconds kMonitorPeriod{100};
constexpr double kDefaultDurationSeconds = 60.0;
constexpr double kDefaultDiscoveryTimeoutSeconds = 5.0;
constexpr double kDefaultRateTolerancePercent = 5.0;
constexpr double kDefaultZeroCommandRateHz = 50.0;
constexpr double kMaximumZeroCommandRateHz = 500.0;
constexpr double kMinimumDurationSeconds = 2.0;
constexpr double kMaximumDurationSeconds = 86400.0;
constexpr double kNanosecondsPerSecond = 1.0e9;
constexpr std::uint16_t kMinimumPwmPulseWidthUs = 500U;
constexpr std::uint16_t kMaximumPwmPulseWidthUs = 2500U;
constexpr std::int16_t kMinimumPwmOffsetUs = -100;
constexpr std::int16_t kMaximumPwmOffsetUs = 100;
constexpr char kMotorCommandTopic[] = "/mentor_pi/motors/command";

std::int64_t MonotonicNowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::int64_t SecondsToNanoseconds(double seconds) {
  return static_cast<std::int64_t>(
      std::llround(seconds * kNanosecondsPerSecond));
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

rclcpp::QoS ReliableQos(std::size_t depth) {
  return rclcpp::QoS{rclcpp::KeepLast{depth}}.reliable().durability_volatile();
}

QualificationTimestamp ConvertStamp(
    const builtin_interfaces::msg::Time& stamp) {
  return QualificationTimestamp{stamp.sec, stamp.nanosec};
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
void AppendCounters(const std::array<Value, Size>& source,
                    DiagnosticsObservation* observation,
                    std::size_t* next_index) {
  for (const Value value : source) {
    if (*next_index < observation->monotonic_counters.size()) {
      observation->monotonic_counters[*next_index] =
          static_cast<std::uint64_t>(value);
    }
    ++(*next_index);
  }
}

template <typename Value>
void AppendCounter(Value value, DiagnosticsObservation* observation,
                   std::size_t* next_index) {
  if (*next_index < observation->monotonic_counters.size()) {
    observation->monotonic_counters[*next_index] =
        static_cast<std::uint64_t>(value);
  }
  ++(*next_index);
}

template <typename Value, std::size_t Size>
void AddCounters(const std::array<Value, Size>& source, std::uint64_t* total) {
  for (const Value value : source) {
    *total += static_cast<std::uint64_t>(value);
  }
}

DiagnosticsObservation ConvertDiagnostics(const ControllerDiagnostics& message,
                                          std::int64_t arrival_time_ns) {
  DiagnosticsObservation observation;
  observation.stamp = ConvertStamp(message.stamp);
  observation.arrival_time_ns = arrival_time_ns;
  observation.uptime_ms = message.uptime_ms;
  observation.session_generation = message.session_generation;
  observation.transport_rx_bytes = message.transport_rx_bytes;
  observation.transport_tx_bytes = message.transport_tx_bytes;
  observation.command_messages = message.command_messages;
  observation.motor_mailbox_overwrites =
      message.mailbox_overwrites[ControllerDiagnostics::SUB_MOTORS];
  observation.motor_command_consumptions = message.motor_command_consumptions;
  observation.motor_command_age_over_20_ms =
      message.motor_command_age_over_20_ms;
  observation.motor_command_max_age_us = message.motor_command_max_age_us;
  observation.peripheral_errors = message.peripheral_errors;
  observation.peripheral_timeouts = message.peripheral_timeouts;
  observation.post_seal_allocation_attempts =
      message.post_seal_allocation_attempts;
  observation.last_error_detail = message.last_error_detail;
  observation.session_state = message.session_state;
  observation.last_reset_reason = message.last_reset_reason;
  observation.last_error_code = message.last_error_code;
  observation.last_error_source = message.last_error_source;

  std::size_t next_index = 0U;
  AppendCounter(message.transport_rx_bytes, &observation, &next_index);
  AppendCounter(message.transport_tx_bytes, &observation, &next_index);
  AppendCounter(message.agent_reconnects, &observation, &next_index);
  AppendCounter(message.command_messages, &observation, &next_index);
  AppendCounter(message.command_rejections, &observation, &next_index);
  AppendCounters(message.mailbox_overwrites, &observation, &next_index);
  AppendCounter(message.button_event_drops, &observation, &next_index);
  AppendCounter(message.publication_errors, &observation, &next_index);
  AppendCounter(message.service_requests, &observation, &next_index);
  AppendCounter(message.service_completions, &observation, &next_index);
  AppendCounter(message.service_busy_rejections, &observation, &next_index);
  AppendCounter(message.service_timeouts, &observation, &next_index);
  AppendCounter(message.service_partial_results, &observation, &next_index);
  AppendCounter(message.late_response_drops, &observation, &next_index);
  AppendCounters(message.motor_lease_expiries, &observation, &next_index);
  AppendCounters(message.motor_command_rejections, &observation, &next_index);
  AppendCounter(message.motor_watchdog_trips, &observation, &next_index);
  AppendCounter(message.motor_command_consumptions, &observation, &next_index);
  AppendCounter(message.motor_command_age_over_20_ms, &observation,
                &next_index);
  AppendCounter(message.motor_command_max_age_us, &observation, &next_index);
  AppendCounter(message.executor_overruns, &observation, &next_index);
  AppendCounters(message.peripheral_errors, &observation, &next_index);
  AppendCounters(message.peripheral_timeouts, &observation, &next_index);
  AppendCounters(message.usart1_errors, &observation, &next_index);
  AppendCounter(message.transport_rx_overruns, &observation, &next_index);
  AppendCounter(message.transport_tx_timeouts, &observation, &next_index);
  AppendCounters(message.task_missed_releases, &observation, &next_index);
  AppendCounter(message.post_seal_allocation_attempts, &observation,
                &next_index);
  AppendCounter(message.usart1_rx_dma_high_water_bytes, &observation,
                &next_index);
  AppendCounter(message.maximum_transport_wait_us, &observation, &next_index);
  AppendCounters(message.task_max_execution_us, &observation, &next_index);
  if (next_index != observation.monotonic_counters.size()) {
    throw std::logic_error("diagnostic counter mapping is incomplete");
  }

  AddCounters(message.usart1_errors, &observation.transport_error_total);
  observation.transport_error_total += message.transport_rx_overruns;
  observation.transport_error_total += message.transport_tx_timeouts;

  observation.diagnostic_fault_total = message.command_rejections;
  observation.diagnostic_fault_total += message.button_event_drops;
  observation.diagnostic_fault_total += message.publication_errors;
  observation.diagnostic_fault_total += message.service_busy_rejections;
  observation.diagnostic_fault_total += message.service_timeouts;
  observation.diagnostic_fault_total += message.service_partial_results;
  observation.diagnostic_fault_total += message.late_response_drops;
  AddCounters(message.motor_lease_expiries,
              &observation.diagnostic_fault_total);
  AddCounters(message.motor_command_rejections,
              &observation.diagnostic_fault_total);
  observation.diagnostic_fault_total += message.motor_watchdog_trips;
  observation.diagnostic_fault_total += message.executor_overruns;
  AddCounters(message.peripheral_errors, &observation.diagnostic_fault_total);
  AddCounters(message.peripheral_timeouts, &observation.diagnostic_fault_total);
  observation.diagnostic_fault_total += observation.transport_error_total;
  AddCounters(message.task_missed_releases,
              &observation.diagnostic_fault_total);
  observation.diagnostic_fault_total += message.post_seal_allocation_attempts;
  return observation;
}

bool IsMotorStateValid(const MotorState& message) {
  if (!AllFinite(message.target_rps) || !AllFinite(message.measured_rps) ||
      message.motor_model > MotorState::MODEL_JGB528 ||
      (message.watchdog_stop_mask & UINT8_C(0xF0)) != 0U) {
    return false;
  }
  for (const float target : message.target_rps) {
    if (target != 0.0F) {
      return false;
    }
  }
  return true;
}

bool IsPwmServoStateValid(const PwmServoState& message) {
  for (std::size_t index = 0; index < message.target_pulse_width_us.size();
       ++index) {
    if (message.target_pulse_width_us[index] < kMinimumPwmPulseWidthUs ||
        message.target_pulse_width_us[index] > kMaximumPwmPulseWidthUs ||
        message.output_pulse_width_us[index] < kMinimumPwmPulseWidthUs ||
        message.output_pulse_width_us[index] > kMaximumPwmPulseWidthUs ||
        message.offset_us[index] < kMinimumPwmOffsetUs ||
        message.offset_us[index] > kMaximumPwmOffsetUs) {
      return false;
    }
  }
  return (message.moving_mask & UINT8_C(0xF0)) == 0U;
}

bool IsBatteryStateValid(const BatteryState& message,
                         bool allow_absent_battery) {
  return IsValidQualificationBatteryState(
             message.voltage_mv, message.low_threshold_mv, message.valid) ||
         (allow_absent_battery &&
          IsAbsentQualificationBatteryState(
              message.voltage_mv, message.low_threshold_mv, message.valid,
              message.below_threshold));
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

class QualificationMonitorNode final : public rclcpp::Node {
 public:
  QualificationMonitorNode(const rclcpp::NodeOptions& options,
                           std::shared_ptr<QualificationMonitorOutcome> outcome)
      : rclcpp::Node("qualification_monitor", "/mentor_pi",
                     MakeStrictOptions(options)),
        outcome_(std::move(outcome)),
        start_time_ns_(MonotonicNowNs()) {
    const double duration_seconds =
        declare_parameter<double>("duration_sec", kDefaultDurationSeconds);
    const double discovery_timeout_seconds = declare_parameter<double>(
        "discovery_timeout_sec", kDefaultDiscoveryTimeoutSeconds);
    const double rate_tolerance_percent = declare_parameter<double>(
        "rate_tolerance_percent", kDefaultRateTolerancePercent);
    imu_characterization_mode_ =
        declare_parameter<bool>("imu_characterization_mode", false);
    allow_missing_oled_ = declare_parameter<bool>("allow_missing_oled", false);
    publish_zero_motor_commands_ =
        declare_parameter<bool>("publish_zero_motor_commands", false);
    zero_command_rate_hz_ = declare_parameter<double>(
        "zero_command_rate_hz", kDefaultZeroCommandRateHz);
    ValidateParameters(duration_seconds, discovery_timeout_seconds,
                       rate_tolerance_percent);

    QualificationConfiguration configuration;
    configuration.start_time_ns = start_time_ns_;
    configuration.duration_ns = SecondsToNanoseconds(duration_seconds);
    configuration.discovery_timeout_ns =
        SecondsToNanoseconds(discovery_timeout_seconds);
    configuration.rate_tolerance_fraction = rate_tolerance_percent / 100.0;
    configuration.imu_characterization_mode = imu_characterization_mode_;
    configuration.allow_missing_oled = allow_missing_oled_;
    configuration.motor_command_age_evidence_required =
        publish_zero_motor_commands_;
    duration_ns_ = configuration.duration_ns;
    core_.emplace(configuration);

    parameter_callback_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter>&) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason =
              "qualification parameters are immutable after startup";
          return result;
        });

    CreateSubscriptions();
    if (imu_characterization_mode_) {
      RCLCPP_WARN(
          get_logger(),
          "first-board characterization requires live valid IMU data and a "
          "READY controller; its provisional axis transform is not release "
          "evidence");
    }
    if (allow_missing_oled_) {
      RCLCPP_WARN(
          get_logger(),
          "OLED is declared not installed: only the exact SSD1306 startup "
          "NACK is excused; OLED verification remains BLOCKED");
    }
    if (publish_zero_motor_commands_) {
      CreateZeroCommandPublisher();
      RCLCPP_WARN(
          get_logger(),
          "zero-command receive-path exercise enabled at %.3f Hz; all four "
          "targets are hard-coded literal zeros and cannot be parameterized",
          zero_command_rate_hz_);
    } else {
      RCLCPP_INFO(get_logger(),
                  "qualification monitor is read-only; no command publisher "
                  "was created");
    }

    graph_timer_ =
        create_wall_timer(kMonitorPeriod, [this]() { CheckPublishers(); });
    completion_timer_ =
        create_wall_timer(kMonitorPeriod, [this]() { CheckCompletion(); });
  }

 private:
  void ValidateParameters(double duration_seconds,
                          double discovery_timeout_seconds,
                          double rate_tolerance_percent) const {
    if (!std::isfinite(duration_seconds) ||
        duration_seconds < kMinimumDurationSeconds ||
        duration_seconds > kMaximumDurationSeconds) {
      throw std::invalid_argument(
          "duration_sec must be finite and in [2, 86400]");
    }
    if (!std::isfinite(discovery_timeout_seconds) ||
        discovery_timeout_seconds < 0.0 ||
        discovery_timeout_seconds > duration_seconds) {
      throw std::invalid_argument(
          "discovery_timeout_sec must be finite, nonnegative, and no greater "
          "than duration_sec");
    }
    if (!std::isfinite(rate_tolerance_percent) ||
        rate_tolerance_percent < 0.0 || rate_tolerance_percent > 100.0) {
      throw std::invalid_argument(
          "rate_tolerance_percent must be finite and in [0, 100]");
    }
    if (!std::isfinite(zero_command_rate_hz_) || zero_command_rate_hz_ <= 0.0 ||
        zero_command_rate_hz_ > kMaximumZeroCommandRateHz) {
      throw std::invalid_argument(
          "zero_command_rate_hz must be finite and in (0, 500]");
    }
    if (allow_missing_oled_ && !imu_characterization_mode_) {
      throw std::invalid_argument(
          "allow_missing_oled is valid only in IMU characterization mode");
    }
  }

  void CreateSubscriptions() {
    heartbeat_subscription_ = create_subscription<Heartbeat>(
        "heartbeat", ReliableQos(1U),
        [this](Heartbeat::ConstSharedPtr message) {
          HeartbeatObservation observation;
          observation.stamp = ConvertStamp(message->stamp);
          observation.arrival_time_ns = MonotonicNowNs();
          observation.sequence = message->sequence;
          observation.uptime_ms = message->uptime_ms;
          observation.agent_session_id = message->agent_session_id;
          observation.state = message->state;
          observation.time_synchronized =
              (message->flags & Heartbeat::TIME_SYNCHRONIZED) != 0U;
          observation.imu_healthy =
              (message->flags & Heartbeat::IMU_HEALTHY) != 0U;
          core_->ObserveHeartbeat(observation);
        });
    diagnostics_subscription_ = create_subscription<ControllerDiagnostics>(
        "diagnostics", ReliableQos(1U),
        [this](ControllerDiagnostics::ConstSharedPtr message) {
          core_->ObserveDiagnostics(
              ConvertDiagnostics(*message, MonotonicNowNs()));
        });
    motor_state_subscription_ = create_subscription<MotorState>(
        "motors/state", BestEffortDepthOneQos(),
        [this](MotorState::ConstSharedPtr message) {
          core_->ObserveTelemetry(
              QualificationStream::kMotorState, ConvertStamp(message->stamp),
              MonotonicNowNs(), IsMotorStateValid(*message));
        });
    pwm_state_subscription_ = create_subscription<PwmServoState>(
        "pwm_servos/state", BestEffortDepthOneQos(),
        [this](PwmServoState::ConstSharedPtr message) {
          core_->ObserveTelemetry(
              QualificationStream::kPwmServoState, ConvertStamp(message->stamp),
              MonotonicNowNs(), IsPwmServoStateValid(*message));
        });
    imu_subscription_ = create_subscription<ImuState>(
        "imu", BestEffortDepthOneQos(),
        [this](ImuState::ConstSharedPtr message) {
          ImuObservation observation;
          observation.stamp = ConvertStamp(message->stamp);
          observation.arrival_time_ns = MonotonicNowNs();
          observation.valid = message->valid;
          observation.vectors_finite =
              AllFinite(message->angular_velocity_rad_s) &&
              AllFinite(message->linear_acceleration_m_s2);
          core_->ObserveImu(observation);
        });
    battery_subscription_ = create_subscription<BatteryState>(
        "battery/state", ReliableQos(1U),
        [this](BatteryState::ConstSharedPtr message) {
          core_->ObserveTelemetry(
              QualificationStream::kBattery, ConvertStamp(message->stamp),
              MonotonicNowNs(),
              IsBatteryStateValid(*message, imu_characterization_mode_));
        });
    button_subscription_ = create_subscription<ButtonEvent>(
        "buttons/events", ReliableQos(8U),
        [this](ButtonEvent::ConstSharedPtr message) {
          core_->ObserveTelemetry(
              QualificationStream::kButtonEvents, ConvertStamp(message->stamp),
              MonotonicNowNs(), IsButtonEventValid(*message));
        });
  }

  void CreateZeroCommandPublisher() {
    zero_command_publisher_ = create_publisher<MotorCommand>(
        "motors/command", BestEffortDepthOneQos());
    const std::int64_t period_ns =
        SecondsToNanoseconds(1.0 / zero_command_rate_hz_);
    zero_command_timer_ =
        create_wall_timer(std::chrono::nanoseconds{period_ns},
                          [this]() { PublishZeroMotorCommand(); });
  }

  void PublishZeroMotorCommand() {
    if (outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    if (!core_->MotorCommandAgeBaselineReady()) {
      return;
    }
    MotorCommand command;
    command.update_mask = MotorCommand::ALL_MOTORS;
    command.target_rps.fill(0.0F);
    zero_command_publisher_->publish(command);
    core_->RecordZeroMotorCommand();
  }

  void CheckPublishers() {
    const std::int64_t now_ns = MonotonicNowNs();
    for (std::size_t index = 0; index < publisher_topics_.size(); ++index) {
      core_->ObservePublisher(
          static_cast<QualificationStream>(index),
          count_publishers(publisher_topics_[index]) > std::size_t{0}, now_ns);
    }
    if (publish_zero_motor_commands_) {
      core_->ObserveMotorCommandPublisherCount(
          count_publishers(kMotorCommandTopic), now_ns);
    }
    core_->Tick(now_ns);
  }

  void CheckCompletion() {
    if (outcome_->complete.load(std::memory_order_acquire)) {
      return;
    }
    const std::int64_t now_ns = MonotonicNowNs();
    if (now_ns - start_time_ns_ < duration_ns_) {
      return;
    }

    CheckPublishers();
    const QualificationSummary summary = core_->Finish(now_ns);
    const char* const mode =
        imu_characterization_mode_ ? "CHARACTERIZATION" : "PREFLIGHT";
    std::array<char, 2048U> text{};
    static_cast<void>(
        std::snprintf(
            text.data(), text.size(),
            "%s %s\n"
            "  Session: %" PRIu32 "\n"
            "  OLED: %s\n"
            "\n"
            "  Stream             Samples    Rate (Hz)\n"
            "  -----------------  ---------  -----------\n"
            "  Heartbeat          %9" PRIu64 "  %11.2f\n"
            "  Diagnostics        %9" PRIu64 "  %11.2f\n"
            "  Motor state        %9" PRIu64 "  %11.2f\n"
            "  PWM servo state    %9" PRIu64 "  %11.2f\n"
            "  IMU                %9" PRIu64 "  %11.2f\n"
            "  Battery            %9" PRIu64 "  %11.2f\n"
            "  Button events      %9" PRIu64 "  %11.2f\n"
            "\n"
            "  Traffic: average %.1f B/s, peak interval %.1f B/s\n"
            "  Zero commands: published=%" PRIu64 ", evidence_window=%" PRIu64
            "\n"
            "  Motor evidence: messages=%" PRIu64 ", consumed=%" PRIu64
            ", overwritten=%" PRIu64 ", accounted=%" PRIu64 "\n"
            "                  over_20_ms=%" PRIu64 ", maximum_age_us=%" PRIu32
            "\n"
            "  Failure mask: 0x%016" PRIx64 "\n"
            "  Stream masks: missing=0x%02x, lost=0x%02x, rate=0x%02x, "
            "stamp=0x%02x, invalid=0x%02x",
            mode, summary.passed ? "PASS" : "FAIL", summary.agent_session_id,
            allow_missing_oled_ ? "NOT INSTALLED / NOT TESTED" : "REQUIRED",
            summary.streams[0].sample_count,
            summary.streams[0].observed_rate_hz,
            summary.streams[1].sample_count,
            summary.streams[1].observed_rate_hz,
            summary.streams[2].sample_count,
            summary.streams[2].observed_rate_hz,
            summary.streams[3].sample_count,
            summary.streams[3].observed_rate_hz,
            summary.streams[4].sample_count,
            summary.streams[4].observed_rate_hz,
            summary.streams[5].sample_count,
            summary.streams[5].observed_rate_hz,
            summary.streams[6].sample_count,
            summary.streams[6].observed_rate_hz,
            summary.transport_bytes_per_second,
            summary.maximum_transport_interval_bytes_per_second,
            summary.zero_motor_commands_published,
            summary.zero_motor_commands_in_evidence_window,
            summary.motor_command_messages, summary.motor_command_consumptions,
            summary.motor_mailbox_overwrites, summary.motor_commands_accounted,
            summary.motor_command_age_over_20_ms,
            summary.motor_command_max_age_us, summary.failure_mask,
            static_cast<unsigned int>(summary.missing_publisher_mask),
            static_cast<unsigned int>(summary.lost_publisher_mask),
            static_cast<unsigned int>(summary.rate_failure_mask),
            static_cast<unsigned int>(summary.timestamp_failure_mask),
            static_cast<unsigned int>(summary.invalid_telemetry_mask)));

    outcome_->passed.store(summary.passed, std::memory_order_release);
    outcome_->complete.store(true, std::memory_order_release);
    if (summary.passed) {
      RCLCPP_INFO(get_logger(), "%s", text.data());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", text.data());
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<QualificationMonitorOutcome> outcome_;
  const std::int64_t start_time_ns_;
  std::int64_t duration_ns_ = 0;
  bool imu_characterization_mode_ = false;
  bool allow_missing_oled_ = false;
  bool publish_zero_motor_commands_ = false;
  double zero_command_rate_hz_ = kDefaultZeroCommandRateHz;
  std::optional<QualificationMonitorCore> core_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;

  const std::array<std::string, kQualificationStreamCount> publisher_topics_{
      "/mentor_pi/heartbeat",     "/mentor_pi/diagnostics",
      "/mentor_pi/motors/state",  "/mentor_pi/pwm_servos/state",
      "/mentor_pi/imu",           "/mentor_pi/battery/state",
      "/mentor_pi/buttons/events"};

  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeat_subscription_;
  rclcpp::Subscription<ControllerDiagnostics>::SharedPtr
      diagnostics_subscription_;
  rclcpp::Subscription<MotorState>::SharedPtr motor_state_subscription_;
  rclcpp::Subscription<PwmServoState>::SharedPtr pwm_state_subscription_;
  rclcpp::Subscription<ImuState>::SharedPtr imu_subscription_;
  rclcpp::Subscription<BatteryState>::SharedPtr battery_subscription_;
  rclcpp::Subscription<ButtonEvent>::SharedPtr button_subscription_;
  rclcpp::Publisher<MotorCommand>::SharedPtr zero_command_publisher_;
  rclcpp::TimerBase::SharedPtr zero_command_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  rclcpp::TimerBase::SharedPtr completion_timer_;
};

}  // namespace

QualificationMonitorInstance MakeQualificationMonitorNode(
    const rclcpp::NodeOptions& options) {
  QualificationMonitorInstance instance;
  instance.outcome = std::make_shared<QualificationMonitorOutcome>();
  instance.node =
      std::make_shared<QualificationMonitorNode>(options, instance.outcome);
  return instance;
}

}  // namespace mentor_pi_bringup
