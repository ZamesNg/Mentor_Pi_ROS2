// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_campaign_node.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>

#include "mentor_pi_interfaces/motor_profile_contract.hpp"
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
#include "rclcpp/rclcpp.hpp"

namespace {

using namespace std::chrono_literals;

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

constexpr std::uint8_t kBusServoId = 7U;
constexpr std::uint16_t kBusHoldPosition = 500U;
constexpr std::uint16_t kBatteryThresholdMv = 6300U;
constexpr std::uint32_t kSessionId = 42U;
constexpr std::uint32_t kTestMotorRateHz = 10U;
constexpr std::int64_t kTestMinimumTelemetryGapNs = INT64_C(1000000000);

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

rclcpp::QoS ReliableQos(std::size_t depth) {
  return rclcpp::QoS{rclcpp::KeepLast{depth}}.reliable().durability_volatile();
}

class ScopedRosContext final {
 public:
  ScopedRosContext() : context_(std::make_shared<rclcpp::Context>()) {
    context_->init(0, nullptr);
  }

  ScopedRosContext(const ScopedRosContext&) = delete;
  ScopedRosContext& operator=(const ScopedRosContext&) = delete;

  ~ScopedRosContext() {
    if (context_->is_valid()) {
      static_cast<void>(context_->shutdown("qualification node test complete"));
    }
  }

  rclcpp::NodeOptions NodeOptions() const {
    rclcpp::NodeOptions options;
    options.context(context_);
    return options;
  }

  rclcpp::ExecutorOptions ExecutorOptions() const {
    rclcpp::ExecutorOptions options;
    options.context = context_;
    return options;
  }

 private:
  std::shared_ptr<rclcpp::Context> context_;
};

class ScopedEvidenceDirectory final {
 public:
  ScopedEvidenceDirectory() {
    static std::atomic<std::uint64_t> sequence{0U};
    const auto clock_value =
        std::chrono::steady_clock::now().time_since_epoch().count();
    run_id_ = "campaign_node_test_" + std::to_string(clock_value) + "_" +
              std::to_string(sequence.fetch_add(1U));
    path_ = std::filesystem::current_path() / run_id_;
  }

  ScopedEvidenceDirectory(const ScopedEvidenceDirectory&) = delete;
  ScopedEvidenceDirectory& operator=(const ScopedEvidenceDirectory&) = delete;

  ~ScopedEvidenceDirectory() {
    std::error_code error;
    const std::filesystem::path expected_parent =
        std::filesystem::current_path(error);
    if (error || !path_.is_absolute() ||
        path_.parent_path() != expected_parent ||
        path_.filename().string().rfind("campaign_node_test_", 0U) != 0U) {
      return;
    }
    const std::filesystem::file_status root_status =
        std::filesystem::symlink_status(path_, error);
    if (error || !std::filesystem::exists(root_status) ||
        std::filesystem::is_symlink(root_status) ||
        !std::filesystem::is_directory(root_status)) {
      return;
    }
    std::filesystem::permissions(path_, std::filesystem::perms::owner_write,
                                 std::filesystem::perm_options::add, error);
    if (error) {
      return;
    }
    std::filesystem::recursive_directory_iterator iterator(
        path_, std::filesystem::directory_options::none, error);
    const std::filesystem::recursive_directory_iterator end;
    while (!error && iterator != end) {
      std::error_code status_error;
      const std::filesystem::file_status status =
          iterator->symlink_status(status_error);
      if (!status_error && !std::filesystem::is_symlink(status) &&
          std::filesystem::is_directory(status)) {
        std::filesystem::permissions(
            iterator->path(), std::filesystem::perms::owner_write,
            std::filesystem::perm_options::add, status_error);
      }
      iterator.increment(error);
    }
    error.clear();
    static_cast<void>(std::filesystem::remove_all(path_, error));
  }

  const std::filesystem::path& path() const { return path_; }
  const std::string& run_id() const { return run_id_; }

 private:
  std::filesystem::path path_;
  std::string run_id_;
};

rclcpp::NodeOptions ValidOptions(
    const ScopedRosContext& ros_context,
    const ScopedEvidenceDirectory& evidence, double duration_seconds = 2.0,
    const std::string& acknowledgement =
        mentor_pi_bringup::kGuardedFixtureAcknowledgement,
    const std::string& mode = "load500", double discovery_timeout_seconds = 2.0,
    double campaign_timeout_seconds = 3.0) {
  rclcpp::NodeOptions options = ros_context.NodeOptions();
  options.parameter_overrides(
      {rclcpp::Parameter("mode", mode),
       rclcpp::Parameter("duration_sec", duration_seconds),
       rclcpp::Parameter("discovery_timeout_sec", discovery_timeout_seconds),
       rclcpp::Parameter("campaign_timeout_sec", campaign_timeout_seconds),
       rclcpp::Parameter("require_button_stimulus", false),
       rclcpp::Parameter("require_valid_imu", true),
       rclcpp::Parameter("evidence_directory", evidence.path().string()),
       rclcpp::Parameter("run_id", evidence.run_id()),
       rclcpp::Parameter("source_revision", "node-test-source"),
       rclcpp::Parameter("firmware_sha256", std::string(64U, 'a')),
       rclcpp::Parameter("host_revision", "node-test-host"),
       rclcpp::Parameter("ros_distribution", "humble"),
       rclcpp::Parameter("board_serial", "node-test-board"),
       rclcpp::Parameter("fixture_revision", "node-test-fixture"),
       rclcpp::Parameter("fixture_acknowledgement", acknowledgement),
       rclcpp::Parameter("bus_servo_id",
                         static_cast<std::int64_t>(kBusServoId)),
       rclcpp::Parameter("bus_hold_position",
                         static_cast<std::int64_t>(kBusHoldPosition)),
       rclcpp::Parameter("bus_position_tolerance", 0),
       rclcpp::Parameter("bus_current_offset", 0),
       rclcpp::Parameter("bus_torque_enabled", false)});
  return options;
}

mentor_pi_bringup::QualificationCampaignInstance MakeTestCampaignNode(
    const rclcpp::NodeOptions& options,
    mentor_pi_bringup::CampaignMode mode =
        mentor_pi_bringup::CampaignMode::kLoad500) {
  mentor_pi_bringup::QualificationCampaignTestOverrides overrides;
  overrides.profile = mentor_pi_bringup::CampaignProfileForMode(mode);
  const std::size_t motor_index =
      static_cast<std::size_t>(mentor_pi_bringup::CampaignCommand::kMotor);
  overrides.profile.command_rates[motor_index] = {kTestMotorRateHz, 1U};
  overrides.minimum_telemetry_gap_ns = kTestMinimumTelemetryGapNs;
  return mentor_pi_bringup::MakeQualificationCampaignNodeForTest(
      options, std::move(overrides));
}

template <typename Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor* executor,
               Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(100us);
  }
  executor->spin_some();
  return predicate();
}

void SpinFor(rclcpp::executors::SingleThreadedExecutor* executor,
             std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(2ms);
  }
}

template <typename Predicate>
bool WaitUntil(Predicate predicate, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(2ms);
  }
  return predicate();
}

enum class MalformedResponse : std::uint8_t {
  kNone = 0,
  kBusPreflight,
  kDelayedBusPreflightAcrossSession,
  kDelayedMotorModelAcrossSession,
  kMotorModel,
  kMotorModelNonOk,
  kMotorModelReturnedTimeout,
  kPwmOffsets,
  kBusConfigure,
  kBusStop,
  kBatteryThreshold,
  kInvalidDiscoveryTelemetry,
  kInvalidPwmAfterAdmission,
};

const char* MalformedResponseName(MalformedResponse response) {
  switch (response) {
    case MalformedResponse::kNone:
      return "none";
    case MalformedResponse::kBusPreflight:
      return "bus preflight";
    case MalformedResponse::kDelayedBusPreflightAcrossSession:
      return "delayed bus preflight across session";
    case MalformedResponse::kDelayedMotorModelAcrossSession:
      return "delayed motor model across session";
    case MalformedResponse::kMotorModel:
      return "motor model";
    case MalformedResponse::kMotorModelNonOk:
      return "motor model non-OK";
    case MalformedResponse::kMotorModelReturnedTimeout:
      return "motor model returned TIMEOUT";
    case MalformedResponse::kPwmOffsets:
      return "PWM offsets";
    case MalformedResponse::kBusConfigure:
      return "bus configure";
    case MalformedResponse::kBusStop:
      return "bus stop";
    case MalformedResponse::kBatteryThreshold:
      return "battery threshold";
    case MalformedResponse::kInvalidDiscoveryTelemetry:
      return "invalid discovery telemetry";
    case MalformedResponse::kInvalidPwmAfterAdmission:
      return "invalid PWM after admission";
  }
  return "unknown";
}

struct TrafficSnapshot {
  std::uint64_t motor = 0U;
  std::uint64_t pwm = 0U;
  std::uint64_t bus = 0U;
  std::uint64_t led = 0U;
  std::uint64_t buzzer = 0U;
  std::uint64_t rgb = 0U;
  std::uint64_t oled = 0U;

  bool operator==(const TrafficSnapshot& other) const {
    return motor == other.motor && pwm == other.pwm && bus == other.bus &&
           led == other.led && buzzer == other.buzzer && rgb == other.rgb &&
           oled == other.oled;
  }
};

class FakeQualificationController final : public rclcpp::Node {
 public:
  FakeQualificationController(const rclcpp::NodeOptions& options,
                              MalformedResponse malformed_response,
                              bool withhold_abort_bus_stop = false)
      : rclcpp::Node("fake_qualification_controller", "/mentor_pi", options),
        malformed_response_(malformed_response),
        withhold_abort_bus_stop_(withhold_abort_bus_stop) {
    CreateTelemetryPublishers();
    CreateCommandSubscriptions();
    CreateServices();
    telemetry_timer_ =
        create_wall_timer(20ms, [this]() { PublishTelemetry(); });
  }

  ~FakeQualificationController() override {
    ReleaseDelayedPreflight();
    ReleaseDelayedMotorModel();
    ReleaseWithheldAbortStop();
  }

  bool fault_injected() const { return fault_injected_.load(); }
  bool bus_command_before_current_fixture_match() const {
    return bus_command_before_current_fixture_match_;
  }
  std::uint64_t bus_fixture_successes() const {
    return bus_fixture_successes_.load();
  }
  std::uint64_t bus_commands() const { return bus_commands_.load(); }
  std::uint64_t abort_stop_requests() const {
    return abort_stop_requests_.load();
  }
  bool zero_motor_policy_valid() const { return zero_motor_policy_valid_; }
  bool safe_led_seen() const { return safe_led_seen_; }
  bool safe_buzzer_seen() const { return safe_buzzer_seen_; }
  bool active_led_after_safe() const { return active_led_after_safe_; }
  bool active_buzzer_after_safe() const { return active_buzzer_after_safe_; }
  TrafficSnapshot traffic() const { return traffic_; }
  bool delayed_preflight_started() const {
    return delayed_preflight_started_.load();
  }
  bool delayed_preflight_timed_out() const {
    return delayed_preflight_timed_out_.load();
  }
  std::uint64_t bus_state_requests() const {
    return bus_state_requests_.load();
  }
  std::uint64_t motor_model_requests() const {
    return motor_model_requests_.load();
  }
  bool delayed_motor_model_started() const {
    return delayed_motor_model_started_.load();
  }
  bool withheld_abort_stop_started() const {
    return withheld_abort_stop_started_.load();
  }
  bool abort_pwm_hold_seen() const { return abort_pwm_hold_seen_.load(); }
  bool invalid_pwm_command_seen() const {
    return invalid_pwm_command_seen_.load();
  }
  bool bus_stop_disabled() const { return bus_stop_disabled_.load(); }
  std::uint32_t last_published_session_id() const {
    return last_published_session_id_.load();
  }

  void ChangeSession(std::uint32_t session_id) {
    session_id_.store(session_id);
  }

  void RequestBusStopServiceRemoval() { remove_bus_stop_service_.store(true); }

  void ReleaseDelayedPreflight() {
    {
      std::lock_guard<std::mutex> lock(delayed_preflight_mutex_);
      delayed_preflight_released_ = true;
    }
    delayed_preflight_condition_.notify_all();
  }

  void ReleaseDelayedMotorModel() {
    {
      std::lock_guard<std::mutex> lock(delayed_motor_model_mutex_);
      delayed_motor_model_released_ = true;
    }
    delayed_motor_model_condition_.notify_all();
  }

  void ReleaseWithheldAbortStop() {
    {
      std::lock_guard<std::mutex> lock(withheld_abort_stop_mutex_);
      withheld_abort_stop_released_ = true;
    }
    withheld_abort_stop_condition_.notify_all();
  }

 private:
  bool InjectIfSelected(MalformedResponse response) {
    if (malformed_response_ != response) {
      return false;
    }
    bool expected = false;
    return fault_injected_.compare_exchange_strong(expected, true);
  }

  void CreateTelemetryPublishers() {
    heartbeat_publisher_ =
        create_publisher<Heartbeat>("heartbeat", ReliableQos(1U));
    diagnostics_publisher_ =
        create_publisher<ControllerDiagnostics>("diagnostics", ReliableQos(1U));
    motor_state_publisher_ =
        create_publisher<MotorState>("motors/state", BestEffortDepthOneQos());
    pwm_state_publisher_ = create_publisher<PwmServoState>(
        "pwm_servos/state", BestEffortDepthOneQos());
    imu_publisher_ = create_publisher<ImuState>("imu", BestEffortDepthOneQos());
    battery_publisher_ =
        create_publisher<BatteryState>("battery/state", ReliableQos(1U));
    button_publisher_ =
        create_publisher<ButtonEvent>("buttons/events", ReliableQos(8U));
  }

  void CreateCommandSubscriptions() {
    motor_subscription_ = create_subscription<MotorCommand>(
        "motors/command", BestEffortDepthOneQos(),
        [this](MotorCommand::ConstSharedPtr message) {
          ++traffic_.motor;
          if (message->update_mask == 0U ||
              (message->update_mask & UINT8_C(0xF0)) != 0U) {
            zero_motor_policy_valid_ = false;
          }
          for (const float target : message->target_rps) {
            if (!std::isfinite(target) || target != 0.0F) {
              zero_motor_policy_valid_ = false;
            }
          }
        });
    pwm_subscription_ = create_subscription<PwmServoCommand>(
        "pwm_servos/command", BestEffortDepthOneQos(),
        [this](PwmServoCommand::ConstSharedPtr message) {
          ++traffic_.pwm;
          if (malformed_response_ !=
                  MalformedResponse::kInvalidPwmAfterAdmission ||
              !fault_injected_.load()) {
            return;
          }
          const bool invalid_value = std::any_of(
              message->pulse_width_us.begin(), message->pulse_width_us.end(),
              [](std::uint16_t value) { return value == 3000U; });
          invalid_pwm_command_seen_.store(invalid_value ||
                                          invalid_pwm_command_seen_.load());
          const bool held_last_valid = std::all_of(
              message->pulse_width_us.begin(), message->pulse_width_us.end(),
              [](std::uint16_t value) { return value == 1500U; });
          abort_pwm_hold_seen_.store(held_last_valid ||
                                     abort_pwm_hold_seen_.load());
        });
    bus_subscription_ = create_subscription<BusServoCommand>(
        "bus_servos/command", ReliableQos(1U),
        [this](BusServoCommand::ConstSharedPtr message) {
          ++traffic_.bus;
          bus_commands_.fetch_add(1U);
          const bool fixture_matches_current_session =
              bus_fixture_successes_.load() > 0U &&
              admitted_session_id_.load() == session_id_.load();
          if (!fixture_matches_current_session || message->count != 1U ||
              message->servo_id[0U] != kBusServoId ||
              message->position[0U] != kBusHoldPosition) {
            bus_command_before_current_fixture_match_ = true;
          }
        });
    led_subscription_ = create_subscription<LedCommand>(
        "leds/command", ReliableQos(1U),
        [this](LedCommand::ConstSharedPtr message) {
          ++traffic_.led;
          const bool safe = message->on_time_ms == 0U;
          if (safe) {
            safe_led_seen_ = true;
          } else if (safe_led_seen_) {
            active_led_after_safe_ = true;
          }
        });
    buzzer_subscription_ = create_subscription<BuzzerCommand>(
        "buzzer/command", ReliableQos(1U),
        [this](BuzzerCommand::ConstSharedPtr message) {
          ++traffic_.buzzer;
          const bool safe =
              message->frequency_hz == 0U && message->on_time_ms == 0U;
          if (safe) {
            safe_buzzer_seen_ = true;
          } else if (safe_buzzer_seen_) {
            active_buzzer_after_safe_ = true;
          }
        });
    rgb_subscription_ = create_subscription<RgbCommand>(
        "rgb/command", BestEffortDepthOneQos(),
        [this](RgbCommand::ConstSharedPtr) { ++traffic_.rgb; });
    oled_subscription_ = create_subscription<OledCommand>(
        "oled/command", ReliableQos(1U),
        [this](OledCommand::ConstSharedPtr) { ++traffic_.oled; });
  }

  void CreateServices() {
    bus_state_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    motor_model_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    motor_model_service_ = create_service<SetMotorModel>(
        "motors/set_model",
        [this](const std::shared_ptr<SetMotorModel::Request> request,
               std::shared_ptr<SetMotorModel::Response> response) {
          const std::uint64_t request_number =
              motor_model_requests_.fetch_add(1U) + 1U;
          response->result.code = Result::OK;
          const auto* profile =
              mentor_pi_interfaces::FindMotorProfileContract(request->model);
          if (profile == nullptr) {
            response->result.code = Result::INVALID_ARGUMENT;
            return;
          }
          response->active_model = profile->model;
          response->ticks_per_revolution = profile->ticks_per_revolution;
          response->max_rps = profile->max_rps;
          if (malformed_response_ ==
                  MalformedResponse::kDelayedMotorModelAcrossSession &&
              request_number == 1U) {
            delayed_motor_model_started_.store(true);
            std::unique_lock<std::mutex> lock(delayed_motor_model_mutex_);
            static_cast<void>(delayed_motor_model_condition_.wait_for(
                lock, 500ms,
                [this]() { return delayed_motor_model_released_; }));
            response->result.code = Result::BUSY;
            fault_injected_.store(true);
            return;
          }
          if (InjectIfSelected(MalformedResponse::kMotorModelNonOk)) {
            response->result.code = Result::BUSY;
            return;
          }
          if (InjectIfSelected(MalformedResponse::kMotorModelReturnedTimeout)) {
            response->result.code = Result::TIMEOUT;
            return;
          }
          if (InjectIfSelected(MalformedResponse::kMotorModel)) {
            ++response->ticks_per_revolution;
          }
        },
        rmw_qos_profile_services_default, motor_model_callback_group_);
    pwm_offsets_service_ = create_service<SetPwmServoOffsets>(
        "pwm_servos/set_offsets",
        [this](const std::shared_ptr<SetPwmServoOffsets::Request> request,
               std::shared_ptr<SetPwmServoOffsets::Response> response) {
          response->result.code = Result::OK;
          response->applied_mask = request->update_mask;
          if (InjectIfSelected(MalformedResponse::kPwmOffsets)) {
            response->applied_mask = 0U;
          }
        });
    bus_state_service_ = create_service<GetBusServoState>(
        "bus_servos/get_state",
        [this](const std::shared_ptr<GetBusServoState::Request> request,
               std::shared_ptr<GetBusServoState::Response> response) {
          const std::uint64_t request_number =
              bus_state_requests_.fetch_add(1U) + 1U;
          const std::uint32_t request_session = session_id_.load();
          response->result.code = Result::OK;
          response->state.valid_fields = request->fields;
          response->state.requested_id = request->servo_id;
          response->state.reported_id = request->servo_id;
          response->state.position =
              static_cast<std::int16_t>(kBusHoldPosition);
          response->state.offset = 0;
          response->state.torque_enabled = false;
          if (InjectIfSelected(MalformedResponse::kBusPreflight)) {
            response->state.valid_fields = 0U;
            return;
          }
          if (malformed_response_ ==
              MalformedResponse::kDelayedBusPreflightAcrossSession) {
            if (request_number == 1U) {
              delayed_preflight_started_.store(true);
              std::unique_lock<std::mutex> lock(delayed_preflight_mutex_);
              if (!delayed_preflight_condition_.wait_for(lock, 500ms, [this]() {
                    return delayed_preflight_released_;
                  })) {
                delayed_preflight_timed_out_.store(true);
              }
              bus_fixture_successes_.fetch_add(1U);
              admitted_session_id_.store(request_session);
              return;
            }
            fault_injected_.store(true);
            response->state.valid_fields = 0U;
            return;
          }
          bus_fixture_successes_.fetch_add(1U);
          admitted_session_id_.store(request_session);
        },
        rmw_qos_profile_services_default, bus_state_callback_group_);
    bus_configure_service_ = create_service<ConfigureBusServo>(
        "bus_servos/configure",
        [this](const std::shared_ptr<ConfigureBusServo::Request> request,
               std::shared_ptr<ConfigureBusServo::Response> response) {
          response->result.code = Result::OK;
          response->applied_mask = request->update_mask;
          response->effective_id = request->servo_id;
          if (InjectIfSelected(MalformedResponse::kBusConfigure)) {
            response->effective_id =
                static_cast<std::uint8_t>(request->servo_id + 1U);
          }
        });
    bus_stop_service_ = create_service<StopBusServos>(
        "bus_servos/stop",
        [this](const std::shared_ptr<StopBusServos::Request> request,
               std::shared_ptr<StopBusServos::Response> response) {
          response->result.code = Result::OK;
          response->commands_transmitted = request->count;
          const bool malformed = InjectIfSelected(MalformedResponse::kBusStop);
          if (malformed) {
            response->commands_transmitted = 0U;
          } else if (fault_injected_.load()) {
            abort_stop_requests_.fetch_add(1U);
            if (withhold_abort_bus_stop_) {
              withheld_abort_stop_started_.store(true);
              std::unique_lock<std::mutex> lock(withheld_abort_stop_mutex_);
              static_cast<void>(withheld_abort_stop_condition_.wait_for(
                  lock, 1s,
                  [this]() { return withheld_abort_stop_released_; }));
            }
          }
        });
    battery_threshold_service_ = create_service<SetBatteryThreshold>(
        "battery/set_low_threshold",
        [this](const std::shared_ptr<SetBatteryThreshold::Request> request,
               std::shared_ptr<SetBatteryThreshold::Response> response) {
          response->result.code = Result::OK;
          response->active_threshold_mv = request->threshold_mv;
          if (InjectIfSelected(MalformedResponse::kBatteryThreshold)) {
            ++response->active_threshold_mv;
          }
        });
  }

  void PublishTelemetry() {
    if (remove_bus_stop_service_.exchange(false)) {
      bus_stop_service_.reset();
      bus_stop_disabled_.store(true);
    }
    ++sequence_;
    uptime_ms_ += 100U;

    Heartbeat heartbeat;
    heartbeat.sequence = sequence_;
    heartbeat.uptime_ms = uptime_ms_;
    heartbeat.agent_session_id = session_id_.load();
    heartbeat.state = Heartbeat::READY;
    heartbeat_publisher_->publish(heartbeat);

    ControllerDiagnostics diagnostics;
    diagnostics.uptime_ms = uptime_ms_;
    diagnostics.session_generation = session_id_.load();
    diagnostics.session_state = ControllerDiagnostics::SESSION_ACTIVE;
    diagnostics.last_reset_reason = ControllerDiagnostics::RESET_POWER_ON;
    diagnostics_publisher_->publish(diagnostics);

    MotorState motor;
    motor.motor_model = MotorState::MODEL_JGA27;
    if (malformed_response_ == MalformedResponse::kInvalidDiscoveryTelemetry) {
      motor.motor_model = 255U;
      fault_injected_.store(true);
    }
    motor_state_publisher_->publish(motor);

    PwmServoState pwm;
    pwm.target_pulse_width_us.fill(1500U);
    pwm.output_pulse_width_us.fill(1500U);
    if (malformed_response_ == MalformedResponse::kInvalidDiscoveryTelemetry ||
        (malformed_response_ == MalformedResponse::kInvalidPwmAfterAdmission &&
         bus_commands_.load() > 0U)) {
      pwm.target_pulse_width_us.fill(3000U);
      fault_injected_.store(true);
    }
    pwm_state_publisher_->publish(pwm);

    ImuState imu;
    imu.valid = true;
    imu_publisher_->publish(imu);

    BatteryState battery;
    battery.voltage_mv = 8000U;
    battery.low_threshold_mv = kBatteryThresholdMv;
    battery.valid = true;
    battery.below_threshold = false;
    if (malformed_response_ == MalformedResponse::kInvalidDiscoveryTelemetry) {
      battery.valid = false;
    }
    battery_publisher_->publish(battery);
    last_published_session_id_.store(session_id_.load());
  }

  MalformedResponse malformed_response_ = MalformedResponse::kNone;
  bool withhold_abort_bus_stop_ = false;
  std::atomic<bool> fault_injected_{false};
  bool bus_command_before_current_fixture_match_ = false;
  bool zero_motor_policy_valid_ = true;
  bool safe_led_seen_ = false;
  bool safe_buzzer_seen_ = false;
  bool active_led_after_safe_ = false;
  bool active_buzzer_after_safe_ = false;
  std::atomic<std::uint64_t> bus_fixture_successes_{0U};
  std::atomic<std::uint64_t> bus_commands_{0U};
  std::atomic<std::uint64_t> abort_stop_requests_{0U};
  std::atomic<std::uint64_t> bus_state_requests_{0U};
  std::atomic<std::uint64_t> motor_model_requests_{0U};
  std::atomic<std::uint32_t> admitted_session_id_{0U};
  std::atomic<std::uint32_t> session_id_{kSessionId};
  std::atomic<std::uint32_t> last_published_session_id_{0U};
  std::atomic<bool> remove_bus_stop_service_{false};
  std::atomic<bool> bus_stop_disabled_{false};
  std::atomic<bool> abort_pwm_hold_seen_{false};
  std::atomic<bool> invalid_pwm_command_seen_{false};
  std::atomic<bool> delayed_preflight_started_{false};
  std::atomic<bool> delayed_preflight_timed_out_{false};
  std::mutex delayed_preflight_mutex_;
  std::condition_variable delayed_preflight_condition_;
  bool delayed_preflight_released_ = false;
  std::atomic<bool> delayed_motor_model_started_{false};
  std::mutex delayed_motor_model_mutex_;
  std::condition_variable delayed_motor_model_condition_;
  bool delayed_motor_model_released_ = false;
  std::atomic<bool> withheld_abort_stop_started_{false};
  std::mutex withheld_abort_stop_mutex_;
  std::condition_variable withheld_abort_stop_condition_;
  bool withheld_abort_stop_released_ = false;
  std::uint32_t sequence_ = 0U;
  std::uint32_t uptime_ms_ = 0U;
  TrafficSnapshot traffic_{};

  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Publisher<ControllerDiagnostics>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<MotorState>::SharedPtr motor_state_publisher_;
  rclcpp::Publisher<PwmServoState>::SharedPtr pwm_state_publisher_;
  rclcpp::Publisher<ImuState>::SharedPtr imu_publisher_;
  rclcpp::Publisher<BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<ButtonEvent>::SharedPtr button_publisher_;
  rclcpp::Subscription<MotorCommand>::SharedPtr motor_subscription_;
  rclcpp::Subscription<PwmServoCommand>::SharedPtr pwm_subscription_;
  rclcpp::Subscription<BusServoCommand>::SharedPtr bus_subscription_;
  rclcpp::Subscription<LedCommand>::SharedPtr led_subscription_;
  rclcpp::Subscription<BuzzerCommand>::SharedPtr buzzer_subscription_;
  rclcpp::Subscription<RgbCommand>::SharedPtr rgb_subscription_;
  rclcpp::Subscription<OledCommand>::SharedPtr oled_subscription_;
  rclcpp::Service<SetMotorModel>::SharedPtr motor_model_service_;
  rclcpp::Service<SetPwmServoOffsets>::SharedPtr pwm_offsets_service_;
  rclcpp::Service<GetBusServoState>::SharedPtr bus_state_service_;
  rclcpp::Service<ConfigureBusServo>::SharedPtr bus_configure_service_;
  rclcpp::Service<StopBusServos>::SharedPtr bus_stop_service_;
  rclcpp::Service<SetBatteryThreshold>::SharedPtr battery_threshold_service_;
  rclcpp::CallbackGroup::SharedPtr bus_state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr motor_model_callback_group_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
};

void TestDurationBelowSentinelIsRejected() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  bool threw = false;
  try {
    static_cast<void>(
        MakeTestCampaignNode(ValidOptions(ros_context, evidence, -2.0)));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  Expect(threw, "duration_sec below the exact -1 sentinel is rejected");
}

void TestNonfiniteAndOversizeTimesAreRejected() {
  constexpr double kAboveMaximumTimeout = 172801.0;
  const std::array<double, 3U> invalid_values{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), kAboveMaximumTimeout};
  for (const double value : invalid_values) {
    {
      ScopedRosContext ros_context;
      ScopedEvidenceDirectory evidence;
      bool threw = false;
      try {
        static_cast<void>(MakeTestCampaignNode(
            ValidOptions(ros_context, evidence, 2.0,
                         mentor_pi_bringup::kGuardedFixtureAcknowledgement,
                         "load500", value, 3.0)));
      } catch (const std::invalid_argument&) {
        threw = true;
      }
      Expect(threw,
             "invalid discovery timeout is rejected before integer rounding");
    }
    {
      ScopedRosContext ros_context;
      ScopedEvidenceDirectory evidence;
      bool threw = false;
      try {
        static_cast<void>(MakeTestCampaignNode(
            ValidOptions(ros_context, evidence, 2.0,
                         mentor_pi_bringup::kGuardedFixtureAcknowledgement,
                         "load500", 2.0, value)));
      } catch (const std::invalid_argument&) {
        threw = true;
      }
      Expect(threw,
             "invalid campaign timeout is rejected before integer rounding");
    }
  }

  for (const double value :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), 86401.0}) {
    ScopedRosContext ros_context;
    ScopedEvidenceDirectory evidence;
    bool threw = false;
    try {
      static_cast<void>(
          MakeTestCampaignNode(ValidOptions(ros_context, evidence, value)));
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    Expect(threw, "invalid duration is rejected before integer rounding");
  }
}

void TestInvalidDiscoveryTelemetryCannotPrimeCaches() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), MalformedResponse::kInvalidDiscoveryTelemetry);
  const auto campaign = MakeTestCampaignNode(ValidOptions(
      ros_context, evidence, 2.0,
      mentor_pi_bringup::kGuardedFixtureAcknowledgement, "load500", 0.35, 3.0));
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool completed = SpinUntil(
      &executor,
      [&campaign]() {
        return campaign.outcome->complete.load(std::memory_order_acquire);
      },
      1500ms);

  Expect(completed && !campaign.outcome->passed.load(std::memory_order_acquire),
         "invalid initial motor/PWM/battery samples end in discovery failure");
  Expect(controller->fault_injected(),
         "the invalid discovery telemetry was actually published");
  Expect(controller->bus_state_requests() == 0U,
         "invalid initial state cannot reach bus fixture preflight");
  Expect(controller->bus_commands() == 0U,
         "invalid initial state cannot admit command traffic");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestInvalidOngoingPwmDoesNotPoisonAbortHold() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), MalformedResponse::kInvalidPwmAfterAdmission);
  const auto campaign =
      MakeTestCampaignNode(ValidOptions(ros_context, evidence));
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool failed_closed = SpinUntil(
      &executor,
      [&campaign, &controller]() {
        return campaign.outcome->complete.load(std::memory_order_acquire) &&
               controller->abort_pwm_hold_seen();
      },
      2s);

  Expect(failed_closed &&
             !campaign.outcome->passed.load(std::memory_order_acquire),
         "an invalid ongoing PWM state causes a bounded failed outcome");
  Expect(controller->fault_injected(),
         "the invalid ongoing PWM sample was actually published");
  Expect(controller->abort_pwm_hold_seen(),
         "abort republishes the last valid 1500-us PWM hold");
  Expect(!controller->invalid_pwm_command_seen(),
         "the rejected 3000-us PWM sample never enters the command cache");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestInvalidAcknowledgementCreatesNoCommandPublishers() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto observer =
      std::make_shared<rclcpp::Node>("qualification_construction_observer",
                                     "/mentor_pi", ros_context.NodeOptions());
  bool threw = false;
  try {
    static_cast<void>(MakeTestCampaignNode(
        ValidOptions(ros_context, evidence, 2.0, "PERIPHERALS_DISCONNECTED")));
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  Expect(threw, "inexact guarded-fixture acknowledgement is rejected");

  const std::array<std::string, 7U> command_topics{
      "/mentor_pi/motors/command",     "/mentor_pi/pwm_servos/command",
      "/mentor_pi/bus_servos/command", "/mentor_pi/leds/command",
      "/mentor_pi/buzzer/command",     "/mentor_pi/rgb/command",
      "/mentor_pi/oled/command"};
  for (const std::string& topic : command_topics) {
    Expect(observer->count_publishers(topic) == 0U,
           "rejected acknowledgement creates no publisher on " + topic);
  }
}

void TestBusCommandsRequireSuccessfulFixturePreflight() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), MalformedResponse::kBusPreflight);
  const auto campaign =
      MakeTestCampaignNode(ValidOptions(ros_context, evidence));
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool aborted = SpinUntil(
      &executor,
      [&campaign, &controller]() {
        return campaign.outcome->complete.load(std::memory_order_acquire) &&
               controller->abort_stop_requests() > 0U;
      },
      2s);
  SpinFor(&executor, 50ms);

  Expect(aborted, "mismatched bus fixture preflight aborts within two seconds");
  Expect(controller->fault_injected(),
         "the test actually injected a malformed bus preflight response");
  Expect(controller->bus_fixture_successes() == 0U,
         "mismatched preflight never records bus admission");
  Expect(controller->bus_commands() == 0U,
         "no bus command is published before a successful fixture match");
  Expect(campaign.outcome->complete.load(std::memory_order_acquire) &&
             !campaign.outcome->passed.load(std::memory_order_acquire),
         "bus fixture mismatch produces an explicit failed outcome");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestStalePreflightResponseCannotCrossSession() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(),
      MalformedResponse::kDelayedBusPreflightAcrossSession);
  const auto campaign = MakeTestCampaignNode(
      ValidOptions(ros_context, evidence, -1.0,
                   mentor_pi_bringup::kGuardedFixtureAcknowledgement,
                   "reconnect_agent"),
      mentor_pi_bringup::CampaignMode::kReconnectAgent);
  rclcpp::executors::MultiThreadedExecutor executor{
      ros_context.ExecutorOptions(), std::size_t{4}};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  std::atomic<bool> executor_failed{false};
  std::thread executor_thread([&executor, &executor_failed]() {
    try {
      executor.spin();
    } catch (const std::exception&) {
      executor_failed.store(true);
    } catch (...) {
      executor_failed.store(true);
    }
  });

  const bool preflight_started = WaitUntil(
      [&controller]() { return controller->delayed_preflight_started(); }, 2s);
  if (preflight_started) {
    controller->ChangeSession(kSessionId + 1U);
    std::this_thread::sleep_for(120ms);
  }
  controller->ReleaseDelayedPreflight();
  const bool terminal_observed = WaitUntil(
      [&campaign, &controller]() {
        return campaign.outcome->complete.load(std::memory_order_acquire) ||
               controller->bus_commands() > 0U;
      },
      2s);

  executor.cancel();
  executor_thread.join();

  Expect(preflight_started,
         "the first bus preflight response was held for a session change");
  Expect(!controller->delayed_preflight_timed_out(),
         "the held preflight was released before its bounded test timeout");
  Expect(
      terminal_observed,
      "session-crossing preflight test reaches an observable terminal state");
  Expect(!executor_failed.load(),
         "session-crossing preflight test has no executor exception");
  Expect(controller->bus_state_requests() >= 2U,
         "new session requires a second bus fixture preflight");
  Expect(controller->fault_injected(),
         "the new-session preflight response reached its deliberate mismatch");
  Expect(controller->bus_fixture_successes() == 1U,
         "only the stale old-session server response matched the fixture");
  Expect(controller->bus_commands() == 0U,
         "stale old-session preflight response never admits a bus command");
  Expect(campaign.outcome->complete.load(std::memory_order_acquire) &&
             !campaign.outcome->passed.load(std::memory_order_acquire),
         "new-session fixture mismatch produces a bounded failed outcome");
  Expect(controller->abort_stop_requests() > 0U,
         "new-session fixture mismatch performs the bounded bus stop");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestStaleRegularServiceResponseCannotCrossSession() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(),
      MalformedResponse::kDelayedMotorModelAcrossSession);
  const auto campaign = MakeTestCampaignNode(
      ValidOptions(ros_context, evidence, -1.0,
                   mentor_pi_bringup::kGuardedFixtureAcknowledgement,
                   "reconnect_agent", 2.0, 6.0),
      mentor_pi_bringup::CampaignMode::kReconnectAgent);
  rclcpp::executors::MultiThreadedExecutor executor{
      ros_context.ExecutorOptions(), std::size_t{4}};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  std::atomic<bool> executor_failed{false};
  std::thread executor_thread([&executor, &executor_failed]() {
    try {
      executor.spin();
    } catch (const std::exception&) {
      executor_failed.store(true);
    } catch (...) {
      executor_failed.store(true);
    }
  });

  const bool request_started = WaitUntil(
      [&controller]() { return controller->delayed_motor_model_started(); },
      2s);
  bool new_session_published = false;
  if (request_started) {
    controller->ChangeSession(kSessionId + 1U);
    new_session_published = WaitUntil(
        [&controller]() {
          return controller->last_published_session_id() == kSessionId + 1U;
        },
        100ms);
    std::this_thread::sleep_for(20ms);
  }
  controller->ReleaseDelayedMotorModel();
  const bool old_response_released = WaitUntil(
      [&controller]() { return controller->fault_injected(); }, 100ms);
  std::this_thread::sleep_for(30ms);

  executor.cancel();
  executor_thread.join();

  Expect(request_started,
         "the first regular service response was held for a session change");
  Expect(new_session_published,
         "the changed session was published before the old response release");
  Expect(old_response_released,
         "the released old-session response carried a deliberate non-OK");
  Expect(!executor_failed.load(),
         "regular-service token isolation has no executor exception");
  Expect(!campaign.outcome->complete.load(std::memory_order_acquire),
         "a stale old-session non-OK response is ignored, not failed");
  Expect(!controller->bus_command_before_current_fixture_match(),
         "new-session bus commands wait for new-session fixture admission");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestHeartbeatSessionChangeRevokesBusAdmission() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), MalformedResponse::kNone);
  const auto campaign = MakeTestCampaignNode(
      ValidOptions(ros_context, evidence, -1.0,
                   mentor_pi_bringup::kGuardedFixtureAcknowledgement,
                   "reconnect_agent", 2.0, 6.0),
      mentor_pi_bringup::CampaignMode::kReconnectAgent);
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool admitted = SpinUntil(
      &executor, [&controller]() { return controller->bus_commands() > 0U; },
      2s);
  controller->ChangeSession(kSessionId + 1U);
  const bool readmitted = SpinUntil(
      &executor,
      [&controller]() { return controller->bus_fixture_successes() >= 2U; },
      500ms);
  Expect(admitted, "heartbeat-session fixture reached admitted bus traffic");
  Expect(readmitted,
         "a changed heartbeat session completes a new fixture preflight");
  Expect(!controller->bus_command_before_current_fixture_match(),
         "no bus command crosses a heartbeat admission boundary");
  Expect(!campaign.outcome->complete.load(std::memory_order_acquire),
         "a valid reconnect session change remains nonterminal");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestGraphLossWithUnavailableBusStopIsBounded() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), MalformedResponse::kNone);
  const auto campaign =
      MakeTestCampaignNode(ValidOptions(ros_context, evidence));
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool admitted = SpinUntil(
      &executor, [&controller]() { return controller->bus_commands() > 0U; },
      2s);
  controller->RequestBusStopServiceRemoval();
  const bool service_removed = SpinUntil(
      &executor, [&controller]() { return controller->bus_stop_disabled(); },
      500ms);
  const auto removal_time = std::chrono::steady_clock::now();
  const bool completed = SpinUntil(
      &executor,
      [&campaign]() {
        return campaign.outcome->complete.load(std::memory_order_acquire);
      },
      750ms);
  const auto completion_delay = std::chrono::steady_clock::now() - removal_time;

  Expect(admitted && service_removed,
         "graph-loss test removes bus-stop only after admitted traffic");
  Expect(completed && !campaign.outcome->passed.load(std::memory_order_acquire),
         "loss of a required endpoint fails the continuous campaign");
  Expect(completion_delay <= 600ms,
         "unavailable bus-stop cannot hold abort beyond its bounded deadline");
  Expect(controller->abort_stop_requests() == 0U,
         "the removed bus-stop endpoint was not falsely reported successful");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestWithheldAbortBusStopIsBounded() {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), MalformedResponse::kMotorModel, true);
  const auto campaign =
      MakeTestCampaignNode(ValidOptions(ros_context, evidence));
  rclcpp::executors::MultiThreadedExecutor executor{
      ros_context.ExecutorOptions(), std::size_t{4}};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  std::thread executor_thread([&executor]() { executor.spin(); });
  const bool stop_withheld = WaitUntil(
      [&controller]() { return controller->withheld_abort_stop_started(); },
      2s);
  const auto withheld_time = std::chrono::steady_clock::now();
  const bool completed = WaitUntil(
      [&campaign]() {
        return campaign.outcome->complete.load(std::memory_order_acquire);
      },
      600ms);
  const auto completion_delay =
      std::chrono::steady_clock::now() - withheld_time;
  controller->ReleaseWithheldAbortStop();
  executor.cancel();
  executor_thread.join();

  Expect(stop_withheld,
         "the abort bus-stop response was deliberately withheld");
  Expect(completed && !campaign.outcome->passed.load(std::memory_order_acquire),
         "withheld bus-stop still reaches a failed terminal outcome");
  Expect(completion_delay <= 400ms,
         "withheld bus-stop cannot extend the 250-ms abort deadline");
  Expect(controller->abort_stop_requests() > 0U,
         "the bounded abort attempted the bus-servo stop service");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestConnectedNonOkFailsOperatorMode(MalformedResponse response_mode) {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), response_mode);
  const auto campaign = MakeTestCampaignNode(
      ValidOptions(ros_context, evidence, -1.0,
                   mentor_pi_bringup::kGuardedFixtureAcknowledgement,
                   "reconnect_agent", 2.0, 6.0),
      mentor_pi_bringup::CampaignMode::kReconnectAgent);
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool completed = SpinUntil(
      &executor,
      [&campaign, &controller]() {
        return campaign.outcome->complete.load(std::memory_order_acquire) &&
               controller->abort_stop_requests() > 0U;
      },
      2s);
  const std::string endpoint = MalformedResponseName(response_mode);
  Expect(completed && controller->fault_injected(),
         endpoint + " was returned while the service session was connected");
  Expect(!campaign.outcome->passed.load(std::memory_order_acquire),
         endpoint + " fails even in a non-continuous operator campaign");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

void TestMalformedOkResponseFailsClosed(MalformedResponse response_mode) {
  ScopedRosContext ros_context;
  ScopedEvidenceDirectory evidence;
  auto controller = std::make_shared<FakeQualificationController>(
      ros_context.NodeOptions(), response_mode);
  const auto campaign =
      MakeTestCampaignNode(ValidOptions(ros_context, evidence));
  rclcpp::executors::SingleThreadedExecutor executor{
      ros_context.ExecutorOptions()};
  executor.add_node(controller);
  executor.add_node(campaign.node);

  const bool failed_closed = SpinUntil(
      &executor,
      [&campaign, &controller]() {
        return campaign.outcome->complete.load(std::memory_order_acquire) &&
               controller->abort_stop_requests() > 0U &&
               controller->safe_led_seen() && controller->safe_buzzer_seen();
      },
      2s);
  SpinFor(&executor, 50ms);
  const TrafficSnapshot settled_traffic = controller->traffic();
  SpinFor(&executor, 100ms);

  const std::string endpoint = MalformedResponseName(response_mode);
  Expect(failed_closed,
         endpoint + " malformed OK response performs a bounded safe abort");
  Expect(controller->fault_injected(),
         endpoint + " malformed OK response was reached and injected");
  Expect(campaign.outcome->complete.load(std::memory_order_acquire) &&
             !campaign.outcome->passed.load(std::memory_order_acquire),
         endpoint + " malformed OK response cannot produce PASS");
  Expect(controller->abort_stop_requests() > 0U,
         endpoint + " failure issues a bounded bus-servo stop request");
  Expect(controller->zero_motor_policy_valid(),
         endpoint + " failure never emits a nonzero motor command");
  Expect(!controller->active_led_after_safe(),
         endpoint + " failure emits no active LED command after LED-off");
  Expect(!controller->active_buzzer_after_safe(),
         endpoint + " failure emits no active buzzer command after buzzer-off");
  Expect(controller->traffic() == settled_traffic,
         endpoint + " failure stops all command traffic after completion");
  Expect(controller->bus_fixture_successes() > 0U,
         endpoint + " test passed a real bus fixture preflight first");
  Expect(controller->bus_commands() > 0U,
         endpoint + " test exercised admitted bus command publication");
  Expect(!controller->bus_command_before_current_fixture_match(),
         endpoint +
             " bus commands follow a successful current-session fixture match");

  executor.remove_node(campaign.node);
  executor.remove_node(controller);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    TestDurationBelowSentinelIsRejected();
    TestNonfiniteAndOversizeTimesAreRejected();
    TestInvalidAcknowledgementCreatesNoCommandPublishers();
    TestInvalidDiscoveryTelemetryCannotPrimeCaches();
    TestBusCommandsRequireSuccessfulFixturePreflight();
    TestStalePreflightResponseCannotCrossSession();
    TestStaleRegularServiceResponseCannotCrossSession();
    TestHeartbeatSessionChangeRevokesBusAdmission();
    TestInvalidOngoingPwmDoesNotPoisonAbortHold();
    TestGraphLossWithUnavailableBusStopIsBounded();
    TestWithheldAbortBusStopIsBounded();
    TestConnectedNonOkFailsOperatorMode(MalformedResponse::kMotorModelNonOk);
    TestConnectedNonOkFailsOperatorMode(
        MalformedResponse::kMotorModelReturnedTimeout);
    const std::array<MalformedResponse, 5U> malformed_responses{
        MalformedResponse::kMotorModel, MalformedResponse::kPwmOffsets,
        MalformedResponse::kBusConfigure, MalformedResponse::kBusStop,
        MalformedResponse::kBatteryThreshold};
    for (const MalformedResponse response : malformed_responses) {
      TestMalformedOkResponseFailsClosed(response);
    }
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    ++g_failures;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  if (g_failures == 0) {
    std::cout << "qualification campaign ROS node tests passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
