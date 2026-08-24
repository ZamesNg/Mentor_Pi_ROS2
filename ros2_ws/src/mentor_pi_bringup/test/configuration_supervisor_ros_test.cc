// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mentor_pi_bringup/configuration_supervisor_node.h"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/result.hpp"
#include "mentor_pi_interfaces/srv/set_battery_threshold.hpp"
#include "mentor_pi_interfaces/srv/set_motor_adrc.hpp"
#include "mentor_pi_interfaces/srv/set_motor_model.hpp"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace {

using namespace std::chrono_literals;
using Heartbeat = mentor_pi_interfaces::msg::Heartbeat;
using Result = mentor_pi_interfaces::msg::Result;
using SetBatteryThreshold = mentor_pi_interfaces::srv::SetBatteryThreshold;
using SetMotorModel = mentor_pi_interfaces::srv::SetMotorModel;
using SetMotorAdrc = mentor_pi_interfaces::srv::SetMotorAdrc;
using SetPwmServoOffsets = mentor_pi_interfaces::srv::SetPwmServoOffsets;

enum class Operation {
  kMotorModel,
  kMotorAdrc,
  kPwmOffsets,
  kBatteryThreshold,
};

enum class MotorResponseMode {
  kExact,
  kMismatchedTicks,
};

enum class MotorAdrcResponseMode {
  kExact,
  kOkWithIncompleteMask,
  kFailureWithNonzeroMask,
};

constexpr std::array<std::int16_t, 4> kExpectedOffsets{10, -20, 30, -40};
constexpr std::array<float, 4> kExpectedKnownVelocityDecay{0.0F, 1.0F, 2.0F,
                                                           3.0F};
constexpr std::array<float, 4> kExpectedInputGain{0.03F, 0.031F, 0.032F,
                                                  0.033F};
constexpr std::array<float, 4> kExpectedControllerBandwidth{4.0F, 4.1F, 4.2F,
                                                            4.3F};
constexpr std::array<float, 4> kExpectedControllerFalExponent{0.1F, 0.4F,
                                                              0.7F, 1.0F};
constexpr std::array<float, 4> kExpectedControllerFalThreshold{0.001F, 0.1F,
                                                               1.0F, 6.0F};
constexpr std::array<float, 4> kExpectedObserverBandwidth{12.0F, 12.1F, 12.2F,
                                                          12.3F};
constexpr std::array<float, 4> kExpectedObserverVelocityFalExponent{
    1.0F, 0.8F, 0.6F, 0.4F};
constexpr std::array<float, 4> kExpectedObserverDisturbanceFalExponent{
    0.4F, 0.6F, 0.8F, 1.0F};
constexpr std::array<float, 4> kExpectedObserverFalThreshold{6.0F, 1.0F, 0.1F,
                                                             0.001F};
constexpr std::array<float, 4> kExpectedDisturbanceLeakage{0.0F, 10.0F, 20.0F,
                                                           50.0F};
constexpr std::array<float, 4> kExpectedDisturbanceLimit{30.0F, 20.0F, 10.0F,
                                                         0.0F};
constexpr std::array<float, 4> kExpectedFilterWeight{0.5F, 0.6F, 0.7F, 0.8F};
constexpr std::array<std::uint16_t, 4> kExpectedPositiveMinimumDrive{0U, 50U,
                                                                    100U,
                                                                    250U};
constexpr std::array<std::uint16_t, 4> kExpectedNegativeMinimumDrive{250U,
                                                                    100U, 50U,
                                                                    0U};
constexpr std::uint16_t kExpectedBatteryThresholdMv = 7000;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

std::vector<rclcpp::Parameter> ConfigurationOverrides() {
  return {
      rclcpp::Parameter("motor_model", "JGA27"),
      rclcpp::Parameter("known_velocity_decay_rate_s_inverse",
                        std::vector<double>{0.0, 1.0, 2.0, 3.0}),
      rclcpp::Parameter("input_gain_rps_per_second_per_permille",
                        std::vector<double>{0.03, 0.031, 0.032, 0.033}),
      rclcpp::Parameter("controller_bandwidth_rad_s",
                        std::vector<double>{4.0, 4.1, 4.2, 4.3}),
      rclcpp::Parameter("controller_fal_exponent",
                        std::vector<double>{0.1, 0.4, 0.7, 1.0}),
      rclcpp::Parameter("controller_fal_threshold_rps",
                        std::vector<double>{0.001, 0.1, 1.0, 6.0}),
      rclcpp::Parameter("observer_bandwidth_rad_s",
                        std::vector<double>{12.0, 12.1, 12.2, 12.3}),
      rclcpp::Parameter("observer_velocity_fal_exponent",
                        std::vector<double>{1.0, 0.8, 0.6, 0.4}),
      rclcpp::Parameter("observer_disturbance_fal_exponent",
                        std::vector<double>{0.4, 0.6, 0.8, 1.0}),
      rclcpp::Parameter("observer_fal_threshold_rps",
                        std::vector<double>{6.0, 1.0, 0.1, 0.001}),
      rclcpp::Parameter("disturbance_leakage_s_inverse",
                        std::vector<double>{0.0, 10.0, 20.0, 50.0}),
      rclcpp::Parameter("disturbance_estimate_limit_rps_per_second",
                        std::vector<double>{30.0, 20.0, 10.0, 0.0}),
      rclcpp::Parameter("velocity_filter_new_weight",
                        std::vector<double>{0.5, 0.6, 0.7, 0.8}),
      rclcpp::Parameter("positive_minimum_drive_permille",
                        std::vector<std::int64_t>{0, 50, 100, 250}),
      rclcpp::Parameter("negative_minimum_drive_permille",
                        std::vector<std::int64_t>{250, 100, 50, 0}),
      rclcpp::Parameter("pwm_servo_offsets_us",
                        std::vector<std::int64_t>{10, -20, 30, -40}),
      rclcpp::Parameter("battery_low_threshold_mv", std::int64_t{7000}),
  };
}

class ControllerStub {
 public:
  explicit ControllerStub(
      MotorResponseMode motor_response_mode = MotorResponseMode::kExact,
      MotorAdrcResponseMode motor_adrc_response_mode =
          MotorAdrcResponseMode::kExact)
      : node_(std::make_shared<rclcpp::Node>("controller_stub", "/mentor_pi")),
        motor_response_mode_(motor_response_mode),
        motor_adrc_response_mode_(motor_adrc_response_mode) {
    const auto reliable_depth_one =
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .durability_volatile();
    heartbeat_publisher_ =
        node_->create_publisher<Heartbeat>("heartbeat", reliable_depth_one);
    motion_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        "configuration/motion_enabled",
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          motion_gate_events_.push_back(message->data);
        });
    authorization_subscription_ =
        node_->create_subscription<std_msgs::msg::UInt64>(
            "configuration/motion_authorization",
            rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
                .reliable()
                .transient_local(),
            [this](std_msgs::msg::UInt64::ConstSharedPtr message) {
              authorization_events_.push_back(message->data);
            });

    motor_service_ = node_->create_service<SetMotorModel>(
        "motors/set_model",
        [this](const std::shared_ptr<SetMotorModel::Request> request,
               std::shared_ptr<SetMotorModel::Response> response) {
          operations_.push_back(Operation::kMotorModel);
          Expect(request->model == SetMotorModel::Request::MODEL_JGA27,
                 "motor service receives the configured JGA27 model");

          if (session_id_ == 2U && !second_session_busy_injected_) {
            second_session_busy_injected_ = true;
            response->result.code = Result::BUSY;
            return;
          }

          response->result.code = Result::OK;
          response->active_model = request->model;
          response->ticks_per_revolution =
              motor_response_mode_ == MotorResponseMode::kExact ? 1040U : 1041U;
          response->max_rps = 6.0F;
        });
    motor_adrc_service_ = node_->create_service<SetMotorAdrc>(
        "motors/set_adrc",
        [this](const std::shared_ptr<SetMotorAdrc::Request> request,
               std::shared_ptr<SetMotorAdrc::Response> response) {
          operations_.push_back(Operation::kMotorAdrc);
          Expect(request->update_mask == SetMotorAdrc::Request::ALL_MOTORS,
                 "NLADRC service receives the complete update mask");
          Expect(
              request->known_velocity_decay_rate_s_inverse ==
                      kExpectedKnownVelocityDecay &&
                  request->input_gain_rps_per_second_per_permille ==
                      kExpectedInputGain &&
                  request->controller_bandwidth_rad_s ==
                      kExpectedControllerBandwidth &&
                  request->controller_fal_exponent ==
                      kExpectedControllerFalExponent &&
                  request->controller_fal_threshold_rps ==
                      kExpectedControllerFalThreshold &&
                  request->observer_bandwidth_rad_s ==
                      kExpectedObserverBandwidth &&
                  request->observer_velocity_fal_exponent ==
                      kExpectedObserverVelocityFalExponent &&
                  request->observer_disturbance_fal_exponent ==
                      kExpectedObserverDisturbanceFalExponent &&
                  request->observer_fal_threshold_rps ==
                      kExpectedObserverFalThreshold &&
                  request->disturbance_leakage_s_inverse ==
                      kExpectedDisturbanceLeakage &&
                  request->disturbance_estimate_limit_rps_per_second ==
                      kExpectedDisturbanceLimit &&
                  request->velocity_filter_new_weight == kExpectedFilterWeight &&
                  request->positive_minimum_drive_permille ==
                      kExpectedPositiveMinimumDrive &&
                  request->negative_minimum_drive_permille ==
                      kExpectedNegativeMinimumDrive,
              "nonlinear ADRC service receives every configured array");
          response->result.code =
              motor_adrc_response_mode_ ==
                      MotorAdrcResponseMode::kFailureWithNonzeroMask
                  ? Result::INVALID_ARGUMENT
                  : Result::OK;
          response->applied_mask =
              motor_adrc_response_mode_ == MotorAdrcResponseMode::kExact
                  ? SetMotorAdrc::Request::ALL_MOTORS
                  : SetMotorAdrc::Request::MOTOR_1;
        });
    pwm_service_ = node_->create_service<SetPwmServoOffsets>(
        "pwm_servos/set_offsets",
        [this](const std::shared_ptr<SetPwmServoOffsets::Request> request,
               std::shared_ptr<SetPwmServoOffsets::Response> response) {
          operations_.push_back(Operation::kPwmOffsets);
          Expect(
              request->update_mask == SetPwmServoOffsets::Request::ALL_SERVOS,
              "PWM service receives the complete update mask");
          Expect(request->offset_us == kExpectedOffsets,
                 "PWM service receives all configured offsets");
          response->result.code = Result::OK;
          response->applied_mask = request->update_mask;
        });
    battery_service_ = node_->create_service<SetBatteryThreshold>(
        "battery/set_low_threshold",
        [this](const std::shared_ptr<SetBatteryThreshold::Request> request,
               std::shared_ptr<SetBatteryThreshold::Response> response) {
          operations_.push_back(Operation::kBatteryThreshold);
          Expect(request->threshold_mv == kExpectedBatteryThresholdMv,
                 "battery service receives the configured threshold");
          response->result.code = Result::OK;
          response->active_threshold_mv = request->threshold_mv;
        });
  }

  void PublishHeartbeat() {
    Heartbeat heartbeat;
    heartbeat.sequence = heartbeat_sequence_++;
    heartbeat.uptime_ms = heartbeat_sequence_ * 10U;
    heartbeat.agent_session_id = session_id_;
    heartbeat.state = Heartbeat::READY;
    heartbeat_publisher_->publish(heartbeat);
  }

  void SetSession(std::uint32_t session_id) {
    session_id_ = session_id;
    heartbeat_sequence_ = 0;
  }

  const std::shared_ptr<rclcpp::Node>& node() const { return node_; }
  const std::vector<Operation>& operations() const { return operations_; }
  const std::vector<bool>& motion_gate_events() const {
    return motion_gate_events_;
  }
  const std::vector<std::uint64_t>& authorization_events() const {
    return authorization_events_;
  }
  bool second_session_busy_injected() const {
    return second_session_busy_injected_;
  }

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      authorization_subscription_;
  rclcpp::Service<SetMotorModel>::SharedPtr motor_service_;
  rclcpp::Service<SetMotorAdrc>::SharedPtr motor_adrc_service_;
  rclcpp::Service<SetPwmServoOffsets>::SharedPtr pwm_service_;
  rclcpp::Service<SetBatteryThreshold>::SharedPtr battery_service_;
  std::vector<Operation> operations_;
  std::vector<bool> motion_gate_events_;
  std::vector<std::uint64_t> authorization_events_;
  std::uint32_t session_id_ = 1;
  std::uint32_t heartbeat_sequence_ = 0;
  bool second_session_busy_injected_ = false;
  MotorResponseMode motor_response_mode_;
  MotorAdrcResponseMode motor_adrc_response_mode_;
};

template <typename Predicate>
bool SpinUntil(rclcpp::executors::SingleThreadedExecutor* executor,
               ControllerStub* controller, Predicate predicate,
               std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    controller->PublishHeartbeat();
    executor->spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  executor->spin_some();
  return predicate();
}

void SpinFor(rclcpp::executors::SingleThreadedExecutor* executor,
             ControllerStub* controller, std::chrono::milliseconds duration) {
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    controller->PublishHeartbeat();
    executor->spin_some();
    std::this_thread::sleep_for(2ms);
  }
  executor->spin_some();
}

bool ContainsEventAfter(const std::vector<bool>& events, std::size_t start,
                        bool expected) {
  for (std::size_t index = start; index < events.size(); ++index) {
    if (events[index] == expected) {
      return true;
    }
  }
  return false;
}

void ExpectOperation(const std::vector<Operation>& operations,
                     std::size_t index, Operation expected,
                     const std::string& description) {
  Expect(index < operations.size(), description + ": operation is present");
  if (index < operations.size()) {
    Expect(operations[index] == expected,
           description + ": operation order is correct");
  }
}

void RunIntegrationTest() {
  rclcpp::NodeOptions options;
  options.parameter_overrides(ConfigurationOverrides());

  auto supervisor = mentor_pi_bringup::MakeConfigurationSupervisorNode(options);
  ControllerStub controller;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(supervisor);
  executor.add_node(controller.node());

  const bool first_session_ready = SpinUntil(
      &executor, &controller,
      [&controller]() {
        const auto& events = controller.motion_gate_events();
        const auto& authorization_events = controller.authorization_events();
        return controller.operations().size() >= 4U && !events.empty() &&
               events.back() && !authorization_events.empty() &&
               authorization_events.back() ==
                   ((UINT64_C(1) << 32U) | UINT64_C(1));
      },
      5s);
  Expect(first_session_ready,
         "first session opens a generation- and session-bound authorization");
  Expect(controller.operations().size() == 4U,
         "first session uses exactly four service calls");
  ExpectOperation(controller.operations(), 0, Operation::kMotorModel,
                  "first session motor model");
  ExpectOperation(controller.operations(), 1, Operation::kMotorAdrc,
                  "first session motor LADRC");
  ExpectOperation(controller.operations(), 2, Operation::kPwmOffsets,
                  "first session PWM offsets");
  ExpectOperation(controller.operations(), 3, Operation::kBatteryThreshold,
                  "first session battery threshold");

  const auto immutable_result = supervisor->set_parameter(
      rclcpp::Parameter("battery_low_threshold_mv", std::int64_t{8000}));
  Expect(!immutable_result.successful,
         "deployment parameters are immutable after startup");

  const std::size_t event_count_before_reconnect =
      controller.motion_gate_events().size();
  controller.SetSession(2);
  const bool retry_observed = SpinUntil(
      &executor, &controller,
      [&controller, event_count_before_reconnect]() {
        return controller.second_session_busy_injected() &&
               ContainsEventAfter(controller.motion_gate_events(),
                                  event_count_before_reconnect, false) &&
               !controller.authorization_events().empty() &&
               controller.authorization_events().back() == 0U;
      },
      2s);
  Expect(retry_observed,
         "new session closes the motion gate while BUSY is retried");

  const bool second_session_ready = SpinUntil(
      &executor, &controller,
      [&controller, event_count_before_reconnect]() {
        return controller.operations().size() >= 9U &&
               ContainsEventAfter(controller.motion_gate_events(),
                                  event_count_before_reconnect, true) &&
               !controller.authorization_events().empty() &&
               controller.authorization_events().back() ==
                   ((UINT64_C(2) << 32U) | UINT64_C(2));
      },
      5s);
  Expect(second_session_ready,
         "second controller session reapplies configuration and reopens gate");
  Expect(controller.operations().size() == 9U,
         "second session has one bounded BUSY retry plus four successes");
  ExpectOperation(controller.operations(), 4, Operation::kMotorModel,
                  "second session BUSY motor attempt");
  ExpectOperation(controller.operations(), 5, Operation::kMotorModel,
                  "second session successful motor retry");
  ExpectOperation(controller.operations(), 6, Operation::kMotorAdrc,
                  "second session motor LADRC");
  ExpectOperation(controller.operations(), 7, Operation::kPwmOffsets,
                  "second session PWM offsets");
  ExpectOperation(controller.operations(), 8, Operation::kBatteryThreshold,
                  "second session battery threshold");

  executor.remove_node(controller.node());
  executor.remove_node(supervisor);
}

void RunInconsistentMotorProfileTest() {
  rclcpp::NodeOptions options;
  options.parameter_overrides(ConfigurationOverrides());

  auto supervisor = mentor_pi_bringup::MakeConfigurationSupervisorNode(options);
  ControllerStub controller{MotorResponseMode::kMismatchedTicks};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(supervisor);
  executor.add_node(controller.node());

  const bool motor_response_observed = SpinUntil(
      &executor, &controller,
      [&controller]() { return !controller.operations().empty(); }, 2s);
  Expect(motor_response_observed,
         "inconsistent-profile test receives the motor-model request");
  SpinFor(&executor, &controller, 300ms);

  Expect(controller.operations().size() == 1U,
         "inconsistent OK profile prevents later configuration calls");
  Expect(!controller.motion_gate_events().empty(),
         "inconsistent OK profile publishes the closed motion gate");
  for (const bool enabled : controller.motion_gate_events()) {
    Expect(!enabled,
           "inconsistent OK profile never publishes an enabled motion gate");
  }
  Expect(!controller.authorization_events().empty(),
         "inconsistent OK profile publishes disabled authorization");
  for (const std::uint64_t authorization : controller.authorization_events()) {
    Expect(authorization == 0U,
           "inconsistent OK profile never publishes motion authorization");
  }

  executor.remove_node(controller.node());
  executor.remove_node(supervisor);
}

void RunAdrcAppliedMaskMismatchTest(MotorAdrcResponseMode response_mode,
                                    const std::string& description) {
  rclcpp::NodeOptions options;
  options.parameter_overrides(ConfigurationOverrides());

  auto supervisor = mentor_pi_bringup::MakeConfigurationSupervisorNode(options);
  ControllerStub controller{MotorResponseMode::kExact, response_mode};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(supervisor);
  executor.add_node(controller.node());
  const bool adrc_response_observed = SpinUntil(
      &executor, &controller,
      [&controller]() { return controller.operations().size() >= 2U; }, 2s);
  Expect(adrc_response_observed, description + ": receives the LADRC request");
  SpinFor(&executor, &controller, 300ms);
  Expect(controller.operations().size() == 2U,
         description + ": prevents later configuration calls");
  for (const bool enabled : controller.motion_gate_events()) {
    Expect(!enabled, description + ": never enables motion");
  }
  executor.remove_node(controller.node());
  executor.remove_node(supervisor);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    RunIntegrationTest();
    RunInconsistentMotorProfileTest();
    RunAdrcAppliedMaskMismatchTest(MotorAdrcResponseMode::kOkWithIncompleteMask,
                                   "LADRC OK with incomplete applied mask");
    RunAdrcAppliedMaskMismatchTest(
        MotorAdrcResponseMode::kFailureWithNonzeroMask,
        "LADRC failure with nonzero applied mask");
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    ++g_failures;
  }
  rclcpp::shutdown();
  if (g_failures == 0) {
    std::cout << "configuration supervisor ROS integration test passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
