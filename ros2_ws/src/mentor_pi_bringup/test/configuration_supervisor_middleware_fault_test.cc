// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "mentor_pi_bringup/configuration_supervisor_node.h"
#include "mentor_pi_interfaces/motor_profile_contract.hpp"
#include "mentor_pi_interfaces/msg/heartbeat.hpp"
#include "mentor_pi_interfaces/msg/result.hpp"
#include "mentor_pi_interfaces/srv/set_battery_threshold.hpp"
#include "mentor_pi_interfaces/srv/set_motor_adrc.hpp"
#include "mentor_pi_interfaces/srv/set_motor_model.hpp"
#include "mentor_pi_interfaces/srv/set_pwm_servo_offsets.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
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

constexpr std::size_t kFirstWithheldMotorRequest = 1U;
constexpr std::size_t kSessionChangeWithheldMotorRequest = 3U;
constexpr std::uint64_t kFirstAuthorization =
    (UINT64_C(1) << 32U) | UINT64_C(1);
constexpr std::uint64_t kThirdAuthorization =
    (UINT64_C(3) << 32U) | UINT64_C(3);

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

struct ControllerObservation {
  std::size_t motor_requests = 0;
  std::size_t motor_adrc_requests = 0;
  std::size_t pwm_requests = 0;
  std::size_t battery_requests = 0;
  std::size_t gate_events = 0;
  std::size_t authorization_events = 0;
  bool gate_enabled = false;
  std::uint64_t authorization = 0;
};

class WithholdingControllerPeer {
 public:
  WithholdingControllerPeer()
      : node_(std::make_shared<rclcpp::Node>("middleware_fault_controller_peer",
                                             "/mentor_pi")),
        service_callback_group_(node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant)) {
    const auto reliable_depth_one =
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .durability_volatile();
    heartbeat_publisher_ =
        node_->create_publisher<Heartbeat>("heartbeat", reliable_depth_one);
    gate_subscription_ = node_->create_subscription<std_msgs::msg::Bool>(
        "configuration/motion_enabled",
        rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
            .reliable()
            .transient_local(),
        [this](std_msgs::msg::Bool::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          gate_events_.push_back(message->data);
        });
    authorization_subscription_ =
        node_->create_subscription<std_msgs::msg::UInt64>(
            "configuration/motion_authorization",
            rclcpp::QoS{rclcpp::KeepLast{std::size_t{1}}}
                .reliable()
                .transient_local(),
            [this](std_msgs::msg::UInt64::ConstSharedPtr message) {
              std::lock_guard<std::mutex> lock(mutex_);
              authorization_events_.push_back(message->data);
            });

    motor_service_ = node_->create_service<SetMotorModel>(
        "motors/set_model",
        [this](const std::shared_ptr<SetMotorModel::Request> request,
               std::shared_ptr<SetMotorModel::Response> response) {
          HandleMotorRequest(*request, response.get());
        },
        rmw_qos_profile_services_default, service_callback_group_);
    motor_adrc_service_ = node_->create_service<SetMotorAdrc>(
        "motors/set_adrc",
        [this](const std::shared_ptr<SetMotorAdrc::Request>,
               std::shared_ptr<SetMotorAdrc::Response> response) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            ++motor_adrc_requests_;
          }
          response->result.code = Result::OK;
          response->applied_mask = SetMotorAdrc::Request::ALL_MOTORS;
        },
        rmw_qos_profile_services_default, service_callback_group_);
    pwm_service_ = node_->create_service<SetPwmServoOffsets>(
        "pwm_servos/set_offsets",
        [this](const std::shared_ptr<SetPwmServoOffsets::Request>,
               std::shared_ptr<SetPwmServoOffsets::Response> response) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            ++pwm_requests_;
          }
          response->result.code = Result::OK;
          response->applied_mask = SetPwmServoOffsets::Request::ALL_SERVOS;
        },
        rmw_qos_profile_services_default, service_callback_group_);
    battery_service_ = node_->create_service<SetBatteryThreshold>(
        "battery/set_low_threshold",
        [this](const std::shared_ptr<SetBatteryThreshold::Request> request,
               std::shared_ptr<SetBatteryThreshold::Response> response) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            ++battery_requests_;
          }
          response->result.code = Result::OK;
          response->active_threshold_mv = request->threshold_mv;
        },
        rmw_qos_profile_services_default, service_callback_group_);
  }

  WithholdingControllerPeer(const WithholdingControllerPeer&) = delete;
  WithholdingControllerPeer& operator=(const WithholdingControllerPeer&) =
      delete;

  ~WithholdingControllerPeer() { ReleaseAllResponses(); }

  void PublishHeartbeat() {
    Heartbeat heartbeat;
    heartbeat.sequence = heartbeat_sequence_++;
    heartbeat.uptime_ms = heartbeat_sequence_ * 5U;
    heartbeat.agent_session_id = session_id_;
    heartbeat.state = Heartbeat::READY;
    heartbeat_publisher_->publish(heartbeat);
  }

  void SetSession(std::uint32_t session_id) {
    session_id_ = session_id;
    heartbeat_sequence_ = 0;
  }

  void ReleaseResponse(std::size_t request_number) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request_number == kFirstWithheldMotorRequest) {
        release_first_response_ = true;
      } else if (request_number == kSessionChangeWithheldMotorRequest) {
        release_session_change_response_ = true;
      }
    }
    response_release_.notify_all();
  }

  void ReleaseAllResponses() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      release_first_response_ = true;
      release_session_change_response_ = true;
    }
    response_release_.notify_all();
  }

  ControllerObservation Observe() const {
    std::lock_guard<std::mutex> lock(mutex_);
    ControllerObservation observation;
    observation.motor_requests = motor_requests_;
    observation.motor_adrc_requests = motor_adrc_requests_;
    observation.pwm_requests = pwm_requests_;
    observation.battery_requests = battery_requests_;
    observation.gate_events = gate_events_.size();
    observation.authorization_events = authorization_events_.size();
    if (!gate_events_.empty()) {
      observation.gate_enabled = gate_events_.back();
    }
    if (!authorization_events_.empty()) {
      observation.authorization = authorization_events_.back();
    }
    return observation;
  }

  std::size_t completed_motor_responses() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return completed_motor_responses_;
  }

  const std::shared_ptr<rclcpp::Node>& node() const { return node_; }

 private:
  bool ResponseWasReleased(std::size_t request_number) const {
    if (request_number == kFirstWithheldMotorRequest) {
      return release_first_response_;
    }
    if (request_number == kSessionChangeWithheldMotorRequest) {
      return release_session_change_response_;
    }
    return true;
  }

  void HandleMotorRequest(const SetMotorModel::Request& request,
                          SetMotorModel::Response* response) {
    std::size_t request_number = 0;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      request_number = ++motor_requests_;
      response_release_.wait(lock, [this, request_number]() {
        return ResponseWasReleased(request_number);
      });
    }

    const auto* profile =
        mentor_pi_interfaces::FindMotorProfileContract(request.model);
    Expect(profile != nullptr,
           "middleware fault peer receives a supported motor model");
    if (profile == nullptr) {
      response->result.code = Result::INVALID_ARGUMENT;
      return;
    }

    response->result.code = Result::OK;
    response->active_model = request.model;
    response->ticks_per_revolution = profile->ticks_per_revolution;
    response->max_rps = profile->max_rps;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++completed_motor_responses_;
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeat_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gate_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr
      authorization_subscription_;
  rclcpp::Service<SetMotorModel>::SharedPtr motor_service_;
  rclcpp::Service<SetMotorAdrc>::SharedPtr motor_adrc_service_;
  rclcpp::Service<SetPwmServoOffsets>::SharedPtr pwm_service_;
  rclcpp::Service<SetBatteryThreshold>::SharedPtr battery_service_;

  mutable std::mutex mutex_;
  std::condition_variable response_release_;
  std::vector<bool> gate_events_;
  std::vector<std::uint64_t> authorization_events_;
  std::size_t motor_requests_ = 0;
  std::size_t motor_adrc_requests_ = 0;
  std::size_t pwm_requests_ = 0;
  std::size_t battery_requests_ = 0;
  std::size_t completed_motor_responses_ = 0;
  std::uint32_t session_id_ = 1;
  std::uint32_t heartbeat_sequence_ = 0;
  bool release_first_response_ = false;
  bool release_session_change_response_ = false;
};

class ScopedExecutorThread {
 public:
  ScopedExecutorThread(rclcpp::executors::MultiThreadedExecutor* executor,
                       WithholdingControllerPeer* controller)
      : executor_(executor),
        controller_(controller),
        thread_([this]() { executor_->spin(); }) {}

  ScopedExecutorThread(const ScopedExecutorThread&) = delete;
  ScopedExecutorThread& operator=(const ScopedExecutorThread&) = delete;

  ~ScopedExecutorThread() {
    controller_->ReleaseAllResponses();
    executor_->cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  rclcpp::executors::MultiThreadedExecutor* executor_;
  WithholdingControllerPeer* controller_;
  std::thread thread_;
};

template <typename Predicate>
bool PublishUntil(WithholdingControllerPeer* controller, Predicate predicate,
                  std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    controller->PublishHeartbeat();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  controller->PublishHeartbeat();
  return predicate();
}

void PublishFor(WithholdingControllerPeer* controller,
                std::chrono::milliseconds duration) {
  static_cast<void>(PublishUntil(
      controller, []() { return false; }, duration));
}

bool IsAuthorized(const WithholdingControllerPeer& controller,
                  std::uint64_t expected_authorization) {
  const ControllerObservation observation = controller.Observe();
  return observation.gate_enabled &&
         observation.authorization == expected_authorization;
}

void ExpectStableAfterLateResponse(const ControllerObservation& before,
                                   const ControllerObservation& after,
                                   const std::string& description) {
  Expect(after.motor_requests == before.motor_requests,
         description + ": no additional motor request");
  Expect(after.motor_adrc_requests == before.motor_adrc_requests,
         description + ": no additional LADRC request");
  Expect(after.pwm_requests == before.pwm_requests,
         description + ": no additional PWM request");
  Expect(after.battery_requests == before.battery_requests,
         description + ": no additional battery request");
  Expect(after.gate_events == before.gate_events,
         description + ": no motion-gate transition");
  Expect(after.authorization_events == before.authorization_events,
         description + ": no authorization transition");
  Expect(after.gate_enabled == before.gate_enabled,
         description + ": motion-gate value remains stable");
  Expect(after.authorization == before.authorization,
         description + ": authorization token remains stable");
}

void RunMiddlewareFaultTest() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("motor_model", "JGA27"),
      rclcpp::Parameter("known_velocity_decay_rate_s_inverse",
                        std::vector<double>{0.0, 0.0, 0.0, 0.0}),
      rclcpp::Parameter("input_gain_rps_per_second_per_permille",
                        std::vector<double>{0.03, 0.03, 0.03, 0.03}),
      rclcpp::Parameter("controller_bandwidth_rad_s",
                        std::vector<double>{4.0, 4.0, 4.0, 4.0}),
      rclcpp::Parameter("controller_fal_exponent",
                        std::vector<double>{1.0, 1.0, 1.0, 1.0}),
      rclcpp::Parameter("controller_fal_threshold_rps",
                        std::vector<double>{0.1, 0.1, 0.1, 0.1}),
      rclcpp::Parameter("observer_bandwidth_rad_s",
                        std::vector<double>{12.0, 12.0, 12.0, 12.0}),
      rclcpp::Parameter("observer_velocity_fal_exponent",
                        std::vector<double>{1.0, 1.0, 1.0, 1.0}),
      rclcpp::Parameter("observer_disturbance_fal_exponent",
                        std::vector<double>{1.0, 1.0, 1.0, 1.0}),
      rclcpp::Parameter("observer_fal_threshold_rps",
                        std::vector<double>{0.1, 0.1, 0.1, 0.1}),
      rclcpp::Parameter("disturbance_leakage_s_inverse",
                        std::vector<double>{0.0, 0.0, 0.0, 0.0}),
      rclcpp::Parameter("disturbance_estimate_limit_rps_per_second",
                        std::vector<double>{30.0, 30.0, 30.0, 30.0}),
      rclcpp::Parameter("velocity_filter_new_weight",
                        std::vector<double>{0.8, 0.8, 0.8, 0.8}),
      rclcpp::Parameter("positive_minimum_drive_permille",
                        std::vector<std::int64_t>{0, 0, 0, 0}),
      rclcpp::Parameter("negative_minimum_drive_permille",
                        std::vector<std::int64_t>{0, 0, 0, 0}),
      rclcpp::Parameter("pwm_servo_offsets_us",
                        std::vector<std::int64_t>{0, 0, 0, 0}),
      rclcpp::Parameter("battery_low_threshold_mv", std::int64_t{6300}),
  });

  auto supervisor = mentor_pi_bringup::MakeConfigurationSupervisorNode(options);
  WithholdingControllerPeer controller;
  rclcpp::executors::MultiThreadedExecutor executor{rclcpp::ExecutorOptions{},
                                                    std::size_t{4}};
  executor.add_node(supervisor);
  executor.add_node(controller.node());

  {
    ScopedExecutorThread executor_thread(&executor, &controller);

    const bool first_request_withheld = PublishUntil(
        &controller,
        [&controller]() {
          return controller.Observe().motor_requests >=
                 kFirstWithheldMotorRequest;
        },
        2s);
    Expect(first_request_withheld,
           "first real-RMW motor-model request reaches the withholding peer");
    const ControllerObservation while_withheld = controller.Observe();
    Expect(!while_withheld.gate_enabled && while_withheld.authorization == 0U,
           "withheld service reply keeps motion authorization closed");

    const bool timeout_retry_completed = PublishUntil(
        &controller,
        [&controller]() {
          return controller.Observe().motor_requests >= 2U &&
                 IsAuthorized(controller, kFirstAuthorization);
        },
        3s);
    Expect(timeout_retry_completed,
           "a 100 ms client timeout removes the pending request and the real-"
           "RMW retry completes configuration");
    const ControllerObservation before_first_late_reply = controller.Observe();
    Expect(before_first_late_reply.motor_requests == 2U &&
               before_first_late_reply.motor_adrc_requests == 1U &&
               before_first_late_reply.pwm_requests == 1U &&
               before_first_late_reply.battery_requests == 1U,
           "timeout recovery performs one motor retry and one ordered call for "
           "each remaining service");

    controller.ReleaseResponse(kFirstWithheldMotorRequest);
    const bool first_late_response_completed = PublishUntil(
        &controller,
        [&controller]() {
          return controller.completed_motor_responses() >= 2U;
        },
        1s);
    Expect(first_late_response_completed,
           "the peer releases the formerly withheld real service response");
    PublishFor(&controller, 200ms);
    ExpectStableAfterLateResponse(before_first_late_reply, controller.Observe(),
                                  "post-timeout stale reply");

    controller.SetSession(2U);
    const bool session_two_request_withheld = PublishUntil(
        &controller,
        [&controller]() {
          return controller.Observe().motor_requests >=
                 kSessionChangeWithheldMotorRequest;
        },
        2s);
    Expect(session_two_request_withheld,
           "new session sends a second intentionally withheld request");
    const bool session_two_gate_closed = PublishUntil(
        &controller,
        [&controller]() {
          const ControllerObservation observation = controller.Observe();
          return !observation.gate_enabled && observation.authorization == 0U;
        },
        1s);
    Expect(session_two_gate_closed,
           "new session closes authorization before reconfiguration");

    controller.SetSession(3U);
    const bool third_session_ready = PublishUntil(
        &controller,
        [&controller]() {
          return controller.Observe().motor_requests >= 4U &&
                 IsAuthorized(controller, kThirdAuthorization);
        },
        3s);
    Expect(third_session_ready,
           "session change removes the withheld pending request and generation "
           "three configures through real middleware");
    const ControllerObservation before_session_late_reply =
        controller.Observe();
    Expect(before_session_late_reply.motor_requests == 4U &&
               before_session_late_reply.motor_adrc_requests == 2U &&
               before_session_late_reply.pwm_requests == 2U &&
               before_session_late_reply.battery_requests == 2U,
           "abandoned session does not advance beyond its withheld motor call");

    controller.ReleaseResponse(kSessionChangeWithheldMotorRequest);
    const bool session_late_response_completed = PublishUntil(
        &controller,
        [&controller]() {
          return controller.completed_motor_responses() >= 4U;
        },
        1s);
    Expect(session_late_response_completed,
           "the peer releases the old-session service response");
    PublishFor(&controller, 200ms);
    ExpectStableAfterLateResponse(before_session_late_reply,
                                  controller.Observe(),
                                  "old-session stale reply");
  }

  executor.remove_node(controller.node());
  executor.remove_node(supervisor);
}

bool SelectIsolatedRosDomain() {
  const int domain_id = 150 + static_cast<int>(getpid() % 50);
  const std::string domain = std::to_string(domain_id);
  return setenv("ROS_DOMAIN_ID", domain.c_str(), 1) == 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (!SelectIsolatedRosDomain()) {
    std::cerr << "FAIL: could not select an isolated ROS domain\n";
    return 2;
  }
  rclcpp::init(argc, argv);
  try {
    RunMiddlewareFaultTest();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    ++g_failures;
  }
  rclcpp::shutdown();
  if (g_failures == 0) {
    std::cout << "configuration supervisor middleware fault test passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
