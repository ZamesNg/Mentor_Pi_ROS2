// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <csignal>
#include <exception>
#include <memory>
#include <thread>
#include <utility>

#include "mentor_pi_bringup/motor_commissioning_node.h"
#include "rclcpp/rclcpp.hpp"

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void RequestSafeStop(int signal_number) {
  static_cast<void>(signal_number);
  g_stop_requested = 1;
}

class AbnormalStopGuard final {
 public:
  explicit AbnormalStopGuard(
      std::shared_ptr<mentor_pi_bringup::MotorCommissioningStopControl> control)
      : control_(std::move(control)) {}

  ~AbnormalStopGuard() {
    if (armed_ && control_ != nullptr) {
      static_cast<void>(control_->RequestStop());
    }
  }

  AbnormalStopGuard(const AbnormalStopGuard&) = delete;
  AbnormalStopGuard& operator=(const AbnormalStopGuard&) = delete;

  void Disarm() { armed_ = false; }

 private:
  std::shared_ptr<mentor_pi_bringup::MotorCommissioningStopControl> control_;
  bool armed_ = true;
};

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv, rclcpp::InitOptions{},
               rclcpp::SignalHandlerOptions::None);
  if (std::signal(SIGINT, RequestSafeStop) == SIG_ERR ||
      std::signal(SIGTERM, RequestSafeStop) == SIG_ERR) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_commissioning"),
                 "failed to install safe-stop signal handlers");
    rclcpp::shutdown();
    return 1;
  }

  int exit_code = 1;
  try {
    const auto instance = mentor_pi_bringup::MakeMotorCommissioningNode(
        rclcpp::NodeOptions{}, &g_stop_requested);
    AbnormalStopGuard stop_guard(instance.stop_control);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(instance.node);
    while (rclcpp::ok() &&
           !instance.outcome->complete.load(std::memory_order_acquire)) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds{5});
    }
    if (instance.outcome->complete.load(std::memory_order_acquire) &&
        instance.outcome->passed.load(std::memory_order_acquire)) {
      exit_code = 0;
    }
    if (instance.outcome->complete.load(std::memory_order_acquire)) {
      stop_guard.Disarm();
    } else {
      const bool zero_published = instance.stop_control->RequestStop();
      if (!zero_published) {
        RCLCPP_ERROR(
            rclcpp::get_logger("motor_commissioning"),
            "abnormal ROS shutdown prevented the immediate zero command; "
            "the MCU's independent 200 ms lease is the remaining stop path");
      }
      stop_guard.Disarm();
    }
    executor.remove_node(instance.node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_commissioning"),
                 "fatal motor-commissioning exception: %s; an immediate "
                 "best-effort zero was requested, and the MCU's independent "
                 "200 ms lease remains the fallback",
                 error.what());
  }
  rclcpp::shutdown();
  return exit_code;
}
