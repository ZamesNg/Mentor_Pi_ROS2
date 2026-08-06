// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__MOTOR_COMMISSIONING_NODE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__MOTOR_COMMISSIONING_NODE_H_

#include <atomic>
#include <csignal>
#include <cstdint>
#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace mentor_pi_bringup {

struct MotorCommissioningOutcome {
  std::atomic<bool> complete{false};
  std::atomic<bool> passed{false};
  std::atomic<std::uint8_t> failure{0U};
};

class MotorCommissioningStopControl {
 public:
  virtual ~MotorCommissioningStopControl() = default;

  // Requests the normal post-stop state-machine path and makes an immediate
  // best-effort all-motor zero publication. False means the immediate
  // publication failed and the MCU lease is the remaining stop mechanism.
  virtual bool RequestStop() noexcept = 0;
};

struct MotorCommissioningInstance {
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<MotorCommissioningOutcome> outcome;
  std::shared_ptr<MotorCommissioningStopControl> stop_control;
};

MotorCommissioningInstance MakeMotorCommissioningNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions{},
    const volatile std::sig_atomic_t* stop_requested = nullptr);

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__MOTOR_COMMISSIONING_NODE_H_
