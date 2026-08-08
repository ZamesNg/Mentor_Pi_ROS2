// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__QUALIFICATION_MONITOR_NODE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__QUALIFICATION_MONITOR_NODE_H_

#include <atomic>
#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace mentor_pi_bringup {

struct QualificationMonitorOutcome {
  std::atomic<bool> complete{false};
  std::atomic<bool> passed{false};
};

struct QualificationMonitorInstance {
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<QualificationMonitorOutcome> outcome;
};

QualificationMonitorInstance MakeQualificationMonitorNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__QUALIFICATION_MONITOR_NODE_H_
