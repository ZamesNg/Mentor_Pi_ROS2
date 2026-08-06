// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <exception>

#include "mentor_pi_bringup/qualification_monitor_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  int exit_code = 1;
  try {
    const auto instance = mentor_pi_bringup::MakeQualificationMonitorNode();
    rclcpp::spin(instance.node);
    if (instance.outcome->complete.load(std::memory_order_acquire) &&
        instance.outcome->passed.load(std::memory_order_acquire)) {
      exit_code = 0;
    }
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("qualification_monitor"),
                 "fatal qualification-monitor exception: %s", error.what());
  }
  rclcpp::shutdown();
  return exit_code;
}
