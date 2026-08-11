// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <exception>
#include <memory>

#include "mentor_pi_bringup/configuration_supervisor_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    const auto node = mentor_pi_bringup::MakeConfigurationSupervisorNode();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("configuration_supervisor"),
                 "fatal supervisor exception: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
