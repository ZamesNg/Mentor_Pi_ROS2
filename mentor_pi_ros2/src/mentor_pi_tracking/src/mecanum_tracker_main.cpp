// SPDX-License-Identifier: GPL-2.0-or-later

#include "mentor_pi_tracking/tracker_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = mentor_pi::tracking::MakeTrackerNode(
      mentor_pi::tracking::VehicleType::kMecanum, rclcpp::NodeOptions{});
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},
                                                    2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
