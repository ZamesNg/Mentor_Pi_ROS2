// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef MENTOR_PI_TRACKING__TRACKER_NODE_HPP_
#define MENTOR_PI_TRACKING__TRACKER_NODE_HPP_

#include <array>
#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mentor_pi_tracking/mpc_solver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentor_pi::tracking {

std::optional<std::array<double, 3>> GeometryCenterPoseState(
    const geometry_msgs::msg::PoseStamped& pose);

rclcpp::Node::SharedPtr MakeTrackerNode(const rclcpp::NodeOptions& options);

}  // namespace mentor_pi::tracking

#endif  // MENTOR_PI_TRACKING__TRACKER_NODE_HPP_
