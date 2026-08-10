// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef MENTOR_PI_TRACKING__TRACKER_NODE_HPP_
#define MENTOR_PI_TRACKING__TRACKER_NODE_HPP_

#include "mentor_pi_tracking/mpc_solver.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mentor_pi::tracking {

rclcpp::Node::SharedPtr MakeTrackerNode(VehicleType vehicle,
                                        const rclcpp::NodeOptions& options);

}  // namespace mentor_pi::tracking

#endif  // MENTOR_PI_TRACKING__TRACKER_NODE_HPP_
