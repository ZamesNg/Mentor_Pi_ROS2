#ifndef MENTOR_PI_HARDWARES__ODOMETRY_TRANSFORM_HPP_
#define MENTOR_PI_HARDWARES__ODOMETRY_TRANSFORM_HPP_

#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace mentor_pi::hardware {

struct PlanarTransform {
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
};

nav_msgs::msg::Odometry ToGeometryCenterOdometry(
    const nav_msgs::msg::Odometry& source_odometry,
    double source_to_geometry_center_m,
    const std::string& geometry_center_frame_id,
    PlanarTransform output_from_source = {},
    const std::string& output_odom_frame_id = "");

tf2_msgs::msg::TFMessage ToGeometryCenterTf(
    const tf2_msgs::msg::TFMessage& source_tf,
    double source_to_geometry_center_m,
    const std::string& geometry_center_frame_id,
    PlanarTransform output_from_source = {},
    const std::string& output_odom_frame_id = "");

}  // namespace mentor_pi::hardware

#endif  // MENTOR_PI_HARDWARES__ODOMETRY_TRANSFORM_HPP_
