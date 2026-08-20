#ifndef MENTOR_PI_HARDWARES__ODOMETRY_TRANSFORM_HPP_
#define MENTOR_PI_HARDWARES__ODOMETRY_TRANSFORM_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

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

geometry_msgs::msg::PoseStamped GeometryCenterPoseFromOdometry(
    const nav_msgs::msg::Odometry& source_odometry,
    double source_to_geometry_center_m,
    const std::string& geometry_center_frame_id,
    PlanarTransform output_from_source = {},
    const std::string& output_frame_id = "map");

geometry_msgs::msg::PoseStamped ValidateGeometryCenterPose(
    const geometry_msgs::msg::PoseStamped& source_pose);

geometry_msgs::msg::TransformStamped GeometryCenterPoseToTransform(
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& geometry_center_frame_id);

}  // namespace mentor_pi::hardware

#endif  // MENTOR_PI_HARDWARES__ODOMETRY_TRANSFORM_HPP_
