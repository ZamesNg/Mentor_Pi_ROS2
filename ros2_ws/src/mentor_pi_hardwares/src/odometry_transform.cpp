#include "mentor_pi_hardwares/odometry_transform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace mentor_pi::hardware {
namespace {

using Covariance = std::array<double, 36>;

double Yaw(const geometry_msgs::msg::Quaternion& orientation) {
  const std::array<double, 4> elements{
      {orientation.x, orientation.y, orientation.z, orientation.w}};
  for (const double element : elements) {
    if (!std::isfinite(element)) {
      throw std::invalid_argument("orientation must be finite");
    }
  }
  const double norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
    throw std::invalid_argument("orientation must be nonzero");
  }
  return std::atan2(
      2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
      orientation.w * orientation.w + orientation.x * orientation.x -
          orientation.y * orientation.y - orientation.z * orientation.z);
}

geometry_msgs::msg::Quaternion RotateOrientation(
    const geometry_msgs::msg::Quaternion& orientation, double yaw) {
  const double sine = std::sin(0.5 * yaw);
  const double cosine = std::cos(0.5 * yaw);
  geometry_msgs::msg::Quaternion output;
  output.x = cosine * orientation.x - sine * orientation.y;
  output.y = sine * orientation.x + cosine * orientation.y;
  output.z = cosine * orientation.z + sine * orientation.w;
  output.w = cosine * orientation.w - sine * orientation.z;
  return output;
}

Covariance TransformCovariance(const Covariance& input,
                               const std::array<double, 36>& jacobian) {
  Covariance intermediate{};
  Covariance output{};
  for (std::size_t row = 0; row < 6U; ++row) {
    for (std::size_t column = 0; column < 6U; ++column) {
      for (std::size_t index = 0; index < 6U; ++index) {
        intermediate[row * 6U + column] +=
            jacobian[row * 6U + index] * input[index * 6U + column];
      }
    }
  }
  for (std::size_t row = 0; row < 6U; ++row) {
    for (std::size_t column = 0; column < 6U; ++column) {
      for (std::size_t index = 0; index < 6U; ++index) {
        output[row * 6U + column] +=
            intermediate[row * 6U + index] * jacobian[column * 6U + index];
      }
    }
  }
  return output;
}

std::array<double, 36> IdentityJacobian() {
  std::array<double, 36> jacobian{};
  for (std::size_t index = 0; index < 6U; ++index) {
    jacobian[index * 6U + index] = 1.0;
  }
  return jacobian;
}

void Validate(double offset, const std::string& frame_id,
              const PlanarTransform& output_from_source) {
  if (!std::isfinite(offset) || offset < 0.0) {
    throw std::invalid_argument(
        "source_to_geometry_center_m must be nonnegative and finite");
  }
  if (frame_id.empty()) {
    throw std::invalid_argument("geometry_center_frame_id must not be empty");
  }
  if (!std::isfinite(output_from_source.x_m) ||
      !std::isfinite(output_from_source.y_m) ||
      !std::isfinite(output_from_source.yaw_rad)) {
    throw std::invalid_argument("output odometry transform must be finite");
  }
}

void ValidateOdometry(const nav_msgs::msg::Odometry& odometry) {
  const std::array<double, 9> values{{
      odometry.pose.pose.position.x,
      odometry.pose.pose.position.y,
      odometry.pose.pose.position.z,
      odometry.twist.twist.linear.x,
      odometry.twist.twist.linear.y,
      odometry.twist.twist.linear.z,
      odometry.twist.twist.angular.x,
      odometry.twist.twist.angular.y,
      odometry.twist.twist.angular.z,
  }};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(
          "odometry pose and planar twist must be finite");
    }
  }
  for (const double value : odometry.pose.covariance) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("odometry pose covariance must be finite");
    }
  }
  for (const double value : odometry.twist.covariance) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("odometry twist covariance must be finite");
    }
  }
}

std::array<double, 36> PlanarRotationJacobian(double yaw) {
  auto jacobian = IdentityJacobian();
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (const std::size_t offset : {0U, 3U}) {
    jacobian[(offset + 0U) * 6U + offset + 0U] = cosine;
    jacobian[(offset + 0U) * 6U + offset + 1U] = -sine;
    jacobian[(offset + 1U) * 6U + offset + 0U] = sine;
    jacobian[(offset + 1U) * 6U + offset + 1U] = cosine;
  }
  return jacobian;
}

void TransformPosition(geometry_msgs::msg::Point* position,
                       const PlanarTransform& transform) {
  const double source_x = position->x;
  const double source_y = position->y;
  const double cosine = std::cos(transform.yaw_rad);
  const double sine = std::sin(transform.yaw_rad);
  position->x = transform.x_m + cosine * source_x - sine * source_y;
  position->y = transform.y_m + sine * source_x + cosine * source_y;
}

void TransformTranslation(geometry_msgs::msg::Vector3* translation,
                          const PlanarTransform& transform) {
  const double source_x = translation->x;
  const double source_y = translation->y;
  const double cosine = std::cos(transform.yaw_rad);
  const double sine = std::sin(transform.yaw_rad);
  translation->x = transform.x_m + cosine * source_x - sine * source_y;
  translation->y = transform.y_m + sine * source_x + cosine * source_y;
}

}  // namespace

nav_msgs::msg::Odometry ToGeometryCenterOdometry(
    const nav_msgs::msg::Odometry& source_odometry,
    double source_to_geometry_center_m,
    const std::string& geometry_center_frame_id,
    PlanarTransform output_from_source,
    const std::string& output_odom_frame_id) {
  Validate(source_to_geometry_center_m, geometry_center_frame_id,
           output_from_source);
  ValidateOdometry(source_odometry);
  nav_msgs::msg::Odometry output = source_odometry;
  if (!output_odom_frame_id.empty()) {
    output.header.frame_id = output_odom_frame_id;
  }
  const double yaw = Yaw(source_odometry.pose.pose.orientation);
  output.child_frame_id = geometry_center_frame_id;
  output.pose.pose.position.x += source_to_geometry_center_m * std::cos(yaw);
  output.pose.pose.position.y += source_to_geometry_center_m * std::sin(yaw);
  output.twist.twist.linear.y +=
      source_to_geometry_center_m * output.twist.twist.angular.z;

  auto pose_jacobian = IdentityJacobian();
  pose_jacobian[0U * 6U + 5U] = -source_to_geometry_center_m * std::sin(yaw);
  pose_jacobian[1U * 6U + 5U] = source_to_geometry_center_m * std::cos(yaw);
  output.pose.covariance =
      TransformCovariance(source_odometry.pose.covariance, pose_jacobian);

  auto twist_jacobian = IdentityJacobian();
  twist_jacobian[1U * 6U + 5U] = source_to_geometry_center_m;
  output.twist.covariance =
      TransformCovariance(source_odometry.twist.covariance, twist_jacobian);

  TransformPosition(&output.pose.pose.position, output_from_source);
  output.pose.pose.orientation = RotateOrientation(output.pose.pose.orientation,
                                                   output_from_source.yaw_rad);
  output.pose.covariance =
      TransformCovariance(output.pose.covariance,
                          PlanarRotationJacobian(output_from_source.yaw_rad));
  return output;
}

tf2_msgs::msg::TFMessage ToGeometryCenterTf(
    const tf2_msgs::msg::TFMessage& source_tf,
    double source_to_geometry_center_m,
    const std::string& geometry_center_frame_id,
    PlanarTransform output_from_source,
    const std::string& output_odom_frame_id) {
  Validate(source_to_geometry_center_m, geometry_center_frame_id,
           output_from_source);
  tf2_msgs::msg::TFMessage output = source_tf;
  for (auto& transform : output.transforms) {
    const std::array<double, 3> translation{{
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    }};
    if (!std::all_of(translation.begin(), translation.end(),
                     [](double value) { return std::isfinite(value); })) {
      throw std::invalid_argument("transform translation must be finite");
    }
    const double yaw = Yaw(transform.transform.rotation);
    transform.child_frame_id = geometry_center_frame_id;
    if (!output_odom_frame_id.empty()) {
      transform.header.frame_id = output_odom_frame_id;
    }
    transform.transform.translation.x +=
        source_to_geometry_center_m * std::cos(yaw);
    transform.transform.translation.y +=
        source_to_geometry_center_m * std::sin(yaw);
    TransformTranslation(&transform.transform.translation, output_from_source);
    transform.transform.rotation = RotateOrientation(
        transform.transform.rotation, output_from_source.yaw_rad);
  }
  return output;
}

}  // namespace mentor_pi::hardware
