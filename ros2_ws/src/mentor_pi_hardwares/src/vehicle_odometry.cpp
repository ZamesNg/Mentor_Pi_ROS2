#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "mentor_pi_hardwares/odometry_transform.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mentor_pi::hardware {
namespace {

class VehicleOdometry final : public rclcpp::Node {
 public:
  VehicleOdometry() : rclcpp::Node("_vehicle_odometry") {
    offset_m_ = declare_parameter<double>("source_to_geometry_center_m", 0.0);
    geometry_center_frame_id_ =
        declare_parameter<std::string>("geometry_center_frame_id", "");
    output_odom_frame_id_ =
        declare_parameter<std::string>("output_odom_frame_id", "");
    output_from_source_.x_m =
        declare_parameter<double>("output_odom_origin_x_m", 0.0);
    output_from_source_.y_m =
        declare_parameter<double>("output_odom_origin_y_m", 0.0);
    output_from_source_.yaw_rad =
        declare_parameter<double>("output_odom_origin_yaw_rad", 0.0);
    if (!std::isfinite(offset_m_) || offset_m_ < 0.0) {
      throw std::invalid_argument(
          "source_to_geometry_center_m must be nonnegative and finite");
    }
    if (geometry_center_frame_id_.empty()) {
      throw std::invalid_argument("geometry_center_frame_id must not be empty");
    }
    if (output_odom_frame_id_.empty()) {
      throw std::invalid_argument("output_odom_frame_id must not be empty");
    }
    if (!std::isfinite(output_from_source_.x_m) ||
        !std::isfinite(output_from_source_.y_m) ||
        !std::isfinite(output_from_source_.yaw_rad)) {
      throw std::invalid_argument("output odometry origin must be finite");
    }

    const auto qos = rclcpp::SystemDefaultsQoS();
    odometry_publisher_ =
        create_publisher<nav_msgs::msg::Odometry>("vehicle/odometry", qos);
    tf_publisher_ =
        create_publisher<tf2_msgs::msg::TFMessage>("vehicle/tf_odometry", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        "vehicle/_controller_odometry", qos,
        [this](const nav_msgs::msg::Odometry& message) {
          try {
            odometry_publisher_->publish(ToGeometryCenterOdometry(
                message, offset_m_, geometry_center_frame_id_,
                output_from_source_, output_odom_frame_id_));
          } catch (const std::invalid_argument& error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                                  error.what());
          }
        });
    tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
        "vehicle/_controller_tf_odometry", qos,
        [this](const tf2_msgs::msg::TFMessage& message) {
          try {
            const auto output = ToGeometryCenterTf(
                message, offset_m_, geometry_center_frame_id_,
                output_from_source_, output_odom_frame_id_);
            tf_publisher_->publish(output);
            tf_broadcaster_->sendTransform(output.transforms);
          } catch (const std::invalid_argument& error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                                  error.what());
          }
        });
  }

 private:
  double offset_m_{0.0};
  PlanarTransform output_from_source_{};
  std::string geometry_center_frame_id_;
  std::string output_odom_frame_id_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
};

}  // namespace
}  // namespace mentor_pi::hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mentor_pi::hardware::VehicleOdometry>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("_vehicle_odometry"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
