#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mentor_pi_hardwares/odometry_transform.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace mentor_pi::hardware {
namespace {

class VehiclePose final : public rclcpp::Node {
 public:
  VehiclePose() : rclcpp::Node("vehicle_pose") {
    input_type_ = declare_parameter<std::string>("input_type", "mocap_pose");
    offset_m_ = declare_parameter<double>("source_to_geometry_center_m", 0.0);
    geometry_center_frame_id_ =
        declare_parameter<std::string>("geometry_center_frame_id", "");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    output_from_source_.x_m =
        declare_parameter<double>("output_origin_x_m", 0.0);
    output_from_source_.y_m =
        declare_parameter<double>("output_origin_y_m", 0.0);
    output_from_source_.yaw_rad =
        declare_parameter<double>("output_origin_yaw_rad", 0.0);

    if (input_type_ != "mocap_pose" && input_type_ != "controller_odometry") {
      throw std::invalid_argument(
          "input_type must be mocap_pose or controller_odometry");
    }
    if (!std::isfinite(offset_m_) || offset_m_ < 0.0) {
      throw std::invalid_argument(
          "source_to_geometry_center_m must be nonnegative and finite");
    }
    if (geometry_center_frame_id_.empty()) {
      throw std::invalid_argument("geometry_center_frame_id must not be empty");
    }
    if (output_frame_id_ != "map") {
      throw std::invalid_argument("output_frame_id must be map");
    }
    if (!std::isfinite(output_from_source_.x_m) ||
        !std::isfinite(output_from_source_.y_m) ||
        !std::isfinite(output_from_source_.yaw_rad)) {
      throw std::invalid_argument("output pose origin must be finite");
    }
    if (input_type_ == "mocap_pose" &&
        (offset_m_ != 0.0 || output_from_source_.x_m != 0.0 ||
         output_from_source_.y_m != 0.0 ||
         output_from_source_.yaw_rad != 0.0)) {
      throw std::invalid_argument(
          "mocap pose must already describe the geometry center in map");
    }

    const auto qos = rclcpp::SensorDataQoS();
    pose_publisher_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("vehicle/pose", qos);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (input_type_ == "mocap_pose") {
      mocap_subscription_ =
          create_subscription<geometry_msgs::msg::PoseStamped>(
              "vehicle/_mocap_pose", qos,
              [this](const geometry_msgs::msg::PoseStamped& message) {
                try {
                  Publish(ValidateGeometryCenterPose(message));
                } catch (const std::invalid_argument& error) {
                  ReportInvalid(error);
                }
              });
    } else {
      odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
          "vehicle/_controller_odometry", qos,
          [this](const nav_msgs::msg::Odometry& message) {
            try {
              Publish(GeometryCenterPoseFromOdometry(
                  message, offset_m_, geometry_center_frame_id_,
                  output_from_source_, output_frame_id_));
            } catch (const std::invalid_argument& error) {
              ReportInvalid(error);
            }
          });
    }
  }

 private:
  void Publish(const geometry_msgs::msg::PoseStamped& pose) {
    pose_publisher_->publish(pose);
    tf_broadcaster_->sendTransform(
        GeometryCenterPoseToTransform(pose, geometry_center_frame_id_));
  }

  void ReportInvalid(const std::invalid_argument& error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
  }

  std::string input_type_;
  double offset_m_{0.0};
  PlanarTransform output_from_source_{};
  std::string geometry_center_frame_id_;
  std::string output_frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      mocap_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odometry_subscription_;
};

}  // namespace
}  // namespace mentor_pi::hardware

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<mentor_pi::hardware::VehiclePose>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("vehicle_pose"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
