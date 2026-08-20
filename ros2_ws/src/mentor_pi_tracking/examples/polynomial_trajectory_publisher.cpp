// SPDX-License-Identifier: GPL-2.0-or-later

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "mentor_pi_tracking_interfaces/msg/polynomial_segment.hpp"
#include "mentor_pi_tracking_interfaces/msg/polynomial_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {

class ExamplePublisher final : public rclcpp::Node {
 public:
  ExamplePublisher() : Node("polynomial_trajectory_publisher") {
    const std::string vehicle =
        declare_parameter<std::string>("vehicle_type", "mecanum");
    if (vehicle != "mecanum" && vehicle != "ackermann") {
      throw std::invalid_argument("vehicle_type must be mecanum or ackermann");
    }
    publisher_ = create_publisher<
        mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>(
        "/mentor_pi/trajectory_tracker/reference_trajectory",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile());
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
      mentor_pi_tracking_interfaces::msg::PolynomialTrajectory trajectory;
      trajectory.header.frame_id = "map";
      trajectory.header.stamp = now() + rclcpp::Duration::from_seconds(1.0);
      trajectory.trajectory_id = "example_straight_line";
      mentor_pi_tracking_interfaces::msg::PolynomialSegment segment;
      segment.duration.sec = 5;
      segment.x_coefficients[1] = 0.1;
      trajectory.segments.push_back(segment);
      publisher_->publish(trajectory);
      RCLCPP_INFO(get_logger(), "Published approved map-frame example spline");
      timer_->cancel();
    });
  }

 private:
  rclcpp::Publisher<mentor_pi_tracking_interfaces::msg::PolynomialTrajectory>::
      SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExamplePublisher>());
  rclcpp::shutdown();
  return 0;
}
