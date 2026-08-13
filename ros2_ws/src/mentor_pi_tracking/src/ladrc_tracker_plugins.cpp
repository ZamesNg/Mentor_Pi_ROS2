// SPDX-License-Identifier: GPL-2.0-or-later

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "mentor_pi_tracking/tracker_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mentor_pi::tracking {
namespace {

class FirstOrderLadrc final {
 public:
  void Configure(double controller_bandwidth, double observer_bandwidth,
                 double input_gain) {
    if (!std::isfinite(controller_bandwidth) ||
        !std::isfinite(observer_bandwidth) || !std::isfinite(input_gain) ||
        controller_bandwidth <= 0.0 ||
        observer_bandwidth < controller_bandwidth || input_gain <= 0.0) {
      throw std::invalid_argument("invalid LADRC axis configuration");
    }
    controller_bandwidth_ = controller_bandwidth;
    observer_bandwidth_ = observer_bandwidth;
    input_gain_ = input_gain;
    Reset();
  }

  void Reset() {
    z1_ = 0.0;
    z2_ = 0.0;
    initialized_ = false;
  }

  std::optional<double> Update(double reference_position,
                               double reference_velocity,
                               double measured_position, double applied_input,
                               double period) {
    if (!std::isfinite(reference_position) ||
        !std::isfinite(reference_velocity) ||
        !std::isfinite(measured_position) || !std::isfinite(applied_input) ||
        !std::isfinite(period) || period <= 0.0 ||
        observer_bandwidth_ * period > 0.5) {
      Reset();
      return std::nullopt;
    }
    if (!initialized_) {
      z1_ = measured_position;
      z2_ = 0.0;
      initialized_ = true;
    }
    const double error = z1_ - measured_position;
    z1_ += period * (z2_ + input_gain_ * applied_input -
                     2.0 * observer_bandwidth_ * error);
    z2_ += period * (-observer_bandwidth_ * observer_bandwidth_ * error);
    const double output =
        (reference_velocity +
         controller_bandwidth_ * (reference_position - z1_) - z2_) /
        input_gain_;
    if (!std::isfinite(z1_) || !std::isfinite(z2_) || !std::isfinite(output)) {
      Reset();
      return std::nullopt;
    }
    return output;
  }

 private:
  double controller_bandwidth_{1.0};
  double observer_bandwidth_{3.0};
  double input_gain_{1.0};
  double z1_{};
  double z2_{};
  bool initialized_{};
};

bool Finite(const TrackerRequest& request) {
  return request.trajectory != nullptr &&
         request.mpc.trajectory == request.trajectory.get() &&
         std::isfinite(request.mpc.elapsed_seconds) &&
         std::isfinite(request.measured_period_seconds) &&
         request.measured_period_seconds > 0.0 &&
         std::isfinite(request.mpc.state[0]) &&
         std::isfinite(request.mpc.state[1]) &&
         std::isfinite(request.mpc.state[2]);
}

bool ValidMpcGeometry(const MpcConfiguration& configuration,
                      VehicleType vehicle) {
  return configuration.vehicle == vehicle &&
         std::isfinite(configuration.wheelbase) &&
         configuration.wheelbase > 0.0 &&
         std::isfinite(configuration.wheel_track) &&
         configuration.wheel_track > 0.0 &&
         std::isfinite(configuration.geometry_center_offset) &&
         (vehicle == VehicleType::kMecanum ||
          configuration.geometry_center_offset > 0.0) &&
         std::isfinite(configuration.mecanum_radius_sum) &&
         configuration.mecanum_radius_sum > 0.0 &&
         std::isfinite(configuration.max_steering_angle) &&
         configuration.max_steering_angle > 0.0;
}

class LadrcTrackerPlugin : public TrackerPlugin {
 public:
  explicit LadrcTrackerPlugin(VehicleType vehicle) : vehicle_(vehicle) {}

  VehicleType vehicle() const override { return vehicle_; }
  bool requires_worker() const override { return true; }

  void Configure(const TrackerConfiguration& configuration) override {
    if (!ValidMpcGeometry(configuration.mpc, vehicle_)) {
      throw std::invalid_argument("invalid LADRC vehicle configuration");
    }
    configuration_ = configuration;
    const std::size_t count = vehicle_ == VehicleType::kMecanum ? 3U : 2U;
    for (std::size_t index = 0U; index < count; ++index) {
      const bool yaw_axis = vehicle_ == VehicleType::kMecanum && index == 2U;
      axes_[index].Configure(
          yaw_axis ? configuration.yaw_adrc_controller_bandwidth_rad_s
                   : configuration.position_adrc_controller_bandwidth_rad_s,
          yaw_axis ? configuration.yaw_adrc_observer_bandwidth_rad_s
                   : configuration.position_adrc_observer_bandwidth_rad_s,
          yaw_axis ? configuration.yaw_adrc_input_gain
                   : configuration.position_adrc_input_gain);
    }
    Reset();
  }

  void Reset() override {
    for (auto& axis : axes_) {
      axis.Reset();
    }
    applied_world_ = {};
    last_yaw_ = 0.0;
  }

  MpcCommand Compute(const TrackerRequest& request) override {
    if (!Finite(request)) {
      Reset();
      return {false, 0.0, 0.0, 0.0, "invalid LADRC request"};
    }
    const ReferenceState reference =
        request.trajectory->Evaluate(request.mpc.elapsed_seconds);
    if (!std::isfinite(reference.x) || !std::isfinite(reference.y) ||
        !std::isfinite(reference.vx_world) ||
        !std::isfinite(reference.vy_world) ||
        (vehicle_ == VehicleType::kMecanum &&
         (!std::isfinite(reference.yaw) ||
          !std::isfinite(reference.yaw_rate)))) {
      Reset();
      return {false, 0.0, 0.0, 0.0, "non-finite LADRC reference"};
    }

    const std::array<double, 3> measured{
        {request.mpc.state[0], request.mpc.state[1], request.mpc.state[2]}};
    std::array<double, 3> position{{reference.x, reference.y, 0.0}};
    std::array<double, 3> derivative{
        {reference.vx_world, reference.vy_world, 0.0}};
    if (vehicle_ == VehicleType::kMecanum) {
      position[2] = reference.yaw;
      derivative[2] = reference.yaw_rate;
    }
    std::array<double, 3> world_command{};
    const std::size_t count = vehicle_ == VehicleType::kMecanum ? 3U : 2U;
    for (std::size_t index = 0U; index < count; ++index) {
      const std::optional<double> command = axes_[index].Update(
          position[index], derivative[index], measured[index],
          applied_world_[index], request.measured_period_seconds);
      if (!command.has_value()) {
        Reset();
        return {false, 0.0, 0.0, 0.0, "LADRC observer failure"};
      }
      world_command[index] = *command;
    }

    last_yaw_ = request.mpc.state[2];
    const double cosine = std::cos(last_yaw_);
    const double sine = std::sin(last_yaw_);
    if (vehicle_ == VehicleType::kMecanum) {
      return {true,
              cosine * world_command[0] + sine * world_command[1],
              -sine * world_command[0] + cosine * world_command[1],
              world_command[2],
              {}};
    }
    const double rear_speed =
        cosine * world_command[0] + sine * world_command[1];
    const double yaw_rate =
        (-sine * world_command[0] + cosine * world_command[1]) /
        configuration_.mpc.geometry_center_offset;
    return {true, rear_speed, 0.0, yaw_rate, {}};
  }

  void SetAppliedCommand(const MpcCommand& command) override {
    if (!command.solved || !std::isfinite(command.linear_x) ||
        !std::isfinite(command.linear_y) || !std::isfinite(command.angular_z)) {
      Reset();
      return;
    }
    const double cosine = std::cos(last_yaw_);
    const double sine = std::sin(last_yaw_);
    if (vehicle_ == VehicleType::kMecanum) {
      applied_world_ = {{cosine * command.linear_x - sine * command.linear_y,
                         sine * command.linear_x + cosine * command.linear_y,
                         command.angular_z}};
      return;
    }
    const double offset = configuration_.mpc.geometry_center_offset;
    // Feed back the centre velocity that the post-bound rear-axle command can
    // actually produce, never the unbounded LADRC request.
    applied_world_ = {
        {cosine * command.linear_x - offset * sine * command.angular_z,
         sine * command.linear_x + offset * cosine * command.angular_z, 0.0}};
  }

 private:
  VehicleType vehicle_;
  TrackerConfiguration configuration_;
  std::array<FirstOrderLadrc, 3> axes_{};
  std::array<double, 3> applied_world_{};
  double last_yaw_{};
};

}  // namespace

class MecanumLadrcTracker final : public LadrcTrackerPlugin {
 public:
  MecanumLadrcTracker() : LadrcTrackerPlugin(VehicleType::kMecanum) {}
};

class AckermannLadrcTracker final : public LadrcTrackerPlugin {
 public:
  AckermannLadrcTracker() : LadrcTrackerPlugin(VehicleType::kAckermann) {}
};

}  // namespace mentor_pi::tracking

PLUGINLIB_EXPORT_CLASS(mentor_pi::tracking::MecanumLadrcTracker,
                       mentor_pi::tracking::TrackerPlugin)
PLUGINLIB_EXPORT_CLASS(mentor_pi::tracking::AckermannLadrcTracker,
                       mentor_pi::tracking::TrackerPlugin)
