// SPDX-License-Identifier: GPL-2.0-or-later

#include <algorithm>
#include <cmath>

#include "mentor_pi_tracking/mpc_solver.hpp"

namespace mentor_pi::tracking {
namespace {

bool Valid(const MpcConfiguration& configuration) {
  return std::isfinite(configuration.wheelbase) &&
         std::isfinite(configuration.wheel_track) &&
         std::isfinite(configuration.geometry_center_offset) &&
         std::isfinite(configuration.mecanum_radius_sum) &&
         std::isfinite(configuration.max_linear_speed) &&
         std::isfinite(configuration.max_lateral_speed) &&
         std::isfinite(configuration.max_yaw_rate) &&
         std::isfinite(configuration.max_steering_angle) &&
         configuration.wheelbase > 0.0 && configuration.wheel_track > 0.0 &&
         configuration.max_linear_speed > 0.0 &&
         configuration.max_lateral_speed > 0.0 &&
         configuration.max_yaw_rate > 0.0 &&
         configuration.max_steering_angle > 0.0 &&
         (configuration.vehicle == VehicleType::kMecanum ||
          configuration.geometry_center_offset > 0.0);
}

bool Finite(const ReferenceState& reference) {
  return std::isfinite(reference.x) && std::isfinite(reference.y) &&
         std::isfinite(reference.yaw) && std::isfinite(reference.vx_world) &&
         std::isfinite(reference.vy_world) && std::isfinite(reference.yaw_rate);
}

}  // namespace

MpcCommand EnforceCommandBounds(const MpcConfiguration& configuration,
                                MpcCommand command) {
  if (!command.solved || !Valid(configuration) ||
      !std::isfinite(command.linear_x) || !std::isfinite(command.linear_y) ||
      !std::isfinite(command.angular_z)) {
    return {};
  }

  if (configuration.vehicle == VehicleType::kMecanum) {
    if (configuration.mecanum_radius_sum <= 0.0) {
      return {};
    }
    command.linear_x =
        std::clamp(command.linear_x, -configuration.max_linear_speed,
                   configuration.max_linear_speed);
    command.linear_y =
        std::clamp(command.linear_y, -configuration.max_lateral_speed,
                   configuration.max_lateral_speed);
    command.angular_z =
        std::clamp(command.angular_z, -configuration.max_yaw_rate,
                   configuration.max_yaw_rate);
    const double yaw_component =
        configuration.mecanum_radius_sum * command.angular_z;
    const double maximum_wheel = std::max(
        {std::abs(command.linear_x - command.linear_y - yaw_component),
         std::abs(command.linear_x + command.linear_y + yaw_component),
         std::abs(command.linear_x + command.linear_y - yaw_component),
         std::abs(command.linear_x - command.linear_y + yaw_component)});
    if (maximum_wheel > configuration.max_linear_speed) {
      const double scale = configuration.max_linear_speed / maximum_wheel;
      command.linear_x *= scale;
      command.linear_y *= scale;
      command.angular_z *= scale;
    }
    return command;
  }

  command.linear_x =
      std::clamp(command.linear_x, -configuration.max_linear_speed,
                 configuration.max_linear_speed);
  command.linear_y = 0.0;
  const double curvature_yaw_rate = std::abs(command.linear_x) *
                                    std::tan(configuration.max_steering_angle) /
                                    configuration.wheelbase;
  const double yaw_rate_limit =
      std::min(configuration.max_yaw_rate, curvature_yaw_rate);
  command.angular_z =
      std::clamp(command.angular_z, -yaw_rate_limit, yaw_rate_limit);
  const double maximum_rear_wheel_speed =
      std::max(std::abs(command.linear_x -
                        0.5 * configuration.wheel_track * command.angular_z),
               std::abs(command.linear_x +
                        0.5 * configuration.wheel_track * command.angular_z));
  if (maximum_rear_wheel_speed > configuration.max_linear_speed) {
    const double scale =
        configuration.max_linear_speed / maximum_rear_wheel_speed;
    command.linear_x *= scale;
    command.angular_z *= scale;
  }
  return command;
}

MpcCommand FeedbackCommand(const MpcConfiguration& configuration,
                           const MpcRequest& request) {
  if (!Valid(configuration) || request.trajectory == nullptr ||
      !std::isfinite(request.elapsed_seconds) ||
      !std::all_of(request.state.begin(), request.state.end(),
                   [](double value) { return std::isfinite(value); })) {
    return {};
  }
  const ReferenceState reference =
      request.trajectory->Evaluate(request.elapsed_seconds);
  if (!Finite(reference)) {
    return {};
  }
  const double yaw = request.state[2];
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double error_x = reference.x - request.state[0];
  const double error_y = reference.y - request.state[1];
  if (configuration.vehicle == VehicleType::kMecanum) {
    return EnforceCommandBounds(
        configuration,
        {true,
         cosine * reference.vx_world + sine * reference.vy_world +
             cosine * error_x + sine * error_y,
         -sine * reference.vx_world + cosine * reference.vy_world -
             sine * error_x + cosine * error_y,
         reference.yaw_rate + WrapAngle(reference.yaw - yaw),
         "bounded feedback fallback"});
  }

  // Ackermann intentionally uses only centre x/y and their derivatives.  Its
  // common yaw polynomial is not a control objective or feed-forward input.
  const double desired_x = reference.vx_world + error_x;
  const double desired_y = reference.vy_world + error_y;
  const double rear_speed = cosine * desired_x + sine * desired_y;
  const double yaw_rate = (-sine * desired_x + cosine * desired_y) /
                          configuration.geometry_center_offset;
  return EnforceCommandBounds(configuration, {true, rear_speed, 0.0, yaw_rate,
                                              "bounded feedback fallback"});
}

}  // namespace mentor_pi::tracking
