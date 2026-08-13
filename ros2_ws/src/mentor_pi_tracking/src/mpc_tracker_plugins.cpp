// SPDX-License-Identifier: GPL-2.0-or-later

#include <cmath>
#include <stdexcept>

#include "mentor_pi_tracking/tracker_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mentor_pi::tracking {
namespace {

class MpcTrackerPlugin : public TrackerPlugin {
 public:
  explicit MpcTrackerPlugin(VehicleType vehicle) : vehicle_(vehicle) {}

  VehicleType vehicle() const override { return vehicle_; }
  bool requires_worker() const override { return true; }

  void Configure(const TrackerConfiguration& configuration) override {
    const MpcConfiguration& mpc = configuration.mpc;
    if (mpc.vehicle != vehicle_ || mpc.horizon != 10 ||
        !std::isfinite(mpc.prediction_step) || mpc.prediction_step != 0.1 ||
        !std::isfinite(mpc.wheelbase) || mpc.wheelbase <= 0.0 ||
        !std::isfinite(mpc.wheel_track) || mpc.wheel_track <= 0.0 ||
        !std::isfinite(mpc.geometry_center_offset) ||
        (vehicle_ == VehicleType::kAckermann &&
         mpc.geometry_center_offset <= 0.0) ||
        !std::isfinite(mpc.mecanum_radius_sum) ||
        mpc.mecanum_radius_sum <= 0.0 ||
        !std::isfinite(mpc.max_steering_angle) ||
        mpc.max_steering_angle <= 0.0) {
      throw std::invalid_argument("invalid MPC plugin configuration");
    }
  }

  void Reset() override {}

  MpcCommand Compute(const TrackerRequest& request) override {
    if (request.live_configuration.vehicle != vehicle_) {
      return {false, 0.0, 0.0, 0.0, "MPC work vehicle mismatch"};
    }
    return MpcSolver(request.live_configuration).Solve(request.mpc);
  }

  void SetAppliedCommand(const MpcCommand&) override {}

 private:
  VehicleType vehicle_;
};

}  // namespace

class MecanumMpcTracker final : public MpcTrackerPlugin {
 public:
  MecanumMpcTracker() : MpcTrackerPlugin(VehicleType::kMecanum) {}
};

class AckermannMpcTracker final : public MpcTrackerPlugin {
 public:
  AckermannMpcTracker() : MpcTrackerPlugin(VehicleType::kAckermann) {}
};

}  // namespace mentor_pi::tracking

PLUGINLIB_EXPORT_CLASS(mentor_pi::tracking::MecanumMpcTracker,
                       mentor_pi::tracking::TrackerPlugin)
PLUGINLIB_EXPORT_CLASS(mentor_pi::tracking::AckermannMpcTracker,
                       mentor_pi::tracking::TrackerPlugin)
