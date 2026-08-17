// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef MENTOR_PI_TRACKING__TRACKER_PLUGIN_HPP_
#define MENTOR_PI_TRACKING__TRACKER_PLUGIN_HPP_

#include <memory>

#include "mentor_pi_tracking/mpc_solver.hpp"

namespace mentor_pi::tracking {

struct TrackerConfiguration {
  MpcConfiguration mpc{};
  double position_adrc_input_gain{1.0};
  double position_adrc_controller_bandwidth_rad_s{1.0};
  double position_adrc_observer_bandwidth_rad_s{3.0};
  double yaw_adrc_input_gain{1.0};
  double yaw_adrc_controller_bandwidth_rad_s{1.0};
  double yaw_adrc_observer_bandwidth_rad_s{3.0};
};

struct TrackerRequest {
  MpcRequest mpc{};
  // Static vehicle geometry and actuator bounds are part of every work item.
  // MPC must solve against them, not only rely on the node's output clamp.
  MpcConfiguration bounded_configuration{};
  // Keep the accepted trajectory alive while a controller worker is using it.
  std::shared_ptr<const PolynomialTrajectory> trajectory;
  // The node measures this with its steady clock for every controller update.
  double measured_period_seconds{};
};

// Plugins compute an unbounded chassis command. The node is the sole safety
// boundary: it bounds the command and returns that applied command to stateful
// plugins before their next observer update.
class TrackerPlugin {
 public:
  virtual ~TrackerPlugin() = default;
  virtual VehicleType vehicle() const = 0;
  virtual bool requires_worker() const = 0;
  virtual void Configure(const TrackerConfiguration& configuration) = 0;
  virtual void Reset() = 0;
  virtual MpcCommand Compute(const TrackerRequest& request) = 0;
  virtual void SetAppliedCommand(const MpcCommand& command) = 0;
};

}  // namespace mentor_pi::tracking

#endif  // MENTOR_PI_TRACKING__TRACKER_PLUGIN_HPP_
