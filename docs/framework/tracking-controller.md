# Lower-Level Tracking Controller Contract

## Scope and licensing

`mentor_pi_tracking` is an opt-in C++17 lower-level MPC tracker for the mecanum
and Ackermann `ros2_control` adapters. It runs onboard the RDK X5. High-level
planning and frame transforms remain offboard; accepted trajectories are
already transformed into `odom`.

The package is GPL-2.0-or-later because it links to the GPL-2.0-or-later
`https://github.com/ZamesNg/altro-cpp` fork at commit
`7336800baa3f2f6e0c8edfad472c1ea51c54321a`. Setup fetches only that repository
and verifies the commit. The build copies it to a disposable build directory,
applies the checksummed compatibility patch, and builds offline with upstream
tests, examples, documentation, and benchmarks disabled. Handoffs carry the
license, commit provenance, patch, and corresponding ALTO and project source.

## Interfaces

`mentor_pi_tracking_interfaces/PolynomialSegment` contains a positive duration
and six coefficients `c0` through `c5` for each of x, y, and unwrapped yaw.
Coefficients are evaluated in segment-local seconds. `PolynomialTrajectory`
contains a `std_msgs/Header`, a string bounded to 64 characters, and at most 64
segments. `header.frame_id` is exactly `odom`; `header.stamp` is the scheduled
ROS start time.

| Vehicle | Trajectory input | Odometry input | Command output | Cancel service |
| --- | --- | --- | --- | --- |
| Mecanum | `/mentor_pi/mecanum_mpc_tracker/reference_trajectory` | configured mecanum controller odometry | `/mentor_pi/mecanum_drive_controller/reference` | `/mentor_pi/mecanum_mpc_tracker/cancel` |
| Ackermann | `/mentor_pi/ackermann_mpc_tracker/reference_trajectory` | configured Ackermann controller odometry | `/mentor_pi/ackermann_steering_controller/reference` | `/mentor_pi/ackermann_mpc_tracker/cancel` |

Trajectory input uses reliable, volatile QoS. Commands use
`geometry_msgs/msg/TwistStamped`; cancel uses `std_srvs/srv/Trigger`. Status and
faults use `diagnostic_msgs/msg/DiagnosticArray` on `/diagnostics`.

## Runtime behavior

The tracker runs at 30 Hz and solves a ten-step, 0.1-second ALTO horizon on a
worker thread. A result has a 25 ms deadline. Mecanum state is `[x,y,yaw]` with
body-frame `[vx,vy,yaw_rate]`; Ackermann control is `[speed,steering_angle]` and
publishes `linear.x=speed` and
`angular.z=speed*tan(steering_angle)/wheelbase`. Bounds derive from the selected
geometry and fresh live motor profile and never exceed its limit or 6 RPS.

Messages that are malformed, non-finite, empty, expired, discontinuous, in the
wrong frame, or not scheduled sufficiently in the future are rejected without
disturbing the accepted trajectory. Duplicate IDs are ignored. A valid
replacement is staged and switches atomically at its scheduled start. Planner
or network loss after acceptance does not cancel it. Cancel clears active and
pending trajectories immediately.

The host clock must already be synchronized. At acceptance, the ROS start time
is converted to a steady-clock deadline; subsequent scheduling uses monotonic
time. The repository checks synchronization but never changes NTP or chrony
configuration.

The output is zero before start, after completion, after cancellation, with
stale odometry, without current authorization/profile state, or after a local
fault. After a solver miss or failure, bounded feedback may be used for no more
than 100 ms; output is then zero. Launch selection is
`tracking_controller:=none|mecanum|ackermann`, defaults to `none`, and must
match the selected vehicle.

## Qualification boundary

Software tests cover trajectory evaluation/validation, angle handling, both
models, bounds, fallback, cancellation, scheduling, time conversion, stale
state, and safe zero output. Neither those tests nor an amd64 smoke qualify
tracking performance. Native RDK X5 30 Hz/25 ms benchmarks and the guarded HIL
sequence are required before hardware use or any powered-motion or tracking
performance claim.
