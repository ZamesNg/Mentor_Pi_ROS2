# Lower-Level Tracking Controller Contract

## Scope and licensing

`mentor_pi_tracking` is a hardware-independent C++17 trajectory tracker for the
Mecanum and Ackermann `ros2_control` vehicles. One
`/mentor_pi/trajectory_tracker` node owns the ROS interfaces, odometry
freshness gate, scheduling, and command bounds and loads exactly one controller
through pluginlib:

- `mentor_pi_tracking/MecanumMpc`;
- `mentor_pi_tracking/AckermannMpc`;
- `mentor_pi_tracking/MecanumAdrc`; or
- `mentor_pi_tracking/AckermannAdrc`.

The ADRC plugins contain bounded fixed-size feedback work and do not link or
load ALTO. The MPC plugins are GPL-2.0-or-later because they link to the pinned
GPL-2.0-or-later `https://github.com/ZamesNg/altro-cpp` fork at commit
`7336800baa3f2f6e0c8edfad472c1ea51c54321a`. The build applies the reviewed
compatibility patch in a disposable build tree and carries the required source
and license provenance.

High-level planning and arbitrary frame transforms remain offboard. The
hardware launch publishes geometry-center odometry for both vehicle types, so
the tracker performs no odometry reference-point conversion.

## Interfaces and trajectory meaning

`mentor_pi_tracking_interfaces/PolynomialSegment` contains a positive duration
and six coefficients `c0` through `c5` for each of x, y, and unwrapped yaw.
Coefficients are evaluated in segment-local seconds. `PolynomialTrajectory`
contains a `std_msgs/Header`, a string bounded to 64 characters, and at most 64
segments. `header.frame_id` is exactly `odom`; `header.stamp` is the scheduled
ROS start time.

| Interface | Contract |
| --- | --- |
| Trajectory input | `/mentor_pi/trajectory_tracker/reference_trajectory` |
| Cancel service | `/mentor_pi/trajectory_tracker/cancel` |
| Mecanum odometry/output | `/mentor_pi/vehicle/odometry` and `base_footprint` command on `/mentor_pi/vehicle/reference` |
| Ackermann odometry/output | geometry-center `/mentor_pi/vehicle/odometry` and `rear_axle_footprint` command on `/mentor_pi/vehicle/reference` |
| Diagnostics | `diagnostic_msgs/msg/DiagnosticArray` on `/mentor_pi/trajectory_tracker/diagnostics` |

Trajectory input uses reliable, volatile QoS. Commands use
`geometry_msgs/msg/TwistStamped`; cancel uses `std_srvs/srv/Trigger`.

Mecanum tracks scheduled geometry-center `x`, `y`, and unwrapped yaw.
Ackermann tracks scheduled geometry-center `x` and `y`; measured yaw remains a
kinematic state but is not a separate tracking objective. Ackermann messages
retain the common finite, continuous yaw polynomial for interface compatibility,
but neither Ackermann plugin uses it as a cost or reference-control target.

## Ackermann geometry and model

The measured runtime geometry is wheelbase `L=0.135 m`, rear-axle-to-center
offset `l=0.0675 m`, wheel track `0.140 m`, wheel radius `0.0325 m`, and
steering limit `+/-0.6 rad`. Public Ackermann odometry already describes
`base_footprint` at the geometry center. The tracker copies its pose directly
into the three-state vector `(x_center, y_center, theta)`. The fixed URDF
transform places `rear_axle_footprint` `0.0675 m` behind `base_footprint`.
Vehicle launch validates the selected hardware profile against these measured
tracking dimensions and fails closed rather than running mismatched kinematics.

With rear-axle speed `v`, steering angle `delta`, and heading `theta`, both
Ackermann plugins use:

```text
omega = v * tan(delta) / L
x_center_dot = v * cos(theta) - l * omega * sin(theta)
y_center_dot = v * sin(theta) + l * omega * cos(theta)
theta_dot = omega
```

Ackermann MPC penalizes scheduled center-position error and bounded control,
not yaw error. Path derivatives seed the control trajectory while the solver
selects speed and steering subject to the center dynamics and actuator bounds.

## Runtime and plugin selection

The node runs at 30 Hz. It validates and stages a trajectory only when its ROS
start is 0.25 through 60 seconds in the future, then converts that time to a
steady-clock deadline. Duplicate IDs are ignored. A replacement switches
atomically at its scheduled start; planner or network loss after acceptance
does not cancel it.

Every selected plugin executes on the single controller worker and has a 25 ms
result deadline. MPC uses a ten-step, 0.1-second ALTO horizon. After an MPC miss
or solver failure, its bounded model-specific feedback may run for no more than
100 ms before output is inhibited. ADRC has no secondary fallback: a deadline,
timing, or numerical failure immediately resets it and publishes zero.

Launch selection is:

```text
tracking_controller:=auto|none|mecanum|ackermann
tracking_algorithm:=adrc|mpc
```

The defaults are `auto` and `adrc`: `auto` selects the tracker plugin matching
the selected physical or simulated vehicle. The explicit `mecanum` and
`ackermann` values remain accepted only when they match that vehicle, and
`none` disables the tracker for direct `vehicle/reference` testing. MPC and
ADRC always use the same `/<robot_name>/trajectory_tracker` node,
`trajectory_tracker/reference_trajectory` input,
`trajectory_tracker/cancel` service, `vehicle/odometry` input, and
`vehicle/reference` output. Diagnostics use
`trajectory_tracker/diagnostics`. No vehicle- or algorithm-specific topic,
service, node, or executable alias exists. Both `vehicle.launch.py` and
`simulation.launch.py` implement this selection contract and default to ADRC.

## Trajectory LADRC

The ADRC plugins use first-order linear ADRC in world coordinates. For measured
coordinate `y`, scheduled position `r`, scheduled derivative `r_dot`, actual
period `T`, nominal input gain `b0`, controller bandwidth `wc`, observer
bandwidth `wo`, and the previously applied post-bound command `u_applied`:

```text
e = z1 - y
z1 += T * (z2 + b0 * u_applied - 2 * wo * e)
z2 += T * (-wo * wo * e)
u_raw = (r_dot + wc * (r - z1) - z2) / b0
```

`z1` estimates position and `z2` estimates combined unmodelled velocity and
disturbance. Mecanum applies the equations to world X, world Y, and unwrapped
yaw, then rotates the linear result into the body frame. Ackermann applies them
only to center X/Y and converts the desired center velocity using:

```text
v = cos(theta) * ux + sin(theta) * uy
omega = (-sin(theta) * ux + cos(theta) * uy) / l
```

The common bounder converts the configured driven-wheel angular-speed ceiling
through the wheel radius, then applies the Mecanum wheel envelope or Ackermann
speed/steering curvature plus left/right rear-wheel envelopes using the
measured track. It reconstructs the center velocity achievable by that final
command and supplies that value as
`u_applied` on the next ESO update. This is deliberately layered: the trajectory
ESO sees the bounded published twist, chassis ESOs see their bounded corrections,
and MCU ESOs see the final rounded and clamped PWM. Knowledge of an applied
command prevents observer error during saturation but cannot make a nonzero
minimum-drive floor linear; the firmware floor therefore remains zero.

The configurable trajectory parameters and defaults are:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `driven_wheel_angular_speed_limit_rad_s` | `37.69911184307752` | Static driven-wheel ceiling used for tracker command bounds; must be finite, positive, and no greater than the 6 RPS implementation ceiling. |
| `position_adrc_input_gain` | `1.0` | Dimensionless nominal gain from commanded center velocity to position derivative; shared by X/Y. |
| `position_adrc_controller_bandwidth_rad_s` | `1.0` | Shared X/Y response bandwidth. |
| `position_adrc_observer_bandwidth_rad_s` | `3.0` | Shared X/Y ESO bandwidth. |
| `yaw_adrc_input_gain` | `1.0` | Dimensionless nominal Mecanum yaw-rate gain. |
| `yaw_adrc_controller_bandwidth_rad_s` | `1.0` | Mecanum yaw response bandwidth. |
| `yaw_adrc_observer_bandwidth_rad_s` | `3.0` | Mecanum yaw ESO bandwidth. |

All values must be finite and positive, `wo >= wc`, and `wo*T <= 0.5`. ADRC
state resets on a new or replaced trajectory, cancel, completion, stale
odometry, time discontinuity, invalid computation, or recovery from inhibition.

## Safety and qualification boundary

Bounds derive from configured geometry and the static driven-wheel speed limit,
which cannot exceed the 6 RPS implementation ceiling. The output is zero before
start, after completion/cancel, with missing or older-than-100-ms odometry, or
after a local fault. The tracker has no MCU motor-state, heartbeat,
authorization, Agent, or configuration-supervisor dependency. Physical motion
remains independently gated by the hardware adapter, supervisor, firmware
session checks, and motor leases.

Software tests cover plugin discovery, validation, scheduling, both vehicle
models and algorithms, bounds, applied-command observer feedback, fallback,
cancellation, and safe zero. They do not qualify tracking performance. Native
RDK X5 deadline measurements and recorded, reviewed guarded HIL are required
before powered-motion, ADRC-performance, or release claims.
