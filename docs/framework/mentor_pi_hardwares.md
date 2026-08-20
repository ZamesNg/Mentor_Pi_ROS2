# `mentor_pi_hardwares` architecture

`mentor_pi_hardwares` is the command and ros2_control hardware boundary for
the generated robot namespace. The micro-ROS Agent remains an external systemd
prerequisite and the configuration supervisor remains application-owned.

## Runtime topology

Physical launch starts the configuration supervisor, robot state publisher,
controller manager, one hardware plugin, the selected drive controller, and
`joint_state_broadcaster`. Simulation launch starts the corresponding
simulation hardware without firmware-facing endpoints.

Both launches are command-only. They do not start mocap, global pose, TF,
polynomial trajectory, ADRC trajectory-tracking, or MPC processes. An external
application publishes best-effort `geometry_msgs/msg/TwistStamped` commands to
`/<robot>/vehicle/reference`.

The selected controller replaces a zero header timestamp with its own receipt
time and applies the configured 100 ms reference timeout. Its encoder odometry
and TF odometry outputs are remapped to private
`/<robot>/vehicle/_controller_*` topics and `enable_odom_tf` remains false.
An external state adapter exclusively owns public map pose and
`map -> <robot>/base_footprint`.

Robot state publication still owns the URDF fixed and joint transforms below
`<robot>/base_footprint`. Physical joint names, controller-state message
types, authorization, firmware leases, and fail-closed hardware behavior remain
vehicle-specific and unchanged.

## Configuration ownership

The selected absolute `vehicle_config` YAML is the sole authority for
`vehicle.robot_name` and `vehicle.vehicle_type`. Serial/Agent
configuration belongs to `mentor-pi-agent.service`, not application launch.
Mode-specific profiles under `config/mecanum` and `config/ackermann` own:

- vehicle geometry and joint names;
- the `ros2_control` URDF and plugin selection;
- controller parameters;
- wheel geometry and chassis ADRC parameters;
- chassis ADRC linear/yaw measurement-LPF cutoffs;
- `feedback_timeout_ms` and `imu_timeout_ms`, both defaulting to 100 ms;
- Ackermann PWM channel, min/center/max pulse, inversion, angle limits, and
  command duration.

The host timeout does not replace transport recovery. A missing feedback stream
must be diagnosed at the publisher, Agent, and DDS discovery path rather than
hidden with a wider window. The firmware's independent 198 ms per-motor leases
remain the primary motion-loss boundary.

The plugins use a fixed 1500 ms heartbeat freshness limit matching the
configuration supervisor. Missing, invalid, or stale required feedback;
missing, stale, or not-ready heartbeat; invalid authorization; Agent-session
change; and MCU-uptime regression enter transparent recovery. Recovery
publishes zero motor commands, centers Ackermann PWM3, resets the chassis
ADRCs and measurement filters, exposes zero wheel velocities, and freezes the
last exported positions. The hardware lifecycle and controller interface
claims remain active.

Recovery requires a later heartbeat and a later valid sample from every
vehicle-required feedback stream. After an Agent-session change or MCU-uptime
regression, the supervisor generation must also be nonzero, match the current
session, and differ from the last accepted generation. This prevents cached
feedback or a transient-local authorization from the previous session from
rearming motion.

The measured Ackermann runtime geometry is a `0.135 m` wheelbase, `0.140 m`
wheel track, `0.0325 m` wheel radius, and `+/-0.6 rad` steering limit. Physical
mocap already measures `base_footprint` at the geometry center, so no
rear-axle shift is applied to it. In the URDF,
`rear_axle_footprint` is a fixed child `0.0675 m` behind `base_footprint`.
The retained visual/collision wheel coordinates place the imported Mentor Pi
STL visuals against the controller geometry. Meshes remain visualization-only:
primitive collision and inertial data are unchanged and are not controller
authority. Camera and lidar meshes are composed into `base_link` without adding
TF frames. Their provenance and separate `NOASSERTION` status are recorded with
the installed configuration.

`make onboard-configure` regenerates both profiles after a type or namespace
change and flashes firmware with the same namespace. Multiple robots may share
domain 42 because their ROS entity and TF names are distinct.

## Numerical plant

The simulation plugins preserve the logical ROS joint order and do not apply
MCU connector signs. Both vehicle types clamp driven-wheel velocity to
`+/-37.699112 rad/s`, rate-limit it to `+/-188.495559 rad/s^2`, and integrate
wheel position trapezoidally at the existing 30 Hz controller-manager period.
Ackermann averages the two commanded front steering positions into one
physical steering state, clamps it to `+/-0.6 rad`, rate-limits it to
`60 rad/s`, and reports that state through both steering joints.

The upstream controllers derive an internal numerical pose from those joint
states. The common adapter applies the requested initial SE(2) pose and, for
Ackermann, the rear-axle-to-center offset, then publishes `vehicle/pose` and
`map -> base_footprint`. The first public pose is exactly the requested
geometry-center pose. Simulation publishes no `/clock` and models no noise,
slip, collisions, terrain, IMU, or identified actuator lag. It is a numerical
control and kinematics test, not HIL or qualification evidence. See
[simulation.md](../simulation.md).

## Units and connector mapping

`ros2_control` uses radians and radians/second. Firmware motor messages use
encoder counts and revolutions/second. The plugins therefore apply exact
`2*pi` conversions in both directions and compute position from the active
motor profile's ticks per revolution.

The firmware connector order is authoritative. The ROS hardware adapters apply
the chassis-direction sign symmetrically to outgoing commands and incoming
position/velocity feedback:

| MCU connector | ros2_control joint | Chassis-direction sign |
| --- | --- | ---: |
| M1 | front-left | -1 |
| M2 | rear-left | -1 |
| M3 | front-right | +1 |
| M4 | rear-right | +1 |

Positive ROS wheel rotation rolls the chassis toward `+X`. Right-side motors
are mechanically mirrored. Under the qualified connector/encoder convention,
positive ROS commands become negative firmware targets for M1/M2 and positive
targets for M3/M4. The same map converts feedback in the opposite direction.
This chassis conversion is independent of the fixed MCU bridge inversion.

Ackermann drive commands select only M2 and M4. Steering uses the configured
PWM channel.

## Chassis feedback control

Both hardware plugins run first-order linear ADRC at the existing 30 Hz
`ros2_control` update rate, after the drive controller has produced its wheel
or steering references. Consequently manual commands and tracker/MPC commands
use the same feedback path without a new node or topic.

Mecanum reconstructs reference and measured body `vx` and `vy` from the four
wheel velocities. It reconstructs reference yaw rate from wheel commands and
uses `/mentor_pi/imu.angular_velocity_rad_s[2]` as measured yaw rate. Independent
5 Hz first-order filters process measured `vx`, `vy`, and yaw rate before the
ADRCs. Corrections are mapped back through the mecanum kinematics and uniformly
scaled if any wheel would exceed the active motor profile.

Ackermann reconstructs longitudinal speed from the two rear wheels and applies
one common ADRC speed correction, preserving their requested differential. Its
yaw reference is `speed * tan(steering) / wheelbase`; gyro Z is the measured
yaw rate. Independent 5 Hz first-order filters process longitudinal speed and
yaw rate before the ADRCs. The yaw input gain is the configured coefficient
times signed filtered speed. Below `0.1 m/s` measured speed, yaw ADRC and its
measurement filter reset while clamped feedforward steering is retained,
because steering cannot reliably close the yaw loop at standstill.

Both vehicle profiles expose `linear_adrc_measurement_lpf_cutoff_hz` and
`yaw_adrc_measurement_lpf_cutoff_hz`, defaulting to `5.0`. The filters use the
measured control period with `alpha = 1 - exp(-2*pi*cutoff*period)`, seed from
the first valid measurement, never filter references, and reset with the
associated chassis ADRC safety path.

The chassis defaults are provisional: linear input gain `5 s^-1`, controller
bandwidth `1 rad/s`, and observer bandwidth `3 rad/s`; mecanum yaw uses the same
values, while Ackermann yaw uses coefficient `30` times measured speed. Missing,
invalid, or older-than-100-ms IMU or actuator feedback resets chassis ADRC,
publishes zero/center commands, and enters transparent recovery. Acceleration
is not integrated for velocity feedback.

## Lifecycle and failure behavior

Each plugin owns a single-threaded ROS executor and a lifecycle-managed worker
thread. It starts during configuration and is cancelled and joined during
cleanup or destruction. Callback and control-loop state is mutex protected.

The plugin publishes zero actuator commands when it deactivates, when feedback
is stale, when a command is non-finite or exceeds the active motor profile, or
when a control operation cannot safely continue. It also requires a nonzero
configuration-generation and agent-session pair encoded in the transient-local
`/mentor_pi/configuration/motion_authorization` value. The session word must
match a current READY/DEGRADED MCU heartbeat, with exactly one authorization
publisher: `/mentor_pi/configuration_supervisor`. An absent, zero-generation,
zero-session, duplicated, session-mismatched, or differently-owned
authorization rejects the write and publishes zero. Queues are best-effort
depth one, matching the firmware command/state contract. Firmware remains the
final authority for command validation, model and implementation limits,
independent 198 ms leases, and session-loss disarming.

Recoverable communication, session, authorization, and required-feedback
interruptions are expected inhibited states, not `ros2_control` hardware
failures: `read` and `write` return `OK` so interfaces and controller claims
remain available. An uninterrupted live controller reference resumes only
after the complete recovery gate passes; each drive controller's existing
100 ms reference timeout prevents an abandoned reference from replaying.

On Humble controller manager 2.54, a hardware `read()` or `write()` error calls
the SystemInterface error transition and removes its interfaces. A successful
default `on_error()` leaves the component `unconfigured`; controller manager
does not automatically configure, activate, and reclaim its controllers.
Therefore `ERROR` is reserved for local plugin failures such as executor
failure, a non-finite command, or an invalid control calculation. Both plugins
override `on_error()` to send zero, stop and join the private executor, release
ROS endpoints, clear cached state, and leave the component safely
reconfigurable. See the
[ros2_control 2.54 implementation](https://github.com/ros-controls/ros2_control/blob/2.54.0/hardware_interface/src/system.cpp#L213-L253).

## Developer entry points

Use native Ubuntu 22.04/Humble onboard. macOS and other Linux distributions may
build/test through the Dev Container but cannot run the robot there.

```sh
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test
systemctl is-active mentor-pi-agent.service
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
: "${ROS_DOMAIN_ID:?export the deployment ROS_DOMAIN_ID first}"
ros2 launch mentor_pi_hardwares vehicle.launch.py

# Development-only numerical plant; vehicle_type is required.
ros2 launch mentor_pi_hardwares simulation.launch.py \
  vehicle_type:=ackermann robot_name:=ackermann_sim

# Optional shared visualization bridge.
ros2 launch mentor_pi_hardwares foxglove.launch.py
```

The default ADRC firmware accepts bounded nonzero commands only after its normal
session and configuration gates are satisfied. Loading a hardware plugin never
bypasses the firmware limits, per-motor leases, or guarded-HIL prerequisites.
