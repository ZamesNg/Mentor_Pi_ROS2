# `mentor_pi_hardwares` architecture

`mentor_pi_hardwares` is the host-side `ros2_control` adapter for the generated
robot namespace. It does not replace the micro-ROS Agent or the
configuration supervisor. The Agent is an external systemd prerequisite; the
manual application launch owns the supervisor and ROS application processes.

## Runtime topology

The unified top-level launch contains three independently managed
application processes:

1. the configuration supervisor from `controller.launch.py`;
2. `robot_state_publisher`;
3. `controller_manager`, which loads one `SystemInterface` plugin and spawns
   the selected drive controller plus `joint_state_broadcaster`.

Use `vehicle.launch.py`; onboarding installs its generated `vehicle.yaml` and
matching controller profile. The profile's `robot_name` namespaces firmware
and host APIs, controller nodes, tracking interfaces, and TF frame IDs. An
absolute `vehicle_config` override is retained for development; its sibling
`controllers.yaml` is used automatically.

## Configuration ownership

The selected absolute `vehicle_config` YAML is the sole authority for
`vehicle.robot_name` and `vehicle.vehicle_type`. Whether to include bringup and
the optional tracking controller remain launch arguments. Serial/Agent
configuration belongs to `mentor-pi-agent.service`, not application launch.
Mode-specific profiles under `config/mecanum` and `config/ackermann` own:

- vehicle geometry and joint names;
- the `ros2_control` URDF and plugin selection;
- controller parameters;
- wheel geometry and chassis ADRC parameters;
- `feedback_timeout_ms` and `imu_timeout_ms`, both defaulting to 500 ms;
- Ackermann PWM channel, min/center/max pulse, inversion, angle limits, and
  command duration.

The 500 ms host feedback windows tolerate ROS scheduling and discovery jitter;
they do not extend motor authority. The firmware's independent 198 ms
per-motor leases remain the primary motion-loss boundary.

The measured Ackermann runtime geometry is a `0.135 m` wheelbase, `0.140 m`
wheel track, `0.0325 m` wheel radius, and `+/-0.6 rad` steering limit. Its
odometry reference is the rear-axle midpoint `rear_axle_footprint`; a fixed
`0.0675 m` transform places the existing `base_footprint` at the geometry
center. The retained visual/collision wheel coordinates and visual radius are
illustrative and are not the controller's kinematic authority.

`make onboard-configure` regenerates both profiles after a type or namespace
change and flashes firmware with the same namespace. Multiple robots may share
domain 42 because their ROS entity and TF names are distinct.

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
| M1 | front-left | +1 |
| M2 | rear-left | +1 |
| M3 | front-right | -1 |
| M4 | rear-right | -1 |

Positive ROS wheel rotation rolls the chassis toward `+X`. Right-side motors
are mechanically mirrored, so positive ROS commands become negative firmware
targets for M3/M4, and negative M3/M4 firmware feedback becomes positive ROS
wheel state. This chassis conversion is independent of firmware encoder A/B
wiring normalization.

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
ADRC corrections are mapped back through the mecanum kinematics and uniformly
scaled if any wheel would exceed the active motor profile.

Ackermann reconstructs longitudinal speed from the two rear wheels and applies
one common ADRC speed correction, preserving their requested differential. Its
yaw reference is `speed * tan(steering) / wheelbase`; gyro Z is the measured
yaw rate. The yaw input gain is the configured coefficient times signed
measured speed. Below `0.1 m/s` measured speed, yaw ADRC resets and steering is
centered because steering cannot reliably control yaw at standstill.

The chassis defaults are provisional: linear input gain `5 s^-1`, controller
bandwidth `1 rad/s`, and observer bandwidth `3 rad/s`; mecanum yaw uses the same
values, while Ackermann yaw uses coefficient `30` times measured speed. Missing,
invalid, or older-than-100-ms IMU or actuator feedback resets chassis ADRC,
publishes zero/center commands, and enters the existing authorized hardware
error path. Acceleration is not integrated for velocity feedback.

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

Missing or invalid authorization is an expected inhibited state, not a
`ros2_control` hardware failure: `read` and `write` keep their interfaces
available while publishing zero commands. Executor failure, an invalid command
while authorized, or stale feedback while authorized returns an error and
triggers the fail-coupled hardware shutdown.

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
```

The default ADRC firmware accepts bounded nonzero commands only after its normal
session and configuration gates are satisfied. Loading a hardware plugin never
bypasses the firmware limits, per-motor leases, or guarded-HIL prerequisites.
