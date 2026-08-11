# `mentor_pi_hardwares` architecture

`mentor_pi_hardwares` is the host-side `ros2_control` adapter for the native
`/mentor_pi` micro-ROS contract. It does not replace the micro-ROS Agent or the
configuration supervisor. The Agent is an external systemd prerequisite; the
manual application launch owns the supervisor and ROS application processes.

## Runtime topology

The mode-specific top-level launch contains three independently managed
application processes:

1. the configuration supervisor from `controller.launch.py`;
2. `robot_state_publisher`;
3. `controller_manager`, which loads one `SystemInterface` plugin and spawns
   the selected drive controller plus `joint_state_broadcaster`.

Use `vehicle.launch.py` with a selected YAML deployment profile.
`mecanum.launch.py` and `ackermann.launch.py` are convenience wrappers that
select their checked-in profiles, and `exp_vehicle_launch.py` remains only as
a compatibility filename. All four launch files accept `vehicle_config`, not
direct robot-name or vehicle-type values. The profile's `robot_name`
namespaces host-side robot and controller nodes; firmware transport and
supervisor endpoints remain the compatible, absolute `/mentor_pi/*` API.

## Configuration ownership

The selected absolute `vehicle_config` YAML is the sole authority for
`vehicle.robot_name` and `vehicle.vehicle_type`. Whether to include bringup and
the optional tracking controller remain launch arguments. Serial/Agent
configuration belongs to `mentor-pi-agent.service`, not application launch.
Mode-specific profiles under `config/mecanum` and `config/ackermann` own:

- vehicle geometry and joint names;
- the `ros2_control` URDF and plugin selection;
- controller parameters;
- `feedback_timeout_ms`, defaulting to 100 ms;
- Ackermann PWM channel, min/center/max pulse, inversion, angle limits, and
  command duration.

Custom multi-robot deployments copy a profile, change its `robot_name`, and
start it with a distinct `ROS_DOMAIN_ID`. Distinct domains are required because
the firmware-facing API intentionally remains fixed at `/mentor_pi/*`.

## Units and connector mapping

`ros2_control` uses radians and radians/second. Firmware motor messages use
encoder counts and revolutions/second. The plugins therefore apply exact
`2*pi` conversions in both directions and compute position from the active
motor profile's ticks per revolution.

The firmware connector order and polarity are authoritative:

| MCU connector | ros2_control joint |
| --- | --- |
| M1 | front-left |
| M2 | rear-left |
| M3 | front-right |
| M4 | rear-right |

Ackermann drive commands select only M2 and M4. Steering uses the configured
PWM channel.

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
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
ros2 launch mentor_pi_hardwares mecanum.launch.py
```

The default PID firmware accepts bounded nonzero commands only after its normal
session and configuration gates are satisfied. Loading a hardware plugin never
bypasses the firmware limits, per-motor leases, or guarded-HIL prerequisites.
