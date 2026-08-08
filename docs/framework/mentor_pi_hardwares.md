# `mentor_pi_hardwares` architecture

`mentor_pi_hardwares` is the host-side `ros2_control` adapter for the native
`/mentor_pi` micro-ROS contract. It does not replace the micro-ROS Agent or the
configuration supervisor. The coordinated launch starts each of these as a
separate process so a controller-manager failure cannot become an Agent
failure, and conversely.

## Runtime topology

The mode-specific top-level launch contains four independently managed
processes:

1. the micro-ROS Agent from `mentor_pi_bringup/controller.launch.xml`;
2. the configuration supervisor from that same included launch;
3. `robot_state_publisher`;
4. `controller_manager`, which loads one `SystemInterface` plugin and spawns
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
`vehicle.robot_name` and `vehicle.vehicle_type`. The serial device, Agent
executable, and whether to include bringup remain launch arguments.
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
final authority for command validation, independent 198 ms leases,
session-loss disarming, and the normal motor lock.

## Developer entry points

On Ubuntu 22.04 these use native ROS 2 Humble. Other supported Ubuntu releases
use the pinned 22.04/Humble runtime container.

```sh
make host
make start-hardware PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0 \
  VEHICLE_CONFIG=/absolute/robot.yaml
make start-mecanum PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0
make start-ackermann PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=1
```

The normal locked firmware still rejects nonzero motion. Commissioning requires
the separate acknowledged firmware and runtime procedures; loading a hardware
plugin never bypasses those gates.
