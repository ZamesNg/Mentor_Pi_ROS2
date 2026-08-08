# Tutorial 09: Run the `mentor_pi_hardwares` integration

Run the coordinated ROS 2 Humble stack for mecanum or Ackermann using the
native `/mentor_pi` firmware endpoints.

**Host:** Ubuntu 22.04 natively; another Ubuntu release through the pinned
Ubuntu 22.04/Humble Docker runtime  
**Initial fixture:** actuator power disconnected and wheels contained

Previous: [Tutorial 08: Run Stress, Soak, and Release Gates](08-run-stress-soak-and-release-gates.md)
Next: none

**Warning:** Keep actuator power disconnected until passive direction checks
are complete. Any powered-wheel validation requires raised or equivalently
guarded wheels, a current-limited supply, and an independent stop.

## 1. Build and verify the host

```sh
cd /home/zames/Mentor_Pi
make host
```

`make host-hardwares` is a compatibility alias for the same adaptive build and
test path. Both interfaces, bringup, and `mentor_pi_hardwares` are built and
tested together.

## 2. Start one vehicle mode

For mecanum:

```sh
make start-mecanum \
  PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0
```

For Ackermann:

```sh
make start-ackermann \
  PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0
```

The command asks for the normal locked-runtime acknowledgement. It verifies
the firmware artifact, serial device identity, host overlay, and Agent before
starting the mode-specific top-level launch. The Agent, supervisor,
`robot_state_publisher`, and controller manager remain separate processes.

Each convenience target selects its checked-in `hardware.yaml` deployment
profile. The profile is the sole source of `vehicle.robot_name` and
`vehicle.vehicle_type`; there is no direct environment, Make, shell, or launch
override for either value. MCU transport and supervisor endpoints remain the
fixed public `/mentor_pi/*` API.

For a custom or additional robot, copy one complete profile, edit its vehicle
mapping and calibration, then start it explicitly:

```sh
make start-hardware PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=1 \
  VEHICLE_CONFIG=/absolute/robot_two.yaml
```

Use a distinct `ROS_DOMAIN_ID` for each physical robot because their compatible
firmware endpoints all use `/mentor_pi/*`.

## 3. Inspect the graph

Open a second terminal:

```sh
cd /home/zames/Mentor_Pi
make shell ROS_DOMAIN_ID=0
ros2 control list_hardware_interfaces -c /mentor_pi/controller_manager
ros2 control list_controllers -c /mentor_pi/controller_manager
ros2 topic list | grep -E '/mentor_pi/(motors|pwm_servos)'
```

Mecanum exports velocity command and position/velocity state interfaces for
all four wheels. Ackermann exports front steering position interfaces and rear
wheel velocity command plus position/velocity state interfaces.

The deterministic MCU mapping is M1 front-left, M2 rear-left, M3 front-right,
and M4 rear-right. Ackermann uses M2 and M4 for its rear wheels.

## 4. Review deployment configuration

Before applying actuator power, review the files below the selected mode in
`mentor_pi_ros2/src/mentor_pi_hardwares/config/`:

- `controllers.yaml`: geometry, joint names, and controller settings;
- `hardware.yaml`: robot name, vehicle type, feedback timeout and, for
  Ackermann, steering calibration;
- `mentor_pi.urdf.xacro`: installed robot and `ros2_control` description.

The default feedback timeout is 100 ms. Stale or invalid feedback, non-finite
controller output, commands beyond the active motor profile, deactivation, and
lifecycle failures cause zero commands to be published. Nonzero controller
writes also require the supervisor's unique motion authorization with nonzero
configuration-generation and session words; the session must match the current
READY/DEGRADED MCU heartbeat. Only the
`/mentor_pi/configuration_supervisor` node may own that publisher; absence,
revocation, duplication, a different owner, or a session change fails closed
to zero.

## 5. Safety boundary

The normal firmware remains motor-locked, so nonzero motor commands are
rejected. Loading `mentor_pi_hardwares` does not enable motion or bypass the
configuration supervisor, independent 198 ms leases, or transport-loss
disarming.

Any motion test uses the separately acknowledged commissioning flow with raised
wheels, a current-limited supply, and recorded HIL evidence. Software tests do
not qualify wheel direction, steering geometry, or physical PID performance.

Stop the coordinated launch with Ctrl-C in its terminal. Deactivation publishes
zero motor commands and, in Ackermann mode, centers the configured steering
channel.
