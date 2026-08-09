# Normal Computer Tutorial 08: Run the `mentor_pi_hardwares` Integration

Run the coordinated ROS 2 Humble stack for a mecanum or Ackermann vehicle using
the native `/mentor_pi` firmware endpoints.

**Run on:** normal computer through the pinned Ubuntu 22.04/Humble Docker runtime
**Initial fixture:** actuator power disconnected and wheels contained

Previous: [Tutorial 07: Run Stress, Soak, and Release Gates](07-run-stress-soak-and-release-gates.md)
Next: none

**Warning:** The default PID firmware accepts valid nonzero motor commands.
Keep actuator power disconnected until Tutorials 01--06 are complete. Powered
validation requires raised or equivalently guarded wheels, a current-limited
supply, a reachable stop, and recorded HIL evidence.

## 1. Build and verify the host

```sh
cd /home/zames/Mentor_Pi && make host
```

`make host-hardwares` is an alias for the same adaptive build and test path.
Interfaces, bringup, and `mentor_pi_hardwares` build together.

## 2. Start one vehicle mode

For mecanum:

```sh
cd /home/zames/Mentor_Pi && make start-mecanum \
  PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0
```

For Ackermann:

```sh
cd /home/zames/Mentor_Pi && make start-ackermann \
  PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=0
```

The command requests `PID_FIRMWARE_ACTUATORS_PREPARED`, validates the firmware
artifact and serial device, and starts the Python vehicle launch. The launch
includes the Python controller launch for the Agent and supervisor, then starts
`robot_state_publisher`, controller manager, and the selected controllers. A
critical process exit shuts the coordinated launch down.

Each convenience target selects its checked-in `hardware.yaml`. For another
reviewed robot profile:

```sh
cd /home/zames/Mentor_Pi && make start-hardware \
  PORT=/dev/mentor_pi_mcu ROS_DOMAIN_ID=1 \
  VEHICLE_CONFIG=/absolute/robot_two.yaml
```

Use a distinct `ROS_DOMAIN_ID` for each robot because every compatible MCU
uses the fixed `/mentor_pi/*` endpoint namespace.

## 3. Inspect the graph

Open a second terminal:

```sh
cd /home/zames/Mentor_Pi && make shell ROS_DOMAIN_ID=0
ros2 control list_hardware_interfaces -c /mentor_pi/controller_manager
ros2 control list_controllers -c /mentor_pi/controller_manager
ros2 topic list | grep -E '^/mentor_pi/(motors|pwm_servos)'
```

Mecanum exports velocity command plus position/velocity state interfaces for
all four wheels. Ackermann exports front steering position interfaces and rear
wheel velocity command plus position/velocity state interfaces.

The MCU mapping is M1 front-left, M2 rear-left, M3 front-right, and M4
rear-right. Ackermann uses M2 and M4 for rear-wheel drive.

## 4. Review deployment configuration

Before actuator power, review the selected files under
`mentor_pi_ros2/src/mentor_pi_hardwares/config/`:

- `controllers.yaml`: geometry, joint names, and controller settings;
- `hardware.yaml`: robot identity, vehicle type, feedback timeout, and steering
  calibration where applicable;
- `mentor_pi.urdf.xacro`: robot and `ros2_control` description.

Stale or invalid feedback, non-finite controller output, commands beyond the
active motor profile, deactivation, and lifecycle failures publish zero motor
commands. Nonzero controller writes additionally require the supervisor's sole
transient-local motion authorization bound to the current configuration
generation and Agent session. A missing, zero, duplicated, foreign, or stale
authorization fails closed to zero.

## 5. Stop safely

Stop on wrong wheel ownership, steering direction, oscillation, unexpected
current, heat, or any transport/session transition. Software tests do not
qualify wheel direction, steering geometry, or physical PID performance.

Press Ctrl-C in the launch terminal. Deactivation publishes zero motor
commands and, for Ackermann, centers the configured steering channel. Confirm
physical stop, then remove actuator power.
