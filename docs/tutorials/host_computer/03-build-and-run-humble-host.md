# Tutorial 03: Build and Run the Humble Host

Build and run ROS 2 Humble in the pinned Docker image on the development
computer connected to the MCU.

**Run on:** normal Ubuntu development computer
**Hardware state:** verified default PID firmware; all actuators disconnected

Previous: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md)
Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)

## 1. Build the host

```sh
cd "${HOME}/Mentor_Pi" && make host
```

The architecture-matched Humble workspace builds and runs its focused tests.
Stop on dependency, relocation, test, or checksum failure.

## 2. Start the development runtime

**Warning:** Disconnect motor power, all PWM servos, and all bus servos.
Contain every wheel; the PID firmware accepts valid nonzero commands.

```sh
cd "${HOME}/Mentor_Pi" && make start
```

Type `PID_FIRMWARE_ACTUATORS_PREPARED` when prompted. Keep this terminal open.
The command starts the Agent and configuration supervisor as one fail-coupled
launch and reports progress instead of remaining silent.

## 3. Open a ROS shell

In a second terminal:

```sh
cd "${HOME}/Mentor_Pi" && make shell ROS_DOMAIN_ID=0
```

`ros2 node list` must contain `/mentor_pi/controller` and
`/mentor_pi/configuration_supervisor`. Use `make shell` for ROS CLI commands;
do not install or source ROS on the host.

Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
