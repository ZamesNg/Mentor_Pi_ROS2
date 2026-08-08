# Tutorial 03: Build and Run the Humble Host

Run ROS 2 Humble on the computer connected to the MCU. Ubuntu 22.04 runs
natively; every other Ubuntu release uses Docker automatically. There is no
host handoff, architecture override, or native ROS installation on Ubuntu
24.04.

**Run on:** Ubuntu `amd64` or `arm64`
**Hardware state:** verified locked firmware; all actuators disconnected

Previous: [Tutorial 02: Build and Flash the Locked Firmware](02-build-and-flash-locked-firmware.md)
Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)

## 1. Build the host

```sh
cd /home/zames/Mentor_Pi && make host
```

Expected result: the architecture-matched Humble host passes its tests and is
available below `build/runtime/`. Stop on an OS, architecture, dependency,
relocation, test, or checksum failure. `make start` below prepares or reuses
the pinned Agent automatically.

## 2. Start the locked runtime

**Warning:** Disconnect motor power, all four PWM servos, and all bus servos.
Contain every wheel; locked firmware still emits neutral PWM-servo pulses.

```sh
cd /home/zames/Mentor_Pi && make start
```

Type `LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED` when prompted. The command
validates `/dev/mentor_pi_mcu`, verifies the locked artifact, selects native or
Docker Humble, and starts the Agent at 1,000,000 baud/8N1 with
`ROS_DOMAIN_ID=0`. Its DTR/RTS sequence resets into normal application boot;
do not run another serial reader or reset guard.

Expected result: Agent client/session creation and supervisor state `READY` or
an explained `DEGRADED`. LED3 toggles as successful heartbeat publications
advance without pressing RST. Stop on repeated reset, buzzer alarms, port ownership errors,
transport failure, or an absent controller.

## 3. Open a second ROS terminal

```sh
cd /home/zames/Mentor_Pi && make shell
```

Expected result: a sourced Humble shell using domain 0. `ros2 node list` must
contain `/mentor_pi/controller` and `/mentor_pi/configuration_supervisor`.
Keep the `make start` terminal open until the connected checks are finished.

For commissioning firmware, never use `make start`; the separate guarded
command is `make start-commissioning` and is introduced only after the passive
hardware gates.

Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
