# Normal Computer Tutorial 03: Build and Run Humble in Docker

Run ROS 2 Humble in the pinned container on the Ubuntu 24.04 normal computer
connected to the MCU. There is no native ROS installation or architecture
override on this track.

**Run on:** normal computer, Ubuntu 24.04 `amd64` or `arm64`
**Hardware state:** verified default PID firmware; all actuators disconnected

Previous: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md)
Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)

## 1. Build the host

```sh
cd /home/zames/Mentor_Pi && make host
```

Expected result: the architecture-matched Humble host passes its tests and is
available below `build/runtime/`. Stop on an OS, architecture, dependency,
relocation, test, or checksum failure. `make start` below prepares or reuses
the pinned Agent automatically.

## 2. Start the PID runtime

**Warning:** Disconnect motor power, all four PWM servos, and all bus servos.
Contain every wheel; the PID firmware accepts guarded nonzero motor commands
and still emits neutral PWM-servo pulses.

```sh
cd /home/zames/Mentor_Pi && make start
```

Type `PID_FIRMWARE_ACTUATORS_PREPARED` when prompted. The command
validates `/dev/mentor_pi_mcu`, verifies the PID artifact, selects native or
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

Expected result: an enhanced zsh Humble shell using domain 0, with Oh My Zsh,
Tab completion, autosuggestions, and syntax highlighting. `ros2 node list`
must contain `/mentor_pi/controller` and `/mentor_pi/configuration_supervisor`.
Keep the `make start` terminal open until the connected checks are finished.

The launch inside `make start` validates the development artifact and serial
device and shuts down if either the Agent or supervisor exits. Use `make shell`
for every ROS 2 CLI terminal so commands execute inside that same container.

Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
