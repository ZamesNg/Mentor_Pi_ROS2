# Tutorial 03: Build and Run the Humble Host

Run ROS 2 Humble on the computer connected to the MCU. Ubuntu 22.04 runs
natively; every other Ubuntu release uses Docker automatically. There is no
host handoff, architecture override, or native ROS installation on Ubuntu
24.04.

**Run on:** Ubuntu `amd64` or `arm64`
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

Expected result: a sourced Humble shell using domain 0. `ros2 node list` must
contain `/mentor_pi/controller` and `/mentor_pi/configuration_supervisor`.
Keep the `make start` terminal open until the connected checks are finished.

On native Ubuntu 22.04, after `make host agent`, `make shell` can also open the
sourced environment before the runtime starts. From that shell, the equivalent
validated Python launch is:

```sh
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  ros2 launch mentor_pi_bringup controller.launch.py \
    serial_device:=/dev/mentor_pi_mcu
```

The launch validates the development artifact and serial device and shuts down
if either the Agent or supervisor exits. Other supported Ubuntu releases keep
using `make start` so the launch runs inside the pinned Humble container.

Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
