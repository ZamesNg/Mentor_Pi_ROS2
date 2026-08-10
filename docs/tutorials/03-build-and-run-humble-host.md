# Tutorial 03: Build and Run Humble in Docker

Run ROS 2 Humble in the pinned architecture-native container on the computer
connected to the MCU. Host ROS installations are neither required nor sourced.

**Run on:** RDK X5 Ubuntu 22.04 `arm64`, or normal Ubuntu computer on `amd64`/`arm64`
**Hardware state:** verified default PID firmware; all actuators disconnected

Previous: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md)
Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)

## Production RDK installation from a transferred bundle

On the RDK, use the exact bundle verified and flashed in Tutorials 01–02.
Do not run `make host`: install its already-tested host and Agent prefixes while
the controller target is inactive. The compact installer requires Tutorial
01's recorded verified receipt, loads and checks the arm64 image, installs and hashes the Agent,
promotes the host release, installs the site configuration and units, and runs
the systemd unit verifier.

Connect exactly one `1a86:55d4` CH9102F. Inspect its current tty and select its
nonempty unique `ID_SERIAL_SHORT`; use the exact `ID_PATH` with
`--identity-kind id-path` only when no unique serial exists.

```sh
udevadm info --query=property --name=/dev/mentor_pi_mcu | \
  grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)='

make production-install ROS_DOMAIN_ID=0 \
  IDENTITY_KIND=serial IDENTITY_VALUE=RRCLITE_A1B2C3
```

Replace the identity example with the value just observed. Tutorial 02's
packaged flash and read-back verification must already have succeeded with all
actuators disconnected. Only then start production and inspect its logs:

```sh
sudo systemctl enable --now mentor-pi-controller.target
systemctl status mentor-pi-controller.target mentor-pi-runtime.service
journalctl -u mentor-pi-runtime.service
```

Stop on an image-ID/platform mismatch, install failure, unexpected serial
identity, unit verification error, repeated reset, transport failure, or absent
controller. Native RDK compilation remains optional diagnostics; image load,
runtime, memory, tracker timing, serial, peripheral, and HIL gates remain
native work.

## 1. Build the host for connected development

Skip this section when deploying the production RDK bundle.

```sh
cd "${HOME}/Mentor_Pi" && make host
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
cd "${HOME}/Mentor_Pi" && make start
```

Type `PID_FIRMWARE_ACTUATORS_PREPARED` when prompted. The command
validates `/dev/mentor_pi_mcu`, verifies the PID artifact, starts the hardened
Docker Humble runtime, and starts the Agent at 1,000,000 baud/8N1 with
`ROS_DOMAIN_ID=0`. Its DTR/RTS sequence resets into normal application boot;
do not run another serial reader or reset guard.

Expected result: Agent client/session creation and supervisor state `READY` or
an explained `DEGRADED`. LED3 toggles as successful heartbeat publications
advance without pressing RST. Stop on repeated reset, buzzer alarms, port ownership errors,
transport failure, or an absent controller.

The lower-level tracker is intentionally not started by `make start`. Before an
opt-in tracking run, verify that the onboard clock is synchronized; the
repository only checks synchronization and does not reconfigure NTP or chrony:

```sh
./tools/check_time_sync.sh
```

Trajectory planning and frame transforms remain on the high-level computer.
It sends a future-scheduled trajectory already expressed in `odom`; accepted
trajectories continue locally through later planner/network loss.

## 3. Open a second ROS terminal

```sh
cd "${HOME}/Mentor_Pi" && make shell
```

Expected result: an enhanced zsh Humble shell using domain 0, with Oh My Zsh,
Tab completion, autosuggestions, and syntax highlighting. `ros2 node list`
must contain `/mentor_pi/controller` and `/mentor_pi/configuration_supervisor`.
For connected development, keep the `make start` terminal open until the
checks finish. On a production RDK, the same command attaches to the running
`mentor-pi-production` container managed by `mentor-pi-controller.target`; it
does not require a separate `make start` process.

The launch inside `make start` validates the development artifact and serial
device and shuts down if either the Agent or supervisor exits. Use `make shell`
for every ROS 2 CLI terminal so commands execute inside that same container.

Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
