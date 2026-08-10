# Tutorial 03: Install and Start RDK Production

Install the already-built arm64 runtime image, Agent, host release, serial
identity rule, and systemd units from the received handoff.

**Run on:** RDK X5 Ubuntu 22.04 arm64
**Hardware state:** verified packaged PID firmware; all actuators disconnected

Previous: [Tutorial 02: Flash the Packaged PID Firmware](02-build-and-flash-default-pid-firmware.md)
Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)

## 1. Read the actual adapter identity

```sh
cd "${HOME}/Mentor_Pi"
udevadm info --query=property --name=/dev/mentor_pi_mcu | \
  grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)='
```

Use the exact value printed after `ID_SERIAL_SHORT=`:

```sh
make production-install ROS_DOMAIN_ID=0 ID_SERIAL_SHORT=596F060000
```

Replace `596F060000` with the value observed on this RDK. Only when
`ID_SERIAL_SHORT` is empty, use the complete observed `ID_PATH` value:

```sh
make production-install ROS_DOMAIN_ID=0 \
  ID_PATH='platform-xhci-hcd.2.auto-usb-0:1.1:1.0'
```

The compact installer uses the verified receipt, reports progress, loads the
arm64 image, installs the Agent and host release, adopts a compatible
development serial rule when present, and verifies the units. Do not run `make host`
on this deployment track.

## 2. Start production

**Warning:** Keep motor power, PWM servos, and bus servos disconnected. The
target starts the micro-ROS Agent and configuration supervisor; it does not
authorize powered motion by itself.

```sh
sudo systemctl enable --now mentor-pi-controller.target
systemctl status mentor-pi-controller.target mentor-pi-runtime.service
journalctl -u mentor-pi-runtime.service
```

Once active, ROS topics and services are available through the production
container. Open the pinned ROS shell with:

```sh
cd "${HOME}/Mentor_Pi" && make shell ROS_DOMAIN_ID=0
```

The `make shell` command attaches to `mentor-pi-production`; no separate
`make start` process is needed. Stop on an unexpected identity, installation
error, repeated reset, transport failure, or absent controller.

Next: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
