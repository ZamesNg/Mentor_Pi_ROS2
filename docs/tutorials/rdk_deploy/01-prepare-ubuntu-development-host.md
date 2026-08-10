# Tutorial 01: Prepare and Receive the RDK Deployment

Build the pinned arm64 production handoff on a normal amd64 Ubuntu computer,
transfer it, then receive and verify it once on the RDK X5. The RDK does not
compile the production firmware, Agent, or ROS workspace.

**Run on:** amd64 Ubuntu preparation computer, then RDK X5 Ubuntu 22.04 arm64
**Hardware state:** board and actuators disconnected

There is no previous tutorial.

**Warning:** Keep the board, motors, PWM servos, and bus servos disconnected.
Do not start production or apply actuator power in this tutorial.

## 1. Prepare the normal computer

```sh
cd "${HOME}/Mentor_Pi"
sudo apt-get update && sudo apt-get install -y \
  binfmt-support ca-certificates docker.io git make qemu-user-static rsync
sudo systemctl enable --now docker
make doctor
make setup
```

The repository uses a pinned linux/arm64 Docker build and checks QEMU/binfmt
before building. It never installs or sources host ROS.

## 2. Build and transfer the handoff

```sh
cd "${HOME}/Mentor_Pi"
make rdk-handoff
make rdk-transfer RDK_LOGIN=sunrise@rdk-hostname
```

The transfer helper selects the newest completed timestamped bundle, verifies
it, archives it without changing modes or symlinks, and transfers the archive
and SHA-256 sidecar. To deliberately select another completed bundle, pass
`RDK_HANDOFF=/absolute/path/to/rdk-arm64-<actual-timestamp>`.

## 3. Prepare the RDK

On the RDK, install Docker and the checked arm64 CubeProgrammer package:

```sh
cd "${HOME}/Mentor_Pi"
sudo apt-get update && sudo apt-get install -y ca-certificates docker.io git make zsh
sudo systemctl enable --now docker
sudo ./tools/install_onboard_stm32cubeprogrammer.sh --install
test -x /usr/bin/STM32_Programmer_CLI
```

Grant the login Docker access if required with
`sudo usermod -aG docker "${USER}"`, then log out and back in.

## 4. Receive and verify once

```sh
cd "${HOME}/Mentor_Pi"
make rdk-receive
```

This verifies the computer, archive checksum, extraction layout, complete
bundle manifest, and timestamp, then records the verified bundle receipt.
Later production commands use that receipt and perform only their
operation-specific checks; they do not rehash the complete handoff.

Stop on any architecture, checksum, extraction, Docker, or programmer error.
Do not run `make host` on this deployment track.

Next: [Tutorial 02: Flash the Packaged PID Firmware](02-build-and-flash-default-pid-firmware.md).
