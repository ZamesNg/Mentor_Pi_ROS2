# Tutorial 01: Prepare the Host Computer

Prepare a normal Ubuntu development computer for local firmware and ROS 2
Humble builds. ROS stays inside the pinned architecture-native Docker images.

**Run on:** normal Ubuntu amd64 or arm64 development computer
**Hardware state:** board and actuators disconnected

There is no previous tutorial.

**Warning:** Keep the board and every actuator disconnected during host setup.
Do not install or source host ROS and do not apply motor power.

## 1. Install host tools

```sh
cd "${HOME}/Mentor_Pi"
sudo apt-get update && sudo apt-get install -y \
  ca-certificates docker.io git make libusb-1.0-0-dev unzip zsh
sudo systemctl enable --now docker
```

Grant the login Docker access if required with
`sudo usermod -aG docker "${USER}"`, then log out and back in.

Install STM32CubeProgrammer from ST's matching host package. The common amd64
installation is detected here:

```sh
test -x "${HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
```

## 2. Verify and prepare the repository

```sh
cd "${HOME}/Mentor_Pi"
make doctor
make setup
```

Require a supported Ubuntu host, a working Docker engine, and sufficient disk
space. `make setup` builds the pinned local images and fetches the ignored,
pinned firmware dependencies. Stop on any OS, architecture, Docker, checksum,
network, programmer, or dependency error.

Next: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md).
