# Tutorial 01: Prepare the Docker Host

Start here on either the RDK X5 onboard computer or a normal development
computer. Ubuntu 22.04 and 24.04 both use architecture-matched, pinned ROS 2
Humble Docker images. Do not install or source ROS on the host.

**Run on:** RDK X5 Ubuntu 22.04 `arm64`, or a normal Ubuntu 24.04 `amd64`/`arm64` computer
**Hardware state:** board and actuators disconnected

There is no previous tutorial.

## 1. Open the repository

```sh
cd /home/zames/Mentor_Pi
```

All later commands use this exact checkout. Do not set `HOST_ARCH`, install ROS
Jazzy, restore PlatformIO, or run `git clean -fdX`.

## 2. Install basic host packages

```sh
sudo apt-get update && sudo apt-get install -y \
  ca-certificates docker.io git make libusb-1.0-0-dev unzip zsh
```

Enable Docker and grant the current login access:

```sh
sudo systemctl enable --now docker && sudo usermod -aG docker "${USER}"
```

Log out and back in after the group change. Follow Docker's official
[Ubuntu installation guidance](https://docs.docker.com/engine/install/ubuntu/)
if the distribution package is unavailable. The repository supplies ROS 2
[ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) inside Docker;
an existing host ROS installation is ignored.

## 3. Install the headless STM32 programmer

On the RDK X5 onboard computer, install the checked arm64 repository package:

```sh
sudo ./tools/install_onboard_stm32cubeprogrammer.sh --install
test -x /usr/bin/STM32_Programmer_CLI
```

On a normal computer, install the matching package from ST's
STM32CubeProgrammer download and verify its CLI. The usual `amd64` path is:

```sh
test -x "${HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
```

Do not commit the licensed archive or installer. The repository detects this
path automatically; a stable custom installation may instead set
`STM32_CUBE_PROGRAMMER_CLI`.

## 4. Check and prepare everything

```sh
make doctor
```

Require supported Ubuntu, `amd64` or `arm64`, at least 10 GiB free, and a working
Docker engine. CubeProgrammer is optional until `make flash`. The RDK X5 profile is
detected from its device tree and receives the lighter image set. The reported
build-job count is bounded from available memory; set `RRCLITE_BUILD_JOBS` only
when intentionally overriding it.

```sh
make setup
```

Expected result: `Setup complete`. The command pulls native-architecture
images, builds the pinned local derivatives once, and verifies their identity.
Stop on any OS, architecture, Docker, programmer, checksum, network, or
dependency error. Generated dependencies and build outputs remain ignored.

Next: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md).
