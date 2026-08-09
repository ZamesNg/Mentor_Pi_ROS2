# Normal Computer Tutorial 01: Prepare Ubuntu 24.04

Start here on the normal computer connected to the Mentor Pi. This track is
for Ubuntu 24.04 and uses the pinned Ubuntu 22.04/ROS 2 Humble Docker build and
runtime. Do not install ROS natively. Architectures are detected automatically.

**Run on:** normal computer, Ubuntu 24.04 `amd64` or `arm64`
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
sudo apt-get update && sudo apt-get install -y git make g++ unzip libusb-1.0-0-dev ca-certificates docker.io
```

Enable Docker and grant the current login access:

```sh
sudo systemctl enable --now docker && sudo usermod -aG docker "${USER}"
```

Log out and back in after the group change. Do not install ROS 2 Humble or
Jazzy on the Ubuntu 24.04 host; the repository supplies pinned Humble containers.

## 3. Install the headless STM32 programmer

The ST download `SetupSTM32CubeProgrammer_linux_64.zip` is correct for Linux
`amd64`. Install it with its console installer. The expected CLI path on this
server is:

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

Require Ubuntu 24.04, `amd64` or `arm64`, at least 10 GiB free, a working
Docker engine, and an absolute CubeProgrammer CLI path.

```sh
make setup
```

Expected result: `Setup complete`. Stop on any OS, architecture, Docker,
programmer, checksum, network, or dependency error. Generated dependencies and
build outputs remain ignored and must not be committed.

Next: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md).
