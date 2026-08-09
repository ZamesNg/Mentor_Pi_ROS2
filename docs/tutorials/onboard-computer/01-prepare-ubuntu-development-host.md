# Onboard Computer Tutorial 01: Prepare the RDK X5

Prepare the RDK X5 onboard computer for Docker-free ROS 2 Humble, host, Agent,
and firmware builds. This track supports Ubuntu 22.04 arm64 only.

**Run on:** onboard computer (RDK X5), Ubuntu 22.04 arm64
**Hardware state:** board and actuators disconnected

There is no previous tutorial.

## 1. Open the repository

```sh
cd /home/zames/Mentor_Pi
```

All later commands use this exact checkout. Do not install Docker, ROS 2 Jazzy,
PlatformIO, or a second Arm toolchain, and never run `git clean -fdX`.

## 2. Provide native ROS 2 Humble

ROS 2 Humble is the one prerequisite that this repository does not install.
Install ROS 2 Humble on Ubuntu 22.04 arm64 by following the official
[ROS 2 Humble Ubuntu deb-package guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
Complete the guide's repository setup and ROS installation before continuing;
do not install ROS 2 Jazzy or use an unofficial mixed-distribution setup.
Run every interactive command in this onboard tutorial track from the RDK X5
user's existing zsh session; the repository does not change the login shell or
install or edit any zsh configuration.

Verify the prerequisite:

```sh
source /opt/ros/humble/setup.zsh
test -n "${ZSH_VERSION}"
test "${ROS_DISTRO}" = humble
test "$(dpkg --print-architecture)" = arm64
```

Stop if any check fails. Do not use this track on another Ubuntu release.

## 3. Install native build dependencies

```sh
cd /home/zames/Mentor_Pi
sudo ./tools/prepare_host_build_dependencies.sh
```

The helper installs conventional `rosdep`/`colcon`, native compiler and build
tools, archive utilities, and the remaining Ubuntu dependencies. Because the
Humble repository does not publish `ros-humble-micro-ros-setup` for this arm64
host, the helper fetches the upstream `micro_ros_setup` 3.1.3 source at the
repository-locked commit, resolves its dependencies with `rosdep`, builds it
under `/opt/mentor_pi`, and verifies its ROS package prefix. The upstream
manifest's unused `clang-tidy` dependency is excluded from this production
package build because analysis and fuzzing are intentionally not onboard
gates; all dependencies used by the installer and `generate_lib` workflow are
still resolved. It also verifies the checked-in
[`stm32cubeprogrammer_2.23.0_arm64.deb.zip`](../../../thirdpart/stm32cubeprogrammer_2.23.0_arm64.deb.zip)
at SHA-256
`99d2a1bfd8948f713ccae814b3038528d6a4e76e9d9d101857692a4d8da5de6f`,
then installs its sole Debian package. The package displays ST's license and
requires an explicit acceptance; declining stops installation. The helper
never installs or invokes Docker.

If the helper stops on any dependency error, rerun the same command after
updating to the corrected repository state. The operation reuses its pinned
source checkout. Do not continue to `make firmware` while dependency
preparation is incomplete.

ARM/aarch64 Linux support is CLI-only, as documented in ST's
[STM32CubeProgrammer FAQ](https://dev.st.com/stm32cube-docs/prog/2.23.0/en/docs/markup/CubeProg_FAQ.html).
After the helper finishes, verify the source-built ROS package, repository
Debian package, and CLI:

```sh
cd /home/zames/Mentor_Pi
./tools/install_onboard_microros_setup.sh --verify
test "$(dpkg-query -W -f='${Version} ${Architecture}' stm32cubeprogrammer)" = \
  "2.23.0 arm64"
test -x /usr/bin/STM32_Programmer_CLI
source /opt/ros/humble/setup.zsh
source /opt/mentor_pi/micro_ros_setup-3.1.3/install/local_setup.zsh
test "$(ros2 pkg prefix micro_ros_setup)" = \
  /opt/mentor_pi/micro_ros_setup-3.1.3/install/micro_ros_setup
```

The installer must report `Verified source-built micro_ros_setup 3.1.3`.
Only after every command above succeeds should you prepare the pinned inputs
below and continue to Tutorial 02, where the first command is `make firmware`.

## 4. Prepare pinned native inputs

```sh
cd /home/zames/Mentor_Pi
make doctor
make setup
```

`make setup` downloads and SHA-256-verifies the arm64 Arm GNU 13.2.1 toolchain
under the ignored firmware dependency directory and fetches the pinned
standalone Git dependencies. Expected result: `Setup complete` with native
Humble mode and no Docker requirement.

Next: [Onboard Computer Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md).
