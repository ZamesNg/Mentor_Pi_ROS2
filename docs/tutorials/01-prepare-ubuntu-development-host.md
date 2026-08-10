# Tutorial 01: Prepare the Docker Host

Start here on either the RDK X5 onboard computer or a normal development
computer. Ubuntu 22.04 and 24.04 both use architecture-matched, pinned ROS 2
Humble Docker images. Do not install or source ROS on the host.

**Run on:** RDK X5 Ubuntu 22.04 `arm64`, or a normal Ubuntu 24.04 `amd64`/`arm64` computer
**Hardware state:** board and actuators disconnected

There is no previous tutorial.

## 1. Open the repository

```sh
cd "${HOME}/Mentor_Pi"
```

All later commands use this exact checkout. Do not set `HOST_ARCH`, install ROS
Jazzy, restore PlatformIO, or run `git clean -fdX`.

## 2. Install basic host packages

```sh
sudo apt-get update && sudo apt-get install -y \
  ca-certificates docker.io git make libusb-1.0-0-dev unzip zsh
```

An amd64 computer that will produce the RDK handoff also needs the host-side
emulator and copy tool. Installing these packages does not register or repair
binfmt silently; `make rdk-handoff` performs an arm64 execution probe and stops
if the registration is unusable.

```sh
sudo apt-get install -y binfmt-support qemu-user-static rsync
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
when intentionally overriding it. The variable is a general per-command build
override; it is not an RDK runtime setting.

```sh
make setup
```

Expected result: `Setup complete`. The command pulls each native-architecture
base once, builds the pinned local derivatives once, and verifies their
identity. On the RDK this is one Humble base and one project image. On a normal
computer it is the same project image plus the separate Noble/Clang 18 quality
image. Firmware, micro-ROS, Agent, host, shell, runtime, and handoff reuse the
project image; Docker may retain shared parent layers and obsolete images.
Stop on any OS, architecture, Docker, programmer, checksum, network, or
dependency error. Generated dependencies and build outputs remain ignored.

## 5. Build and transfer the production RDK bundle

This is the production primary flow when an amd64 computer prepares the RDK.
After setup, run `make rdk-handoff`. On a host with at least eight online CPUs,
the handoff uses eight package-level QEMU workers; smaller hosts use their
available CPU count. Each package's CMake/Ninja build stays at one worker to
avoid nested `8 x 8` oversubscription. The command prints the selected count,
performs an actual `linux/arm64` QEMU/binfmt execution preflight (and never
registers binfmt), then creates a UTC-named
`build/rdk-handoff/rdk-arm64-*/` bundle with unchanged, checksummed
`host-handoff/` and `board-handoff/` subbundles.

An interrupted command preserves its ignored work and original UTC release ID.
Running `make rdk-handoff` again automatically resumes the compatible build,
revalidates completed stage checksums, and reruns unfinished or failed host
tests incrementally. When source, dependency, runner, image, or worker inputs
change, the same release refreshes its private source context, preserves every
completed stage whose own inputs and output checksum still match, and
invalidates only affected stages. For example, a tutorial or packaging-runner
edit does not rebuild micro-ROS, firmware, the Agent, or the host binaries; a
host source edit preserves the earlier firmware and Agent stages and resumes
the existing colcon build incrementally. Use
`RDK_HANDOFF_FRESH=1 make rdk-handoff` only to deliberately discard the entire
active generated checkpoint. If the amd64 host becomes memory-constrained,
`RRCLITE_BUILD_JOBS=1 make rdk-handoff` is an explicit override for this QEMU
command only; it does not configure compilation or runtime on the RDK.

```sh
cd "${HOME}/Mentor_Pi"
make rdk-handoff
# PLACEHOLDER: replace YYYYMMDDTHHMMSSZ with the timestamp printed above.
readonly RDK_BUNDLE="${HOME}/Mentor_Pi/build/rdk-handoff/rdk-arm64-YYYYMMDDTHHMMSSZ"
(cd "${RDK_BUNDLE}" && sha256sum --check SHA256SUMS)
grep -E '^(build_execution|build_host_architecture|target_architecture|native_target_validated)=' \
  "${RDK_BUNDLE}/RDK-HANDOFF.txt"
```

The values must be `qemu-emulated`, `amd64`, `arm64`, and `0`. This is not
native RDK runtime, memory, tracker, peripheral, HIL, or release evidence.
Replace the timestamp above with the exact path printed by the Make command.
Archive, checksum, and transfer the newest completed bundle with one command:

```sh
make rdk-transfer RDK_LOGIN=sunrise@rdk-hostname
```

The command verifies the newest bundle under `build/rdk-handoff/`, preserves
modes and symlinks in a tar archive, writes and verifies its SHA-256 sidecar,
creates the remote directory, and transfers both files. To select an older
completed bundle or a different remote directory, pass
`RDK_HANDOFF=/absolute/path/to/rdk-arm64-<actual-timestamp>` or
`RDK_DEST=/absolute/remote/path`. Configure non-default ports or jump hosts in
SSH config so the compact command remains unchanged.

A trusted removable drive may replace `scp`. On the RDK, verify the computer
and archive before extracting anything:

```sh
cd "${HOME}/Mentor_Pi"
test "$(uname -m)" = aarch64
test "$(. /etc/os-release && printf '%s/%s' "${ID}" "${VERSION_ID}")" = ubuntu/22.04
./tools/detect_host_profile.sh | grep -Fqx 'profile=rdk-x5'
docker info >/dev/null
cd build/received-handoffs
# PLACEHOLDER: replace YYYYMMDDTHHMMSSZ with the transferred bundle timestamp.
sha256sum --check rdk-arm64-YYYYMMDDTHHMMSSZ.tar.sha256
tar -xpf rdk-arm64-YYYYMMDDTHHMMSSZ.tar
readonly RDK_BUNDLE="${HOME}/Mentor_Pi/build/received-handoffs/rdk-arm64-YYYYMMDDTHHMMSSZ"
(cd "${RDK_BUNDLE}" && sha256sum --check SHA256SUMS)
```

Stop if the native architecture, OS, device-tree profile, Docker engine, or
either checksum differs. Do not run `make host` on the RDK deployment path.

Next: [Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md).
