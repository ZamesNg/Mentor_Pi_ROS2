# Host Preparation and Handoff

This is the pre-hardware host path for RRCLite v2. It produces a separately
versioned, checksummed ROS 2 host handoff; it does not modify the firmware
handoff under `build/board-handoff/`.

## Supported host

Deployment is limited to Ubuntu 24.04 `amd64` or `arm64` with ROS 2 Jazzy
installed at `/opt/ros/jazzy`. Install Jazzy from the official
[ROS 2 Jazzy installation documentation](https://docs.ros.org/en/jazzy/Installation.html),
then verify the exact platform and build tools:

```sh
./tools/verify_host_build_environment.sh
```

The verifier resolves Ubuntu's normal `/etc/os-release` symlink, then requires
its canonical file to identify Ubuntu 24.04. It rejects a dangling link,
non-regular target, unsupported architecture, missing Jazzy setup, wrong
`ROS_DISTRO`, or missing build tool.

For a native clean-machine build, install project build dependencies in a
separate privileged step while the controller target is inactive:

```sh
sudo ./tools/prepare_host_build_dependencies.sh
```

That command verifies the OS and Jazzy before its first mutation, installs the
host build tools, initializes or updates `rosdep`, and resolves the two project
packages. It does not install the Mentor Pi runtime. Build and package afterward
as an unprivileged user:

```sh
readonly RELEASE_ID=pre-hardware-20260806-r1
readonly WORK_DIRECTORY="${PWD}/build/host-native-work/${RELEASE_ID}"
readonly HOST_PREFIX="${PWD}/build/host-native-prefix/${RELEASE_ID}"
readonly HOST_HANDOFF="${PWD}/build/host-handoff/${RELEASE_ID}"

./tools/build_host_release.sh \
  --output-prefix "${HOST_PREFIX}" \
  --work-directory "${WORK_DIRECTORY}"
./tools/package_host_handoff.sh \
  --host-prefix "${HOST_PREFIX}" \
  --output-directory "${HOST_HANDOFF}" \
  --release-id "${RELEASE_ID}"
```

The build starts from nonexistent work/install paths, performs `rosdep check`,
builds one merged `Release` prefix, and runs both ROS package suites. It promotes
a copy below a temporary root, renames the original staging prefix out of the
way, and proves the copied environment, interfaces, executables, and dynamic
libraries work without referring to the staging path. A trap restores the
original prefix on both success and failure.

## Pinned offline container build

On a development machine with Docker, first fetch the reviewed
multi-architecture Jazzy image explicitly:

```sh
readonly HOST_IMAGE='ros:jazzy-ros-base@sha256:da725acf8b0f9f30c683e33ffbdcd6482d077af96d6fdc7688c5f4f280b7d923'
docker pull --platform linux/arm64 "${HOST_IMAGE}"
```

Use `linux/amd64` only for an amd64 deployment host. After the image exists
locally, the project build runs with networking disabled, as the invoking UID,
and without systemd, `/opt` installation, serial devices, or hardware access:

```sh
./tools/build_host_handoff_container.sh \
  --architecture arm64 \
  --release-id pre-hardware-20260806-r1 \
  --output-directory build/host-handoff/pre-hardware-20260806-r1
```

The wrapper refuses a mutable image tag. `HOST-BUILD-METADATA.txt` and
`HOST-HANDOFF.txt` record the exact digest, target architecture, ROS
distribution, build type, compiler, and project-owned source fingerprint.

## Verify and install the handoff

Never mix an arm64 handoff with an amd64 VM. From the reviewed handoff directory:

```sh
sha256sum --check SHA256SUMS
sed -n '1,200p' HOST-HANDOFF.txt
sed -n '1,200p' AGENT-METADATA.txt
```

The handoff contains:

- `host/`: the tested, relocatable merged ROS prefix and deployment assets;
- `agent-installer/`: the pinned source-build Agent installer, state validator,
  runtime wrapper, and authoritative source lock;
- `HOST-HANDOFF.txt`, `AGENT-METADATA.txt`, and a full `SHA256SUMS` manifest;
- this guide and the first-board checklist used for the handoff.

The Agent build is deliberately separate and networked: it installs exact
detached upstream commits recorded in the handoff lock, verifies their origins
and clean state, and installs the native executable below `/opt/mentor_pi`.
On the Ubuntu deployment VM, with the production target inactive:

```sh
sudo ./agent-installer/tools/install_microros_agent.sh
sudo ./host/lib/mentor_pi_bringup/promote_host_release \
  --staged-prefix "${PWD}/host" \
  --release-id pre-hardware-20260806-r1
```

Connect exactly one target CH9102F, identify its current `/dev/ttyUSB*` node and
unique `ID_SERIAL_SHORT` or physical `ID_PATH`, and only then install the site
identity and coordinated services using the exact commands in the installed
[`mentor_pi_bringup` guide](../src/mentor_pi_bringup/README.md#production-installation).
Keep `mentor-pi-controller.target` stopped until the udev identity, YAML, ROS
domain, unit verification, and motor-locked firmware digest are reviewed.

## What still requires the real VM and board

The container build proves package generation, tests, layout, and relocation.
It cannot prove live `systemd-analyze verify`, USB passthrough, CH9102F identity,
serial ownership, Agent/MCU discovery, reconnect behavior, or hardware safety.
Record those results in the
[board-arrival checklist](board-arrival-bringup-checklist.md).
