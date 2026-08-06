# Host Preparation and Handoff

This is the supported host path for RRCLite v2. The production runtime is
Ubuntu 22.04 with ROS 2 Humble on `amd64` or `arm64`. Ubuntu 24.04 is a clean
Docker development host and shall not have ROS installed natively.

The workflow produces a separately versioned, checksummed ROS 2 host handoff;
it does not modify the firmware handoff under `build/board-handoff/`.

## Supported environments

The onboard computer shall run Ubuntu 22.04 with ROS 2 Humble installed at
`/opt/ros/humble`. Install Humble from the official
[ROS 2 Humble Ubuntu documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html),
then verify the exact platform and build tools:

```sh
./tools/verify_host_build_environment.sh
```

The verifier resolves Ubuntu's normal `/etc/os-release` symlink, then requires
its canonical file to identify Ubuntu 22.04. It rejects a dangling link,
non-regular target, unsupported architecture, missing Humble setup, wrong
`ROS_DISTRO`, or missing build tools.

On Ubuntu 24.04, do not install Humble or source another ROS distribution.
Install Git, Make, and Docker Engine, then run:

```sh
make doctor
make setup
make host
```

The root Makefile selects the checked-in, content-addressed Ubuntu 22.04/Humble
image. The container is a build/test environment only: it receives no systemd,
`/opt` installation, serial device, or hardware access. The resulting software
is deployed to the native Ubuntu 22.04/Humble onboard computer.

## Native Ubuntu 22.04 build

For a native clean-machine build, install project dependencies in a separate
privileged step while the controller target is inactive:

```sh
sudo ./tools/prepare_host_build_dependencies.sh
```

That command verifies Ubuntu 22.04 and Humble before its first mutation,
installs build tools, initializes or updates `rosdep`, and resolves the two
project packages. It does not install the Mentor Pi runtime. Build and package
afterward as an unprivileged user:

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
builds one merged `Release` prefix, and runs both ROS package suites. It copies
the prefix below a temporary root, moves the staging prefix aside, and proves
that the copied interfaces, executables, and dynamic libraries work without a
reference to the staging path. A trap restores the original prefix on success
or failure.

## Pinned Humble container build on Ubuntu 24.04

The recommended development workflow is:

```sh
make setup
make host
```

For a separately named handoff, use the container wrapper directly:

```sh
./tools/build_host_handoff_container.sh \
  --architecture arm64 \
  --release-id pre-hardware-20260806-r1 \
  --output-directory build/host-handoff/pre-hardware-20260806-r1
```

Use `amd64` only for an amd64 deployment host. The wrapper refuses a mutable
image tag, builds with networking disabled after dependency preparation, and
runs as the invoking UID. `HOST-BUILD-METADATA.txt` and `HOST-HANDOFF.txt`
record the exact image digest, target architecture, `ros_distro=humble`,
`ubuntu=22.04`, build type, compiler, and project-owned source fingerprint.
A handoff whose metadata names another ROS distribution or Ubuntu release is
not installable.

## Verify and install the handoff

Never mix an arm64 handoff with an amd64 target. From the reviewed handoff
directory:

```sh
sha256sum --check SHA256SUMS
sed -n '1,200p' HOST-HANDOFF.txt
sed -n '1,200p' AGENT-METADATA.txt
```

The handoff contains:

- `host/`: the tested, relocatable merged ROS prefix and deployment assets;
- `agent-installer/`: the pinned Humble source-build Agent installer, state
  validator, runtime wrapper, and authoritative source lock;
- `HOST-HANDOFF.txt`, `AGENT-METADATA.txt`, and a full `SHA256SUMS` manifest;
- this guide and the first-board checklist used for the handoff.

The Agent build is deliberately separate and networked: it installs exact
detached Humble-compatible upstream commits recorded in the handoff lock,
verifies their origins and clean state, and installs the native executable
below `/opt/mentor_pi`. On the onboard Ubuntu 22.04 host, with the production
target inactive:

Before deployment, `make agent HOST_ARCH=amd64|arm64` builds those same pinned
source commits without package installation inside the digest-pinned Humble
micro-ROS builder. The resulting checksummed record binds the resolved
architecture-specific image, installed package manifest, full generated
install tree, source lock, and native executable hash. It is build
compatibility evidence, not a deployable Agent bundle; production installation
continues to use the guarded source installer below.

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

## Qualification boundary and future migration

The container build proves package generation, tests, layout, and relocation.
It cannot prove live `systemd-analyze verify`, CH9102F identity, serial
ownership, Agent/MCU discovery, reconnect behavior, or hardware safety. Record
those results in the
[board-arrival checklist](board-arrival-bringup-checklist.md).

ROS 2 Jazzy is future migration work, not an active build or runtime. Plan a
controlled Ubuntu 24.04/Jazzy migration and requalification before Humble
reaches end of life in May 2027. Do not mix Humble and Jazzy nodes, Agents, or
MCU artifacts in one deployment.
