# ADR-0002: Docker-Everywhere Host Build and Runtime

Status: Accepted  
Date: 2026-08-10  
Decision owners: RRCLite v2 maintainers

## Context

Maintaining a separate native Ubuntu 22.04/RDK X5 build path exposed package
repository skew, source-built micro-ROS setup dependencies, and host PATH
sanitization failures. The normal-computer Docker path already provides the
reviewed Humble environment. The RDK X5 runs arm64 containers directly on the
Linux kernel, so it does not require QEMU or a virtual machine. Its limiting
resource is memory pressure, not instruction emulation. Docker documents this
[shared-kernel container model](https://docs.docker.com/get-started/docker-concepts/the-basics/what-is-a-container/),
and the RDK X5 documentation accounts for approximately 1.15 GB of reserved
[onboard memory](https://d-robotics.github.io/rdk_doc/en/Advanced_development/linux_development/driver_development_x5/memory/)
before normal Linux workloads.

## Decision

Firmware cross-compilation, micro-ROS library generation, the compiled Agent,
ROS host packages, lightweight GCC sanitizer checks, interactive development,
handoff, and production runtime shall use one pinned, content-identified,
target-architecture Ubuntu 22.04/ROS 2 Humble project image per architecture.
The image is based on the matching pinned
`micro_ros_static_library_builder` architecture child. It installs the Arm GNU
13.2.1 toolchain once and verifies its target-specific checksum.
Normal computers additionally use one pinned, architecture-native Ubuntu 24.04
Clang 18 quality image for formatting, tidy, coverage, and fuzzing. The RDK X5
never pulls that image. Existing host ROS installations are not installed,
removed, or sourced.

The project image consumes the signed Humble snapshot dated 2026-08-07 and a
strict `amd64`/`arm64` package lock. The snapshot source uses only the
checksum-pinned ROS Snapshot Builder key with full fingerprint
`4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA`; it does not inherit the separate
normal ROS repository key. A build fails when its architecture has no entry,
the key fingerprint/checksum differs, or an installed package differs. Its
content identity covers the Dockerfile, zsh configuration, ROS lock,
dependency locks, and base-image digest. `make setup` pulls each unique base
once; it does not delete obsolete images already present in Docker's local
store.

The host retains only operations that require direct host integration:

- installing the measured udev rule and serial-access group; and
- flashing through STM32CubeProgrammer, including the checked arm64 package.

The RDK X5 is detected from device-tree identity. Its setup omits the quality
and fuzzing image. Ordinary native builds use a budget bounded by CPU count,
four jobs, and one job per 2 GiB of available memory. The dedicated QEMU
handoff instead uses up to eight package workers while constraining each nested
package build to one; both policies accept a validated explicit override.

Ordinary development commands, including `make host`, remain native to the
current computer. The standard way to produce RDK artifacts on an amd64
development computer is the dedicated `make rdk-handoff` target. That target
uses QEMU/binfmt to build the arm64 project image, firmware inputs, micro-ROS
library, Agent, ROS host prefix, and arm64 OCI handoff; selecting the dedicated
target is sufficient authorization and no additional emulation flag is
required. It must use the arm64 base digest and package lock throughout and
must fail rather than silently fall back to amd64 content. Its handoff records
the amd64 build host, arm64 target, emulated execution, and absence of native
target validation.

The handoff records atomic input and output checkpoints for eight stages. A
retry resumes the release ID, reuses verified stage output and incremental
colcon state, and reruns unfinished or failed tests. Changed inputs refresh the
private source context and invalidate only stages that depend on those inputs;
unaffected firmware, Agent, host-build, or packaging stages retain their
checksummed evidence. Failed work is retained; an explicitly fresh state is
removed only after strict generated-path and metadata validation, and
successful disposable state is removed only after the final bundle checksum
succeeds.

The RDK loads and installs this handoff without rebuilding the host workspace.
Native compilation on the RDK is optional and reserved for diagnosis, not a
release prerequisite. Native RDK image-load, runtime, memory,
tracker-deadline, peripheral, HIL, and release evidence remain mandatory. This
separation offloads expensive compilation without presenting QEMU execution as
RDK execution.

Production uses one systemd-managed Docker service. The container runs the
fail-coupled compiled Agent and configuration-supervisor launch with host
networking, only the reviewed serial device, a read-only root filesystem,
dropped capabilities, and temporary writable home/log paths. Systemd owns
restart and stop behavior. A production handoff carries the exact runtime image
as a checksummed OCI archive plus matching host and Agent artifacts.

`make shell` provides zsh with pinned Oh My Zsh, completion, ROS 2 completion,
autosuggestions, and syntax highlighting. Automation and non-interactive launch
entrypoints remain Bash.

This decision supersedes only ADR-0001's native-Agent and native-Humble host
deployment rules. It does not alter the CH9102F/USART1 transport, serial
settings, ROS interfaces, firmware safety behavior, or HIL evidence boundary.

## Consequences

One project image replaces the former firmware-builder, micro-ROS-builder, and
host-runtime image roles and makes the exact Humble environment transferable.
The RDK has one arm64 project image; a normal computer has its native project
image plus the quality image and builds/exports the arm64 project image when
`make rdk-handoff` is requested. Ephemeral build containers and the single
persistent production container reuse those image layers. Image storage and
Docker Engine remain host requirements. The RDK normally spends resources on
runtime and lightweight release gates rather than host compilation. Emulated
builds and mocks may validate build and packaging behavior, but image loading,
serial, flash, tracker timing, performance, and hardware claims still require
native RDK X5 evidence.
