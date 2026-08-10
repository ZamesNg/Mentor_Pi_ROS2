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

Micro-ROS library generation, the compiled Agent, ROS host packages,
interactive development, and production runtime shall use pinned,
content-identified, architecture-native Ubuntu 22.04/ROS 2 Humble Docker images
on both the RDK X5 and normal computers. Firmware cross-compilation and the
normal-computer quality suite use pinned, content-identified,
architecture-native Ubuntu 24.04 compiler images; those non-ROS images produce
MCU artifacts or test the Humble-compatible source tree and never become a
production runtime. Existing host ROS installations are not installed,
removed, or sourced.

The host retains only operations that require direct host integration:

- installing the measured udev rule and serial-access group; and
- flashing through STM32CubeProgrammer, including the checked arm64 package.

The RDK X5 is detected from device-tree identity. Its setup omits the quality
and fuzzing image. A shared build-job budget is bounded by CPU count, four jobs,
and one job per 2 GiB of available memory; a validated explicit override may be
used. Cross-architecture emulation is prohibited.

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

One build/runtime model replaces host-package variations and makes the exact
Humble environment transferable. Image storage and Docker Engine become host
requirements. RDK memory use must be controlled through the shared job budget
and lightweight release gate. Mocks may validate routing and build behavior,
but serial, flash, performance, and hardware claims still require the RDK X5.
