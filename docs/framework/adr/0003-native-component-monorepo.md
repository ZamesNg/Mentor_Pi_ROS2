# ADR-0003: Native Component Monorepo

Status: Accepted  
Date: 2026-08-11  
Supersedes: [ADR-0002](0002-docker-everywhere-host-runtime.md)

## Context

Firmware, the micro-ROS Agent, and ROS applications have different build,
deployment, and runtime lifecycles. Coupling them through one production image
made ordinary component work and onboard operation depend on container and
handoff machinery. Git submodules would move interface compatibility into
cross-repository coordination without removing those distinct build graphs.

## Decision

Keep one Git history and three root-level components:

```text
firmware/         CMake/Ninja and firmware/Makefile
micro_ros_agent/  CMake + colcon and micro_ros_agent/Makefile
ros2_ws/          rosdep/vcs/colcon and ros2_ws/Makefile
```

`ros2_ws/src/mentor_pi_interfaces` is the canonical editable interface source.
Firmware consumes a checked compressed Humble micro-ROS SDK containing the
generated interfaces, `motor_profile_contract.hpp`, and `libmicroros.a`.
Manifest hashes bind the SDK to the interfaces, upstream sources, and Arm GNU
13.2.Rel1 toolchain. Interface changes regenerate and commit the SDK atomically.

Ubuntu 22.04 amd64/arm64 is the only native build/test and production platform.
macOS and other Linux distributions build/test all components through the VS
Code Dev Container. The Dev Container is development-only and is the sole
remaining Docker definition. Firmware flashing stays on the physical host.

Onboard, the Agent is installed below a versioned `/opt/mentor_pi/agent/`
prefix and runs as a hardened non-root boot service. It owns the stable
`/dev/mentor_pi_mcu` serial device and restarts on loss. It never starts ROS
applications. Applications start manually; the supervisor and adapters retain
their fail-closed authorization behavior, while the optional hardware-
independent tracker fails closed on stale map-frame pose and local control
faults.

The root Makefile contains only integration, passive-check,
characterization/evidence, and qualification actions. Production Docker,
runtime containers, OCI/QEMU, and container handoffs are removed.

## Consequences

- `colcon build` in `ros2_ws` builds only ROS packages.
- Firmware and Agent builds no longer rebuild unrelated components.
- Native onboard service and recovery behavior is directly observable through
  systemd and journald.
- macOS and unsupported Linux distributions require VS Code/Docker for build
  and test, but are not claimed as native targets.
- Checked SDK binary changes require explicit hash review and preserve the GPL
  source/provenance obligations of the pinned Agent/tracker dependencies.
- Existing production-container evidence is historical and must be re-recorded
  for the native topology before release claims.
