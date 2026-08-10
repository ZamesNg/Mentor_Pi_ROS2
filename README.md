# Mentor Pi RRCLite v2

RRCLite v2 is a C++17 ROS 2 Humble controller stack for the Mentor Pi RRCLite
V1.0 board (STM32F407VET6). It replaces the legacy Python serial bridge and
proprietary MCU packet dispatcher with micro-ROS and a compiled micro-ROS Agent.

```text
ROS 2 nodes <-> containerized micro-ROS Agent
            <-> USB-C / CH9102F / USART1 at 1,000,000 baud, 8N1
            <-> FreeRTOS firmware <-> robot hardware
```

The data connector is USB on the host side and USART1 after the CH9102F bridge.
The second USB-C connector is power-only; the MCU has no supported native-USB
transport.

## Start here

Follow the single tutorial sequence in numerical order on either the RDK X5
onboard computer or a normal Ubuntu development computer. One content-addressed
project image per native architecture supplies ROS 2 Humble, the Agent,
firmware tools, builds, lightweight tests, and runtime on both. Only udev
serial configuration and STM32CubeProgrammer flashing remain host operations.
`make shell` opens a zsh runtime with pinned Oh My Zsh, completion,
autosuggestions, and syntax highlighting.

| Step | Tutorial | Main command |
| ---: | --- | --- |
| 01 | [Prepare the Docker host](docs/tutorials/01-prepare-ubuntu-development-host.md) | `make setup` |
| 02 | [Build and flash PID](docs/tutorials/02-build-and-flash-default-pid-firmware.md) | `make firmware`, `make flash` |
| 03 | [Build and run Humble](docs/tutorials/03-build-and-run-humble-host.md) | `make host`, `make start` |
| 04 | [Run passive board bring-up](docs/tutorials/04-run-passive-board-bringup.md) | `make passive-check` |
| 05 | [Characterize board hardware](docs/tutorials/05-characterize-board-hardware.md) | `make characterize-board` |
| 06 | [Run ROS 2 CLI checkout](docs/tutorials/06-ros2-cli-hardware-checkout.md) | `make shell` |
| 07 | [Run software and physical gates](docs/tutorials/07-run-stress-soak-and-release-gates.md) | computer-specific release gate |
| 08 | [Run ros2_control](docs/tutorials/08-run-mentor-pi-hardwares.md) | `make host-hardwares` |

Every complex operation is a one-line Make action. The helper prompts for
hardware-specific values and exact safety acknowledgements, so operators do
not copy long ROS command blocks or edit placeholder text.

For RDK production deployment from an amd64 computer, `make rdk-handoff`
QEMU-builds a checksummed arm64 bundle for offline transfer. It defaults to up
to eight package workers and automatically resumes the active incomplete
release after an interruption. An unchanged rerun reuses the newest compatible
completed bundle and removes a redundant incomplete duplicate;
`RDK_HANDOFF_FRESH=1` explicitly forces a new release. Changed inputs rebuild
only affected stages;
unaffected checksummed stages and incremental colcon work are retained. The RDK loads and installs the
bundle without rebuilding the host workspace; native RDK compilation is
optional diagnostics, not release evidence.

The current board has a hardware-verified timing baseline, while the newest
complete default PID candidate is prepared but not yet flashed. Its practical
resume point is Tutorial 02 for that PID default flash, followed by Tutorial
04's passive checks. A new computer or operator starts at Tutorial 01.

## Safety and current status

The default firmware is the normal closed-loop PID image:
`control_mode=CLOSED_LOOP`, ±1000-permille output limit, a 6 RPS ceiling,
independent 198 ms per-motor leases, session-loss disarming, transport-failure
shutdown, and atomic command validation with no lease refresh on invalid
values. The configuration supervisor gate, model-specific RPS limits, and
supervisor startup inhibition are all preserved.

The PID artifact remains explicitly unqualified (`release_qualified=0`) until
the numbered HIL sequence records the required physical evidence. Software
gates verify its implementation bounds and provenance; they do not promote
its physical qualification status.

Never flash or run with actuators connected until the passive bring-up steps
in Tutorial 04--05 are complete. Before powered motor work, complete passive
encoder-direction checks, raise or equivalently guard every wheel, use a
current-limited supply, and keep a physical motor-power stop reachable. The
tutorials repeat the required warning immediately before every
hardware-sensitive step.

## Supported environments

- Production/onboard computer: RDK X5 arm64 on Ubuntu 22.04 with Docker Engine.
- Development computer: supported Ubuntu on `amd64` or `arm64` with Docker
  Engine; architecture-native pinned Ubuntu 22.04/Humble images are used.
- Existing host ROS installations are left untouched and never sourced.
- Firmware, micro-ROS, Agent, host, and runtime builds are Docker-only.
- The RDK X5 stores one project image. Normal computers additionally store one
  optional Ubuntu 24.04/Clang 18 quality image for formatting, tidy, coverage,
  and fuzzing. Ephemeral build containers do not create additional image roles.
- The RDK X5 release gate excludes fuzzing, coverage, and the full Clang suite;
  `make release-software-gates` retains those normal-computer checks.
- UART flashing: STM32CubeProgrammer CLI through CH9102F/USART1.
- Project-owned data-plane runtime: C++17; Python is limited to ROS launch and
  build orchestration.

Run `make help` for the supported build and flash interface. Generated build,
dependency, micro-ROS, log, and qualification outputs are disposable or
machine-generated and are not committed.

## Detailed contracts

The tutorials are the operator path. Exact public ROS names, QoS, units,
limits, task ownership, memory budgets, safety behavior, and acceptance cases
remain authoritative under [docs/framework/](docs/framework/README.md).
Accepted ADRs take precedence, followed by requirements/safety, interfaces,
architecture/hardware, and verification.

Hardware or release claims require machine-generated HIL/instrument results.
A successful software test, mock, campaign exit status, or visual observation
does not substitute for an unobserved physical metric.

## Repository map

- [`firmware/mentor_pi_mcu/`](firmware/mentor_pi_mcu/) — STM32 firmware,
  drivers, controller workers, and micro-ROS runtime.
- [`mentor_pi_ros2/`](mentor_pi_ros2/) — directly buildable ROS 2 Humble
  workspace containing the firmware interfaces, tracking interfaces, bringup,
  mecanum/Ackermann `mentor_pi_hardwares` adapters, and opt-in GPL ALTO MPC
  trackers. The trackers run onboard; trajectory planning and transforms remain
  offboard.
- [`tools/`](tools/) — pinned setup, build, verification, packaging, flash, and
  developer serial-access helpers.
- [`thirdpart/`](thirdpart/) — the checked repository copy of the licensed
  STM32CubeProgrammer 2.23.0 arm64 Debian-package archive used by the onboard
  CubeProgrammer installer.
- [`docs/tutorials/`](docs/tutorials/) — one complete Docker-first 01--08
  operator workflow for both computer types.
- [`docs/framework/`](docs/framework/) — normative design and verification
  contracts.

For current project status and open work, read
[docs/NEXT_STEPS.md](docs/NEXT_STEPS.md). Codex and contributors must also read
[AGENTS.md](AGENTS.md) before changing the repository.
