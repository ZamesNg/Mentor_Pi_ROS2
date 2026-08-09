# Mentor Pi RRCLite v2

RRCLite v2 is a C++17 ROS 2 Humble controller stack for the Mentor Pi RRCLite
V1.0 board (STM32F407VET6). It replaces the legacy Python serial bridge and
proprietary MCU packet dispatcher with micro-ROS and a native micro-ROS Agent.

```text
ROS 2 nodes <-> native micro-ROS Agent
            <-> USB-C / CH9102F / USART1 at 1,000,000 baud, 8N1
            <-> FreeRTOS firmware <-> robot hardware
```

The data connector is USB on the host side and USART1 after the CH9102F bridge.
The second USB-C connector is power-only; the MCU has no supported native-USB
transport.

## Start here

Choose one computer track and follow its tutorials in numerical order. Do not
mix environments or jump directly to powered motor motion.

### Onboard computer: RDK X5, Ubuntu 22.04 arm64, native Humble

This path uses no Docker. It builds firmware with a checked local Arm GNU
toolchain and uses conventional `rosdep`, `colcon`, and direct `ros2` commands
from the RDK X5 user's existing zsh configuration.

| Step | Tutorial | Main command |
| ---: | --- | --- |
| 01 | [Prepare the RDK X5](docs/tutorials/onboard-computer/01-prepare-ubuntu-development-host.md) | `make setup` |
| 02 | [Build and flash PID natively](docs/tutorials/onboard-computer/02-build-and-flash-default-pid-firmware.md) | `make firmware` |
| 03 | [Build and run native Humble](docs/tutorials/onboard-computer/03-build-and-run-humble-host.md) | `colcon build`, `ros2 launch` |
| 04 | [Run passive board bring-up](docs/tutorials/onboard-computer/04-run-passive-board-bringup.md) | `make passive-check` |
| 05 | [Characterize board hardware](docs/tutorials/onboard-computer/05-characterize-board-hardware.md) | `make characterize-board` |
| 06 | [Run native ROS 2 CLI checkout](docs/tutorials/onboard-computer/06-ros2-cli-hardware-checkout.md) | `ros2` CLI |
| 07 | [Run native and physical gates](docs/tutorials/onboard-computer/07-run-stress-soak-and-release-gates.md) | `make release-onboard-gates` |
| 08 | [Run ros2_control natively](docs/tutorials/onboard-computer/08-run-mentor-pi-hardwares.md) | `ros2 launch` |

### Normal computer: Ubuntu 24.04, pinned Humble Docker

This path keeps ROS off the native OS and performs the complete software suite,
including Clang 18 fuzzing, in the reviewed containers. `make shell` opens a
zsh runtime with pinned Oh My Zsh, completion, autosuggestions, and syntax
highlighting.

| Step | Tutorial | Main command |
| ---: | --- | --- |
| 01 | [Prepare Ubuntu 24.04](docs/tutorials/normal-computer/01-prepare-ubuntu-development-host.md) | `make setup` |
| 02 | [Build and flash PID in Docker](docs/tutorials/normal-computer/02-build-and-flash-default-pid-firmware.md) | `make firmware` |
| 03 | [Build and run Docker Humble](docs/tutorials/normal-computer/03-build-and-run-humble-host.md) | `make start` |
| 04 | [Run passive board bring-up](docs/tutorials/normal-computer/04-run-passive-board-bringup.md) | `make passive-check` |
| 05 | [Characterize board hardware](docs/tutorials/normal-computer/05-characterize-board-hardware.md) | `make characterize-board` |
| 06 | [Run Docker ROS 2 CLI checkout](docs/tutorials/normal-computer/06-ros2-cli-hardware-checkout.md) | `make shell` |
| 07 | [Run full software and physical gates](docs/tutorials/normal-computer/07-run-stress-soak-and-release-gates.md) | `make release-software-gates` |
| 08 | [Run ros2_control in Docker](docs/tutorials/normal-computer/08-run-mentor-pi-hardwares.md) | `make host-hardwares` |

Every complex operation is a one-line Make action. The helper prompts for
hardware-specific values and exact safety acknowledgements, so operators do
not copy long ROS command blocks or edit placeholder text.

The current board has a hardware-verified timing baseline, while the newest
complete default PID candidate is prepared but not yet flashed. Its practical
resume point is Tutorial 02 for that PID default flash, followed by Tutorial
04's passive checks. A new computer or operator starts at Tutorial 01 of the
track matching that computer.

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

- Production/onboard computer: RDK X5 arm64, Ubuntu 22.04 with native ROS 2
  Humble and Docker-free host, Agent, micro-ROS library, and firmware builds.
- Development computer: any Ubuntu release; Ubuntu 22.04 uses native Humble,
  while every other release uses pinned Ubuntu 22.04/Humble Docker.
- No native ROS installation is supported outside Ubuntu 22.04.
- Host architectures: `arm64` and `amd64`, matched to the deployment computer.
- Firmware build: CMake/Ninja through the root Makefile; native pinned Arm GNU
  13.2.1 on Ubuntu 22.04 and pinned Docker elsewhere.
- Host build: native Humble `colcon` on Ubuntu 22.04, otherwise Docker Humble.
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
  workspace containing `mentor_pi_interfaces`, `mentor_pi_bringup`, and the
  mecanum/Ackermann `mentor_pi_hardwares` ros2_control adapters.
- [`tools/`](tools/) — pinned setup, build, verification, packaging, flash, and
  developer serial-access helpers.
- [`thirdpart/`](thirdpart/) — the checked repository copy of the licensed
  STM32CubeProgrammer 2.23.0 arm64 Debian-package archive used by the onboard
  dependency helper.
- [`docs/tutorials/`](docs/tutorials/) — separate complete 01--08 onboard and
  normal-computer operator workflows.
- [`docs/framework/`](docs/framework/) — normative design and verification
  contracts.

For current project status and open work, read
[docs/NEXT_STEPS.md](docs/NEXT_STEPS.md). Codex and contributors must also read
[AGENTS.md](AGENTS.md) before changing the repository.
