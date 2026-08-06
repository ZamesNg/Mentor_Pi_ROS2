# Mentor Pi RRCLite v2

RRCLite v2 is a C++17 ROS 2 Humble controller stack for the Mentor Pi
RRCLite V1.0 board (STM32F407VET6). It replaces the legacy Python serial bridge
and proprietary MCU packet dispatcher with micro-ROS and a native micro-ROS
Agent.

```text
ROS 2 nodes <-> native micro-ROS Agent
            <-> USB-C / CH9102F / USART1 at 1,000,000 baud, 8N1
            <-> FreeRTOS firmware <-> robot hardware
```

The data connector is USB on the host side and USART1 after the CH9102F bridge.
The second USB-C connector is power-only; this board has no supported native-MCU
USB connection.

## Safety and current status

The software is ready for first-board bring-up, but motor/encoder polarity, PID
tuning, IMU axes, analog scaling, peripheral timing, watchdog timing, and the
endurance/reconnect gates still need hardware evidence.

The default firmware is motor-locked: zero/stop motor commands work, but every
nonzero target is rejected. Never flash or commission with actuators connected.
Before any powered motor test, complete the passive encoder-direction checks,
raise or guard every wheel, and use a current-limited supply. Follow
[Flashing and first bring-up](docs/flashing-and-first-bringup.md) and record the
session in the [board-arrival checklist](docs/board-arrival-bringup-checklist.md).

## Supported environments

- Production/onboard computer: Ubuntu 22.04 with ROS 2 Humble.
- Development computer: Ubuntu 24.04 with Docker; do not install ROS natively.
- Host architectures: `arm64` and `amd64`; build for the deployment computer.
- Firmware build: CMake/Ninja through the root Makefile.
- Host build: `colcon` inside the pinned Humble container.
- Flashing without SWD: STM32CubeProgrammer through CH9102F/USART1.

Project-owned runtime code does not use Python. Upstream ROS and code-generation
tools may use Python during builds.

## Quick start

Install Git, Make, and Docker Engine, then start Docker. CubeProgrammer is only
required when flashing. On the Ubuntu 24.04 development computer, use:

```sh
make doctor
make setup HOST_ARCH=arm64
make firmware
make agent HOST_ARCH=arm64
make test HOST_ARCH=arm64
```

`make test` includes the authoritative Humble host build and tests. To build
only a deployable host handoff, run:

```sh
make host HOST_ARCH=arm64
```

Use `HOST_ARCH=amd64` only for an amd64 deployment target. Run `make help` for
all supported targets. Generated outputs are disposable and ignored by Git:

- firmware: `firmware/mentor_pi_mcu/build/stm32/`;
- host handoff: `build/host-handoff/`;
- Agent and test evidence: other directories below `build/`.

## Flash the default locked firmware

Install STM32CubeProgrammer, disconnect motor and servo power, connect the
USB-C port labelled UART1/USB serial 1, then hold `BOOT`, tap `RST`, and release
`BOOT`. Identify the exact CH9102F device and run:

```sh
make flash PORT=/dev/serial/by-id/REPLACE_ME \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

The wrapper verifies the locked artifact, snapshots its exact ELF, programs it
through the factory bootloader at 115200 baud/8E1, and requests read-back
verification. Reset normally afterward; the application returns to
1,000,000 baud/8N1.

Commissioning firmware is intentionally separate and capped at 0.25 RPS and
300 permille output. Use it only after the guarded prerequisites in the bring-up
guide:

```sh
make firmware-commissioning COMMISSIONING_BUILD_ACK=MOTORS_RAISED
make flash-commissioning PORT=/dev/serial/by-id/REPLACE_ME \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  COMMISSIONING_FLASH_ACK=MOTORS_RAISED_CURRENT_LIMITED
```

## Deploy and operate the host

`make host` creates a checksummed Humble handoff for the selected architecture.
Install it on the Ubuntu 22.04 onboard computer by following
[Host preparation and handoff](docs/host-preparation-and-handoff.md). Use
[ROS 2 CLI examples](docs/ros2-cli-examples.md) for bounded telemetry, command,
and service examples.

## Move to another computer

There is currently no required Git remote. Commit all intended tracked changes,
then either copy the whole repository including its hidden `.git` directory, or
create a portable Git bundle:

```sh
git status --short
git bundle create ../Mentor_Pi.bundle --all
```

On the new computer:

```sh
git clone Mentor_Pi.bundle Mentor_Pi
cd Mentor_Pi
make doctor
make setup HOST_ARCH=arm64
```

Do not transfer generated `build/`, firmware `third_party/`, or generated
micro-ROS directories; rebuild them. The raw contents of `docs/reference/` are
ignored and are not included in a Git bundle, so copy that local legacy
evidence separately if it is still needed.

Codex should read [AGENTS.md](AGENTS.md), then use
[docs/NEXT_STEPS.md](docs/NEXT_STEPS.md) as the durable handoff.

## Repository map

- [`firmware/mentor_pi_mcu/`](firmware/mentor_pi_mcu/) — STM32 firmware, drivers,
  controller workers, and micro-ROS runtime.
- [`src/mentor_pi_interfaces/`](src/mentor_pi_interfaces/) — bounded ROS messages
  and services.
- [`src/mentor_pi_bringup/`](src/mentor_pi_bringup/) — C++ supervisor, Agent
  launch, deployment tools, and qualification utilities.
- [`tools/`](tools/) — pinned setup, build, verification, packaging, and flash
  helpers used by the Makefile.
- [`docs/framework/`](docs/framework/) — normative requirements, hardware,
  architecture, interface, safety, and verification contracts.
- [`docs/ci-and-hardware-gates.md`](docs/ci-and-hardware-gates.md) — what software
  tests prove and what still requires the board.

Public ROS names, QoS, units, limits, safety behavior, and acceptance tests are
defined by the framework documents. Do not infer them from this quick-start
guide.
