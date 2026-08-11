# Mentor Pi RRCLite v2

RRCLite v2 is a C++17 ROS 2 Humble controller stack for the Mentor Pi RRCLite
V1.0 board (STM32F407VET6). The repository is one Git history with three
independently built components:

```text
ROS 2 applications (ros2_ws, colcon)
       <-> micro-ROS Agent (micro_ros_agent, CMake + colcon)
       <-> USB-C / CH9102F / USART1 at 1,000,000 baud, 8N1
       <-> FreeRTOS firmware (firmware, CMake + Ninja)
```

There are no Git submodules. `mentor_pi_interfaces` remains the canonical
editable ROS interface source. Firmware consumes a checked, compressed Humble
micro-ROS SDK whose manifest is bound to those interfaces; it never depends on
a built ROS workspace.

## Build environments

- Ubuntu 22.04 amd64/arm64: all components build and test natively. This is the
  only onboard/runtime/service/HIL/production platform.
- macOS and every other Linux distribution: build and test all components in
  the repository's VS Code Dev Container.
- Firmware flashing: run STM32CubeProgrammer on the physical host, using
  `/dev/tty*` on Linux or `/dev/cu.*` on macOS.

The Dev Container is development-only. It cannot be used onboard, manage
systemd, own the production serial transport, flash hardware, or generate
release/HIL evidence. There are no production Docker images or Docker runtime.
Its VS Code terminal uses pinned Oh My Zsh, command completion,
autosuggestions, and syntax highlighting. Post-create setup seeds common
component build/test commands into an idempotent Zsh history block; hardware,
service-installation, and HIL commands are intentionally excluded.

## Component commands

```sh
# Firmware only
make -C firmware setup
make -C firmware test
make -C firmware build
make -C firmware verify
make -C firmware package

# micro-ROS Agent only
make -C micro_ros_agent setup
make -C micro_ros_agent build
make -C micro_ros_agent test

# ROS applications only
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test

# Standard colcon workflow discovers only ros2_ws/src
cd ros2_ws
colcon build
colcon test
colcon test-result --verbose
```

Onboard, install the Agent as a versioned, non-root boot service:

```sh
sudo DEVICE=/dev/ttyUSB0 ID_SERIAL_SHORT=YOUR_SERIAL \
  make -C micro_ros_agent install-service
systemctl status mentor-pi-agent.service
```

ROS applications always start manually. The Agent service never starts them:

```sh
make -C ros2_ws run \
  VEHICLE=mecanum \
  RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED
```

The root Makefile contains integration, passive-check, characterization, and
qualification commands only. Run `make help` for that interface.

## Tutorials

Choose one complete 01–08 track and follow it in order:

- [Host track](docs/tutorials/host/01-prerequisites-and-safety.md): native
  Ubuntu 22.04 or VS Code Dev Container development, host flashing, and
  offboard ROS work.
- [Onboard track](docs/tutorials/onboard/01-prerequisites-and-safety.md): native
  Ubuntu 22.04 firmware, Agent service, ROS applications, recovery, and
  production evidence.

Production handoffs include only the onboard track. Each track repeats every
required command and safety gate and can be followed without the other.

## Safety boundary

The firmware has one `NORMAL_CLOSED_LOOP_DEFAULT` PID artifact with
`control_mode=CLOSED_LOOP`, a 6 RPS implementation ceiling, model-specific
lower limits, a ±1000-permille output bound, independent 198 ms motor leases,
atomic validation without lease refresh on invalid commands, session-loss
disarming, and transport-failure shutdown.

The configuration supervisor publishes a generation/session authorization.
Hardware adapters and trackers require the single expected supervisor
publisher and a matching live heartbeat session. Missing or invalid
configuration, stale feedback, supervisor loss, Agent restart, or serial loss
leaves motion disarmed.

Before powered motor work, complete Tutorials 01–05 passively, verify encoder
direction, raise or equivalently guard every wheel, use a current-limited
supply, and keep a physical stop reachable. A successful build, mock, passive
capture, or software campaign does not qualify powered motion or a release;
those claims require recorded and reviewed HIL/instrument evidence.

## Repository map

- [`firmware/`](firmware/) — standalone firmware Makefile, checked SDK,
  CMake/Ninja build, verification, packaging, and host flash tools.
- [`micro_ros_agent/`](micro_ros_agent/) — standalone source locks, CH9102F
  patch, native build, versioned installation, udev rule, and systemd service.
- [`ros2_ws/`](ros2_ws/) — five directly buildable ROS 2 packages, pinned
  external sources, and manual application launch.
- [`.devcontainer/`](.devcontainer/) — the only Docker definition; development
  build/test environment for macOS and non-Ubuntu-22.04 Linux.
- [`docs/tutorials/host/`](docs/tutorials/host/) and
  [`docs/tutorials/onboard/`](docs/tutorials/onboard/) — ordered operator paths.
- [`docs/framework/`](docs/framework/) — detailed interface, hardware, safety,
  architecture, and verification contracts.

See [docs/NEXT_STEPS.md](docs/NEXT_STEPS.md) for current evidence and open
hardware gates. Contributors must also read [AGENTS.md](AGENTS.md).
