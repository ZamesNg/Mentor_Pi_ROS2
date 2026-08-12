# RRCLite v2 implementation roadmap

Status: implementation architecture current as of 2026-08-11. Physical and
release evidence remains open as listed in `docs/NEXT_STEPS.md`.

## Delivered source structure

| Component | Authority | Public entry point | Runtime ownership |
| --- | --- | --- | --- |
| Firmware | CMake/Ninja | `make -C firmware ...` | STM32/FreeRTOS |
| micro-ROS Agent | CMake + colcon | `make -C micro_ros_agent ...` | non-root systemd service |
| ROS applications | rosdep/vcs/colcon | `make -C ros2_ws ...` | manual ROS launch |

The firmware, Agent, and five ROS packages share one Git history but not a
build graph. `mentor_pi_interfaces` is canonical under `ros2_ws/src`; the
firmware's checked SDK and manifest provide the explicit compatibility bridge.

The Agent owns the stable CH9102F serial link and restarts independently.
`controller.launch.py` owns only the configuration supervisor. Vehicle launch
keeps the supervisor, adapters, controllers, and optional tracker in one
fail-closed application graph. The Agent never starts an application.

## Delivered safety behavior

- one normal closed-loop ADRC firmware classification;
- atomic motor validation and independent 198 ms leases;
- model-specific RPS limits, 6 RPS ceiling, and ±1000-permille clamp;
- session-loss and transport-failure disarming;
- generation/session authorization accepted only from the single expected
  supervisor publisher;
- stale-feedback and invalid-command zero output in hardware/tracker paths;
- bounded passive diagnostics and production evidence below
  `/var/log/mentor-pi/actions`.

## Build and operator delivery

Ubuntu 22.04 amd64/arm64 builds/tests all components natively. macOS and other
Linux distributions use the VS Code Dev Container for all build/test work.
The Dev Container is never an onboard runtime or release-evidence source.

Two complete tutorial sequences are delivered:

- [`docs/tutorials/host/`](../tutorials/host/)
- [`docs/tutorials/onboard/`](../tutorials/onboard/)

Only the onboard sequence belongs in a production handoff.

## Remaining evidence gates

1. Native firmware and Agent build/test evidence on amd64 and arm64.
2. Dev Container build/test smoke on a non-native development host.
3. Agent installation, boot hardening, USB reconnect, and restart evidence.
4. Manual application startup with Agent available/unavailable/restarted and
   proof that old sessions and stale feedback stay disarmed.
5. Firmware memory/stack/resource evidence and tracker 30 Hz/25 ms benchmark.
6. Passive board characterization, powered guarded qualification, 500 Hz load,
   three 100-cycle recovery campaigns, and 24-hour soak.

No roadmap item is release-complete until its required machine-generated
evidence is recorded and reviewed. Mocks and software-only tests do not close
physical gates.
