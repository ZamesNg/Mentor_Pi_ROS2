# RRCLite v2 Framework Specification

Status: design baseline for review  
Target board: Ros Robot Controller Lite V1.0  
Target host: ROS 2 Jazzy on Ubuntu 24.04 (`amd64` or `arm64`)  
Target MCU: STM32F407VET6 with FreeRTOS and STM32 HAL

## Purpose

This directory is the normative design specification for the RRCLite v2
rewrite. It defines the behavior that must be agreed before application code is
written. The rewrite replaces the legacy Python serial bridge and proprietary
packet dispatcher with a micro-ROS client on the MCU and the native micro-ROS
Agent on the host.

The design has four primary goals:

1. Preserve every verified, active hardware-control function of the Lite board.
2. Remain responsive and recoverable under sustained high-rate ROS traffic.
3. Make memory use, queues, deadlines, validation, and failure behavior bounded
   and observable.
4. Keep project-owned host runtime and the MCU framework in C++17, conform all
   handwritten C++ to the Google C++ Style Guide, and confine permitted MCU C11
   to documented module/library boundaries.

## Locked system boundary

The supported physical path is:

```text
ROS 2 graph
    <-> native micro-ROS Agent
    <-> Linux USB serial device
    <-> USB-C / CH9102F USB-to-UART bridge
    <-> STM32 USART1, 1,000,000 baud, 8N1, no flow control
    <-> micro-ROS client and hardware workers
```

The host cable carries USB data, but the MCU-side connection after the CH9102F
is USART1. There is no usable native-MCU USB connector on this board revision.
The second USB-C connector is a 5 V/5 A power output and is not a communication
port. Native USB, PCB rework, and reuse of the power connector are out of scope.

## Functional scope

The v2 controller supports:

- four encoder-motor channels, with closed-loop motion locked in normal images
  until motor HIL qualification;
- four PWM servos;
- up to sixteen half-duplex bus servos;
- three indicator LEDs, one buzzer, and two RGB pixels;
- two buttons;
- the QMI8658 IMU;
- battery voltage monitoring and low-voltage threshold;
- the 128 x 32 OLED.

Gamepad, SBUS, Bluetooth, USB host, LCD/LVGL, and chassis-level kinematics are
explicitly excluded. The old source tree may still contain files for excluded
features; their presence does not make them part of v2.

## Document map

| Document | Normative responsibility |
| --- | --- |
| [Requirements](requirements.md) | Identified functional, performance, safety, platform, and quality requirements. |
| [Hardware baseline](hardware-baseline.md) | Board resources, confirmed signal ownership, memory classes, and peripheral limits. |
| [Legacy audit](legacy-audit.md) | Traceability from each legacy behavior and known defect to a v2 disposition. |
| [Transport ADR](adr/0001-mcu-ros-transport.md) | Accepted micro-ROS serial transport decision and rejected alternatives. |
| [Architecture](architecture.md) | Host/MCU components, task ownership, data flow, resource budgets, and session lifecycle. |
| [ROS interface contract](ros-interface-contract.md) | Exact v2 topics, services, message schemas, QoS, limits, units, and validation. |
| [Reliability and safety](reliability-and-safety.md) | Safe states, command leases, overload policy, watchdog, and fault response. |
| [Verification](verification.md) | Traceable tests and objective acceptance thresholds. |
| [Qualification evidence ledger](qualification-evidence-ledger.md) | Blank release-candidate identity, campaign, evidence, result, and approval template. |
| [Development standards](development-standards.md) | Language, style, build, static-memory, review, and CI rules. |
| [Implementation roadmap](implementation-roadmap.md) | Ordered delivery stages and entry/exit gates. |

Recommended reading order is the table order. Implementers must also read every
document linked from the subsystem they change.

Supporting operator material lives outside this normative framework. Use the
[ROS 2 CLI examples](../ros2-cli-examples.md) for schema-correct commands and
[board-arrival bring-up evidence record](../board-arrival-bringup-checklist.md)
for the first physical session. The
[CI and hardware qualification gates](../ci-and-hardware-gates.md) define the
hosted/software-only versus physical-HIL boundary and the later release
campaign.

## Normative language

The terms **shall**, **must**, and **required** express mandatory behavior.
**Should** expresses a preferred behavior that needs a documented engineering
reason to waive. **May** expresses an allowed option. Informative descriptions
must not override a shall-level requirement.

Requirement identifiers use the form `<AREA>-NNN`, for example `TRN-001`.
Verification identifiers use `VER-<AREA>-NNN`. Every shall-level product
requirement must be covered by one or more verification identifiers before
release.

## Source-of-truth order

When documents appear to conflict, use this order and correct the lower-priority
document in the same change:

1. accepted architecture decision records;
2. `requirements.md` and `reliability-and-safety.md`;
3. `ros-interface-contract.md`;
4. `architecture.md` and `hardware-baseline.md`;
5. `verification.md` and `implementation-roadmap.md`;
6. `legacy-audit.md`, which describes evidence rather than desired behavior;
7. the legacy firmware and host implementation under `../reference/`.

For physical wiring and silicon resources, the board schematic, STM32
datasheet/reference manual, and verified bench measurements override legacy
application code. Any unresolved conflict affecting safe output is a stop-work
condition.

## Design principles

- High-rate state is represented by a fixed latest-value mailbox, not a growing
  FIFO.
- Interrupt handlers only capture state and notify tasks; they do not parse ROS,
  allocate memory, wait for hardware, or chain application work.
- Exactly one task owns each peripheral and exactly one task owns all micro-ROS
  entities and transport state.
- Incoming messages are validated completely before they can affect hardware.
- Normal firmware accepts motor zero/stop commands but rejects nonzero motor
  targets. A separately acknowledged, raised-wheel commissioning image is
  limited to 0.25 RPS and 300 permille; all motor modes still require fresh,
  valid per-motor commands and fail to zero. Servos hold position on an
  ordinary communication loss.
- All queues, strings, arrays, middleware pools, and service transactions are
  bounded.
- No steady-state dynamic allocation is permitted on the MCU.
- Drops, overwrites, timeouts, resets, high-water marks, and transport failures
  are measurable through diagnostics.
- Recovery never replays a command from an earlier Agent session.

## Implementation gates

| Gate | Required evidence | Result on failure |
| --- | --- | --- |
| D0: specification accepted | All documents present, internally linked, reviewed, and without open safety/interface decisions. | Correct the documents; do not write runtime code. |
| D1: micro-ROS feasibility | Jazzy static library and all v2 types fit resource budgets; representative 500 Hz traffic and twenty reconnects pass. | Stop and reopen the transport ADR. Do not silently introduce another protocol. |
| D2: platform foundation | Static FreeRTOS tasks, transport, clock, safe GPIO initialization, allocation seal, and watchdog pass unit/HIL checks. | Keep actuators disabled. |
| D3: hardware parity | Every retained device passes its driver tests and its legacy-audit row is closed. Motor evidence begins with passive encoder direction checks and guarded commissioning HIL. | Do not expose the affected ROS endpoint; keep nonzero motor actuation locked while motor polarity or PID evidence is incomplete. |
| D4: interface complete | Every documented topic/service and validation rule passes graph and contract tests. | Do not begin system stress qualification. |
| D5: qualification | All mandatory tests in `verification.md`, including stress, fault injection, reconnect, and soak, pass. | No release artifact may be produced. |
| D6: release | Reproducible builds, firmware checksum, configuration, diagnostics report, and rollback procedure are archived. | Deployment remains blocked. |

Passing an earlier gate does not waive a later acceptance criterion.

## Change control

Changes to topic or service schemas, hardware scope, safe-state behavior,
transport, supported platform, fixed capacities, or acceptance thresholds require
all of the following in one review:

- the affected requirements and interface text;
- an ADR addition or superseding ADR when a locked decision changes;
- updated legacy traceability where applicable;
- updated verification coverage;
- an explicit compatibility statement.

The v2 API has no compatibility obligation to `ros_robot_controller_msgs`, but
after the v2 interface package is released, incompatible changes require a new
major interface version.

## External technical references

- [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)
- [micro-ROS STM32CubeMX utilities, Jazzy branch](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/jazzy)
- [micro-ROS Agent, Jazzy branch](https://github.com/micro-ROS/micro-ROS-Agent/tree/jazzy)
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)

These upstream projects are dependencies, not substitutes for the requirements
and qualification defined here.
