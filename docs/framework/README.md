# RRCLite v2 Framework Documentation

This directory contains the detailed design and acceptance specifications for
RRCLite v2: STM32F407VET6 firmware using FreeRTOS, STM32 HAL, and a pinned
ROS 2 Humble micro-ROS stack, with an Ubuntu 22.04 Humble host on amd64 or
arm64. Ubuntu 24.04 development uses Docker and no native ROS installation.

For the project overview and supported Make commands, start with the
[root README](../../README.md). For the current handoff state, remaining work,
and the next commands to run, use [Next steps](../NEXT_STEPS.md). This README is
an index; the linked specifications retain their full detail and authority.

## Normative specifications

| Document | Responsibility |
| --- | --- |
| [Requirements](requirements.md) | Functional, platform, performance, safety, and quality requirements. |
| [Hardware baseline](hardware-baseline.md) | Board resources, verified signal ownership, memory classes, and peripheral limits. |
| [Legacy audit](legacy-audit.md) | Traceability from retained legacy behavior and known defects to v2 dispositions. |
| [Transport ADR](adr/0001-mcu-ros-transport.md) | Accepted CH9102F/USART1 micro-ROS transport and rejected alternatives. |
| [Architecture](architecture.md) | Host/MCU components, task ownership, data flow, resource budgets, and session lifecycle. |
| [ROS interface contract](ros-interface-contract.md) | Exact topics, services, schemas, QoS, units, limits, and validation rules. |
| [Reliability and safety](reliability-and-safety.md) | Safe states, command leases, overload policy, watchdog, and fault response. |
| [Verification](verification.md) | Traceable tests and objective acceptance thresholds. |
| [Development standards](development-standards.md) | Language, Google C++ style, embedded restrictions, review, and CI rules. |
| [Implementation roadmap](implementation-roadmap.md) | Ordered implementation stages and entry/exit gates. |

Accepted ADRs take precedence over other design text. Requirements and safety
rules then take precedence over the interface contract, followed by architecture
and hardware detail, verification, and legacy evidence. A conflict affecting
physical wiring or safe output is a stop-work condition until resolved.

## Qualification and operational material

| Document | Use |
| --- | --- |
| [Qualification evidence ledger](qualification-evidence-ledger.md) | Release-candidate identity, campaign results, evidence, and approval record. |
| [Next steps](../NEXT_STEPS.md) | Current status, unresolved gates, and continuation handoff. |
| [Flashing and first bring-up](../flashing-and-first-bringup.md) | CubeProgrammer flashing and initial safety procedure. |
| [Host preparation and handoff](../host-preparation-and-handoff.md) | Humble host build, packaging, installation, and deployment handoff. |
| [Board-arrival checklist](../board-arrival-bringup-checklist.md) | First-board evidence record and guarded commissioning sequence. |
| [CI and hardware gates](../ci-and-hardware-gates.md) | Software-only CI boundary and required physical qualification. |
| [ROS 2 CLI examples](../ros2-cli-examples.md) | Schema-correct diagnostic and bounded command examples. |

Raw legacy material under `docs/reference/` is intentionally ignored and is
never a build input. The tracked [legacy audit](legacy-audit.md) is the maintained
traceability record derived from that evidence.
