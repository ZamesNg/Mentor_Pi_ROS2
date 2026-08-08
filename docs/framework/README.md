# RRCLite v2 Framework Documentation

This directory contains the detailed design and acceptance specifications for
RRCLite v2: STM32F407VET6 firmware using FreeRTOS, STM32 HAL, and a pinned ROS 2
Humble micro-ROS stack, with an Ubuntu 22.04 Humble host on `amd64` or `arm64`.
Ubuntu 22.04 development runs Humble natively; every other Ubuntu release uses
the pinned Humble Docker runtime and no native ROS installation.

For hands-on work, start with
[Tutorial 01](../tutorials/01-prepare-ubuntu-development-host.md) and follow the
numbered `Next` links. For changing project status, use
[Next steps](../NEXT_STEPS.md). This README indexes the normative contracts;
tutorials do not override them.

## Normative specifications

| Document | Responsibility |
| --- | --- |
| [Requirements](requirements.md) | Functional, platform, performance, safety, and quality requirements. |
| [Hardware baseline](hardware-baseline.md) | Board resources, verified signal ownership, memory classes, and peripheral limits. |
| [Verified board profile](verified-hardware-profile.md) | Compiled wheel placement/signs, IMU axes, retained defaults, and MCU status indication. |
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

## Operator and qualification path

The ordered [tutorial series](../tutorials/01-prepare-ubuntu-development-host.md)
contains environment setup, locked flashing, host deployment, passive bring-up,
SWD characterization, guarded commissioning, functional HIL, and long
qualification campaigns. Exact ROS commands appear at the stage where they are
safe to run.

Manual Markdown checklists and ledgers are not used. Diagnostic collectors and
qualification tools create immutable logs, checksums, JSON/CSV metrics, JUnit,
and session records. A hardware or release claim remains incomplete whenever a
required physical metric is absent or `NOT_OBSERVED`.

Raw legacy material under `docs/reference/` is intentionally ignored and never
a build input. The tracked [legacy audit](legacy-audit.md) is the maintained
traceability record derived from that evidence.
