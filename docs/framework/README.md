# RRCLite v2 framework

These documents define the normative Mentor Pi firmware, micro-ROS transport,
Agent, ROS application, safety, and verification contract. The implementation
is a one-history monorepo with three independent native build graphs.

Operators should begin with the ordered [host](../tutorials/host/) or
[onboard](../tutorials/onboard/) tutorial, not with this directory.

## Precedence

When documents conflict, accepted non-superseded ADRs take precedence,
followed by requirements and safety, interfaces, architecture/hardware, and
verification guidance. Historical evidence never overrides the active
contract.

| Document | Purpose |
| --- | --- |
| [ADR-0001](adr/0001-mcu-ros-transport.md) | USB-C/CH9102F/USART1 and micro-ROS transport decision. |
| [ADR-0003](adr/0003-native-component-monorepo.md) | Native component monorepo and development-only Dev Container. |
| [Requirements](requirements.md) | Stable mandatory requirement IDs. |
| [Architecture](architecture.md) | Runtime ownership, tasks, component and service topology. |
| [ROS interface contract](ros-interface-contract.md) | Public names, types, QoS, units, validation, and limits. |
| [Hardware baseline](hardware-baseline.md) | Pins, peripherals, polarities, and electrical facts. |
| [Reliability and safety](reliability-and-safety.md) | Safe states, watchdogs, fault and recovery behavior. |
| [Development standards](development-standards.md) | Language, build, review, and repository rules. |
| [Verification](verification.md) | Requirement-to-test mapping and evidence acceptance. |

[ADR-0002](adr/0002-docker-everywhere-host-runtime.md) is retained only as a
superseded historical decision.

Hardware or release claims require recorded HIL/instrument results. Software
tests, Dev Container builds, mocks, and visual observations cannot substitute
for an unobserved physical metric.
