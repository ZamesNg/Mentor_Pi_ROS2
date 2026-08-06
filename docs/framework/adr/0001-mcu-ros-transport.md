# ADR-0001: MCU ROS Transport

Status: Accepted  
Date: 2026-08-06  
Decision owners: RRCLite v2 maintainers

## Context

RRCLite V1.0 uses an STM32F407VET6. Its communication USB-C connector routes
USB D+ and D- to a CH9102F USB-to-UART bridge. The bridge routes TX and RX to
STM32 USART1 on PA10 and PA9. The existing firmware configures USART1 for
1,000,000 baud, eight data bits, no parity, one stop bit, and no hardware flow
control.

The second USB-C connector is a 5 V/5 A power-delivery output. Its signaling is
owned by the power controller and it has no MCU data path. STM32 USB FS pins
PA11 and PA12 are used by PWM servo channels 1 and 2. USB-capable PB14 and PB15
are not routed to a USB connector. Consequently, this PCB has no available
native-MCU USB port that preserves the existing hardware functions.

The communication connector also supports firmware programming without an
SWD probe: RRCLite V1.0 provides BOOT and RST buttons, and the STM32F407 factory
system-memory bootloader accepts CubeProgrammer traffic over the same
CH9102F/USART1 PA9/PA10 path. That maintenance operation uses 115200 baud, 8E1,
then ends at reset; it is separate from and does not alter the production
micro-ROS transport selected by this ADR.

The legacy architecture translates ROS messages in a Python node to a custom
serial packet protocol. Under burst traffic, transport parsing, unbounded or
unchecked work, blocking peripheral access, and unsafe DMA/queue behavior can
interfere with one another. The rewrite needs direct ROS 2 semantics on the MCU,
bounded resource use, and deterministic recovery.

## Decision

RRCLite v2 shall use:

- the ROS 2 Humble line of micro-ROS;
- the default Micro XRCE-DDS middleware;
- the micro-ROS custom serial transport over STM32 USART1;
- USART1 RX DMA in continuous circular mode;
- the board's existing CH9102F and communication USB-C connector;
- the native micro-ROS Agent as a ROS 2 process on Ubuntu 22.04;
- a clean v2 ROS interface generated from `mentor_pi_interfaces`.

The transport settings are fixed at 1,000,000 baud, 8N1, with no RTS/CTS. The
Linux device must be selected through a stable udev symlink, nominally
`/dev/mentor_pi_mcu`, rather than a changing `ttyUSB*` or `ttyACM*` name. A
deployment with multiple identical adapters must match a unique USB serial
number or a documented physical USB path.

The production Agent shall run natively, not through Docker or Snap. Upstream
can expose serial devices to a privileged container, but this project requires
direct udev ownership by a dedicated `mentor-pi-serial` group, a closed systemd
device policy, and host-visible logs without a container isolation layer. The Agent
and `micro_ros_msgs` sources shall be checked out at the immutable revisions in
`tools/install_microros_agent.sh`, then installed below `/opt/mentor_pi`. The
stable wrapper sources only the upstream ROS environments and immediately
replaces itself with the compiled native binary:

```sh
/opt/mentor_pi/bin/mentor_pi_micro_ros_agent serial \
  --dev /dev/mentor_pi_mcu --baudrate 1000000 -v4
```

`ros2 run` may be used for interactive diagnosis but is not the production
launcher. The exact verbosity may be reduced after qualification.
Production service management must run the Agent as an unprivileged user in
the dedicated serial group, require a unique measured adapter identity, hold an
exclusive wrapper lock, restart it after failure, and preserve its logs and exit
status.

Ubuntu 24.04 is a development host only. It shall have no native ROS
installation. ROS-dependent host builds and micro-ROS generation run inside
pinned Ubuntu 22.04/ROS 2 Humble containers; ROS-free cross-compilation,
analysis, and portable tests may use pinned Ubuntu 24.04 utility containers. A
deployment shall not mix ROS distributions between the MCU client, Agent, and
host nodes. Migration to ROS 2 Jazzy is future work that requires new pinned
artifacts and full requalification before Humble reaches end of life in May
2027; it is not an active fallback under this ADR.

## Why this option

The selected path uses the only data-capable USB connector already present on
the product. The external host-to-board segment remains USB; only the short,
controlled on-board segment is asynchronous UART. This avoids PCB changes and
keeps all four PWM servo channels.

micro-ROS supplies ROS 2 nodes, topics, services, types, and QoS semantics on an
RTOS-class MCU. Its Humble STM32 integration supports FreeRTOS and a custom
USART transport with DMA, and specifically directs circular DMA for RX. The
Agent natively supports serial transport and represents the MCU entities in the
host DDS graph.

This choice removes the first-party Python packet bridge and avoids maintaining
a second public wire protocol. It does not, by itself, guarantee reliability;
the ownership, bounded-memory, safe-state, and qualification requirements in
the rest of this specification remain mandatory.

## Transport implementation constraints

- `MicroRosTask` is the sole caller of `rcl`, `rclc`, `rmw`, transport read/write,
  entity creation/destruction, and ROS publish/response operations.
- RX uses one 8 KiB DMA-accessible circular buffer. DMA half-transfer,
  transfer-complete, UART IDLE, and UART error interrupts only update status and
  notify `MicroRosTask`.
- RX DMA is not aborted and restarted from an interrupt or normal receive
  callback. Producer/consumer distance greater than the buffer capacity is a
  detectable transport overrun and forces safe teardown.
- TX custom-write callbacks use a fixed DMA-safe bounce buffer and a bounded
  completion wait. The upstream stream-framing layer may flush one framed XRCE
  message through multiple callbacks; every callback length is checked. A
  timeout is a session fault, and application code must never force-unlock a
  HAL handle.
- A low-level TX timeout means that TX-DMA completion or UART TC completion did
  not arrive by the length-derived custom-write deadline, or that HAL reported
  a TX error. Missing XRCE reliable ACKs or Agent/session replies are not
  `transport_tx_timeouts` when DMA and TC completed normally. Reliable
  publish/response operations instead use their 10 ms XRCE session timeout;
  an ACK timeout is fatal to that session. With no reliable operation pending,
  Agent loss is detected by three consecutive bounded ACTIVE ping failures.
- The initial XRCE MTU is 512 bytes with reliable stream history depth eight.
  The pinned Humble generated type support is normative and shall report a
  388-byte maximum message-field payload for `ControllerDiagnostics`; the
  pinned RMW/CDR path shall serialize exactly 392 bytes including its four-byte
  encapsulation. The feasibility gate must prove that
  generated CDR plus XRCE headers for every maximum v2 sample fits the MTU
  without endpoint-level fragmentation; conflicting handwritten arithmetic
  does not override the generator. Serial framing, CRC, and byte stuffing occur
  after this MTU and are accounted as wire bytes, not as unused CDR budget.
- Middleware calls remain incremental and bounded: executor spin waits at most
  1 ms; transport reads, ACTIVE ping/time sync, reliable publish/response, and
  each remote finalizer call at most 10 ms; creation-time sync at most 20 ms;
  each entity-creation call at most 40 ms and all creation at most 2 s; and all
  remote destruction at most 500 ms. `MicroRosTask` advances its heartbeat
  between lifecycle calls so no heartbeat interval exceeds 100 ms.
- No UDP, TCP, Ethernet, discovery-server, or USB CDC transport profile is built
  into the release firmware.
- Total measured serialized traffic, summing RX and TX in every complete
  one-second window, must remain below 70 kB/s at the supported 500 Hz stress
  point. This reserves at least 30% of the nominal 100 kB/s payload capacity of
  a 1 Mbps 8N1 link.
- A transport/session failure disarms and zeros drive motors before ROS entity
  teardown. Reconnection creates a new session and cannot restore old commands.

Detailed mechanics are defined in [Architecture](../architecture.md) and
[Reliability and safety](../reliability-and-safety.md).

## Mandatory feasibility gate

This decision authorizes a feasibility implementation, not product deployment.
Before device-driver or application implementation proceeds, Gate D1 shall
prove all of the following on the actual STM32F407 toolchain and board:

1. The Humble micro-ROS static library builds with every interface in
   `mentor_pi_interfaces`; generated maximum CDR/XRCE sizes, including the
   normative 388-byte diagnostics field payload and 392-byte encapsulated
   sample, fit the 512-byte MTU, and every
   stream-framing custom-write callback fits the 1 KiB TX bounce buffer.
2. Linked firmware retains at least 20% flash headroom and each RAM class retains
   at least 20% headroom, with DMA buffers placed in DMA-accessible SRAM.
3. The micro-ROS task retains at least 25% stack headroom under its representative
   worst case and starts with no less than the upstream-recommended 10 KiB.
4. No allocation occurs while an entity generation is sealed and active.
   Entity recreation may reset and use only the bounded arena before resealing;
   twenty complete Agent disconnect/reconnect cycles have no net memory loss.
5. A representative four-motor command at 500 Hz, concurrent telemetry, and a
   worst-case bounded service response remain below 70 kB/s and produce no RX
   overrun, deadlock, watchdog reset, or stale-command replay.
6. The Agent and all MCU ROS entities recover within five seconds after each
   reconnect, while motors satisfy the 200 ms command-lease rule.
7. Fault injection separately proves: a missing reliable ACK returns through
   the 10 ms session bound; three ping-only failures detect an absent Agent
   when reliable operations are acknowledged; and missing TX-DMA/TC completion
   uses the physical write deadline and `transport_tx_timeouts` diagnostic.
   Every middleware/lifecycle bound above is met and task heartbeat age remains
   no greater than 100 ms.

If any item fails, implementation stops and this ADR is reopened. The team must
not silently replace micro-ROS, relax safety behavior, remove an interface, or
add a proprietary bridge.

## Alternatives considered

### Native STM32 USB CDC

Rejected for RRCLite V1.0. No native-USB data connector is wired to the MCU.
PA11/PA12 are required for PWM servo outputs; PB14/PB15 are not connector-routed.
Using CDC would require PCB redesign or unsupported rework, ESD and VBUS design,
signal-integrity validation, and likely function loss. Native USB may be
reconsidered only for a future board revision in a new ADR.

### Power-output USB-C connector

Rejected. It is a source-side 5 V/5 A power-delivery port, not an MCU data port.
Connecting or modifying it as though it were a USB device port is unsupported
and may damage attached hardware.

### Custom framed UART protocol with a C++ ROS bridge

Not selected. It could be made bounded and avoids embedding ROS entities, but it
would retain two interfaces, duplicate serialization and compatibility logic,
and require long-term maintenance of a proprietary host bridge. It is not an
automatic fallback; a future decision would need a complete replacement ADR.

### embeddedRTPS

Rejected. The STM32 integration describes it as experimental, and its practical
design is aligned with UDP/Ethernet. RRCLite exposes neither an Ethernet PHY nor
a supported Ethernet connector.

### Legacy Python bridge

Rejected. It retains the custom packet stack and blocking/correlation defects,
and violates the requirement that project-owned production nodes and the
control/data path be C++17. This does not prohibit pinned upstream ROS tools,
CLI/launch infrastructure, build/code-generation tools, or dependencies from
using Python internally; they are not a first-party bridge or control/data
component.

## Consequences

Positive consequences:

- the existing board and cable remain usable without sacrificing hardware;
- MCU endpoints appear directly in the ROS 2 graph;
- message definitions replace hand-maintained packet enums and struct layouts;
- latest-value QoS and bounded middleware pools can prevent stale command
  backlogs;
- the project-owned host control/data path no longer depends on Python serial
  code.

Costs and risks:

- micro-ROS has a nontrivial flash, RAM, and task-stack footprint;
- the Agent is an additional host process and a session dependency;
- serial bandwidth and XRCE framing must be measured, not assumed;
- services that access the bus-servo UART need asynchronous worker completion;
- upstream explicitly does not claim production readiness, so the full local
  verification program is part of the decision.

## References

- [micro-ROS STM32CubeMX utilities, Humble](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble)
- [micro-ROS Agent, Humble](https://github.com/micro-ROS/micro-ROS-Agent/tree/humble)
- [micro-ROS setup and production-use notice](https://github.com/micro-ROS/micro_ros_setup)
- [Micro XRCE-DDS stream transport and framing](https://micro-xrce-dds.docs.eprosima.com/en/stable/transport.html)
- [ROS 2 Humble Ubuntu binaries](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Hardware baseline](../hardware-baseline.md)
- [Verification](../verification.md)
