# Mentor Pi Controller v2 Requirements

## 1. Purpose and requirement language

This document defines the requirements for the clean v2 replacement of the
Mentor Pi host controller and STM32 firmware. It is the source of truth for
scope and acceptance. The exact ROS 2 wire contract is defined in
[ros-interface-contract.md](ros-interface-contract.md).

Requirement IDs are stable verification identifiers. They shall not be
renumbered when this document is edited; retired IDs remain reserved.

- **Shall** is mandatory for release acceptance.
- **Should** is a recommended engineering objective. A deviation requires a
  recorded rationale, but does not by itself block acceptance.
- Verification methods are **Review**, **Inspection**, **Analysis**, or
  **Test**, as used by [verification.md](verification.md).

## 2. Product and compatibility requirements

| ID | Requirement | Verification |
|---|---|---|
| SCOPE-001 | The system **shall** be a clean v2 implementation consisting of a micro-ROS client on the onboard MCU and a micro-ROS Agent on the host. | Review |
| SCOPE-002 | The v2 release **shall not** implement the legacy `AA 55` packet protocol, legacy Python serial bridge, legacy ROS names, or legacy `ros_robot_controller_msgs` types. | Inspection |
| SCOPE-003 | Project-owned host runtime and the MCU framework/orchestration **shall** use C++17. Individual project-owned MCU application or driver modules may use C11 or C++17 behind documented boundaries to C-based HAL, FreeRTOS, and micro-ROS libraries. No production ROS node or runtime control script shall use Python; ROS/build-system code-generation tools may use Python internally. | Inspection |
| SCOPE-004 | Handwritten C++ **shall** follow the Google C++ Style Guide subject to the embedded exceptions in [development-standards.md](development-standards.md). | Review |
| SCOPE-005 | v2 **shall** expose only the active hardware listed in HW-001 through HW-010. Gamepad, SBUS, Bluetooth, USB-host, LCD/LVGL, and firmware chassis abstractions are removed and out of scope. | Inspection |
| SCOPE-006 | The design **should** keep application policy, such as chassis kinematics, on the ROS 2 host rather than in the MCU hardware controller. | Review |

## 3. Platform and hardware requirements

| ID | Requirement | Verification |
|---|---|---|
| PLAT-001 | Ubuntu 22.04 amd64/arm64 with ROS 2 Humble **shall** be the only native Agent/ROS build, test, onboard runtime, service-installation, HIL, and production platform. Firmware shall also build natively only on that platform. macOS and every other Linux distribution shall build and test all three components through the repository VS Code Dev Container. The Dev Container shall not be used onboard, for systemd installation, serial runtime, firmware flashing, HIL, or release evidence. | Test |
| PLAT-002 | The MCU software **shall** target STM32F407VET6 using STM32 HAL, FreeRTOS, and the pinned ROS 2 Humble-compatible micro-ROS client stack. | Inspection |
| PLAT-003 | MCU application objects, queues, and buffers **shall** be statically sized. Middleware arena setup is allowed only in bounded `CREATE_ENTITIES` preparation before the allocation seal; `ACTIVE` shall perform no allocation, reallocation, or free, and `TEARDOWN` shall finalize references then reset the complete arena to its pre-create baseline before another create cycle. | Analysis, Test |
| PLAT-004 | The authoritative firmware build **shall** use CMake/Ninja behind `firmware/Makefile`, compile project-owned sources with warnings treated as errors, and emit ELF, HEX/BIN, map, metadata, and size reports. Independent IDE build graphs shall not be supported. | Test |
| PLAT-005 | The firmware build **shall** use the checksummed Arm GNU 13.2.Rel1 toolchain and a checked compressed Humble micro-ROS SDK containing generated interfaces, `motor_profile_contract.hpp`, and `libmicroros.a`. Its manifest shall bind the canonical interface source, upstream source lock, SDK archive/tree, and toolchain; a stale interface/SDK pair shall fail. | Test |
| PLAT-006 | The repository **shall** remain one Git history with independent firmware, Agent, and ROS workspace build graphs and no component submodules. `.devcontainer/` shall be the only Docker definition. Production Docker images, Docker runtime, OCI/QEMU workflows, and container handoffs shall be absent. | Inspection, Test |
| HW-001 | The firmware **shall** control four encoder DC motor channels, M1 through M4. | Test |
| HW-002 | The firmware **shall** control four PWM servo outputs, numbered 1 through 4. | Test |
| HW-003 | The firmware **shall** control and query as many as 16 addressable half-duplex bus servos. | Test |
| HW-004 | The firmware **shall** reserve onboard discrete LED1 for a 1 Hz system heartbeat independent of ROS and expose LED2 and LED3 to validated ROS pattern commands. | Test |
| HW-005 | The firmware **shall** control the single onboard PWM buzzer. | Test |
| HW-006 | The firmware **shall** reserve RGB1 for allocation-free status: red toggles after each successful micro-ROS heartbeat publication, green pulses on TX progress, and blue pulses on RX progress. RGB2 shall accept validated ROS commands. | Test |
| HW-007 | The firmware **shall** update the two host-controlled text lines of the onboard 128 x 32 OLED; the battery line remains controller-owned. | Test |
| HW-008 | The firmware **shall** publish raw QMI8658 accelerometer and gyroscope measurements in SI units. | Test |
| HW-009 | The firmware **shall** publish events from the two onboard buttons, numbered 1 and 2. | Test |
| HW-010 | The firmware **shall** measure board supply voltage and evaluate it against the configured low-voltage threshold. | Test |

Pin assignments, peripheral instances, polarities, and electrical limits are
normative in [hardware-baseline.md](hardware-baseline.md). Any conflict with a
legacy source file shall be resolved in favor of that verified baseline.

### Host configuration supervisor

| ID | Requirement | Verification |
|---|---|---|
| HOST-001 | A project-owned C++17 `rclcpp` node named `/mentor_pi/configuration_supervisor` **shall** validate the exact YAML schema in [architecture.md](architecture.md) for motor model, four per-motor ADRC arrays, four PWM-servo offsets, and battery threshold. Invalid configuration shall keep project-owned host motion disabled. | Inspection, Test |
| HOST-002 | The supervisor **shall** treat first discovery, graph/heartbeat reappearance, an `agent_session_id` change, or a wrap-aware MCU uptime discontinuity as a new session and shall apply motor model, all-motor ADRC configuration, PWM offsets, then battery threshold in that order. Project-owned host motion shall remain disabled until all four return `OK`; ADRC `OK` shall also echo an all-motor applied mask. | Test |
| HOST-003 | Each supervisor configuration call **shall** have a 100 ms host timeout and at most four attempts per host configuration generation, with 100, 200, then 400 ms backoff. Only `BUSY`, result `TIMEOUT`, and client timeout are retryable. Each attempt shall be correlated with the configuration generation and Agent session; a late or stale response shall be ignored. A permanent result, exhausted attempts, session change, or invalid YAML shall leave the motion-enable gate false and report the cause. | Test |
| HOST-004 | The supervisor **shall not** open the serial device, translate ROS interfaces, modify bus-servo persistent configuration, or replay actuator, LED, buzzer, RGB, or OLED commands after a session change. | Inspection, Test |
| HOST-005 | The hardware-independent plugin-based tracker **shall** implement the scheduling, validation, vehicle model, deadline, ADRC/MPC selection, fallback, static actuator bounds, terminal endpoint hold, and zero-output behavior in [tracking-controller.md](tracking-controller.md). The execution horizon shall be the sum of segment durations; thereafter the accepted terminal pose shall remain active with zero reference derivatives until replacement or cancellation. Physical and simulation vehicle launches shall default to the vehicle-matched ADRC plugin; an explicit launch selection may choose MPC or disable the tracker. Both algorithms and both vehicle types shall use the same node and endpoint names. Its pose state shall be `(x_center, y_center, yaw)` copied from a fresh geometry-center `PoseStamped` exactly in `map`, and every accepted trajectory shall also be exactly in `map`. It shall not subscribe to MCU motor state, heartbeat, authorization, Agent, or configuration-supervisor interfaces, and shall not perform high-level planning or arbitrary frame transforms (`VER-TRACK-UNIT-001`, `VER-TRACK-INT-001`, `VER-TRACK-RDK-001`). | Inspection, Test |
| HOST-006 | The mecanum and Ackermann hardware plugins **shall** apply first-order ADRC at the existing 30 Hz hardware loop to all accepted manual or tracker commands. Wheel feedback shall supply translation/longitudinal speed and `/mentor_pi/imu` gyroscope Z shall supply yaw rate; acceleration shall not be integrated. Missing, invalid, or older-than-100-ms required feedback shall command zero/center and fail through the existing authorized hardware error path. Ackermann yaw control shall reset and center steering below 0.1 m/s measured speed. | Inspection, Test |
| HOST-007 | Physical launch **shall** consume `/vrpn_mocap/<robot>/pose` as the already geometry-centered `PoseStamped`, require its frame to be exactly `map`, publish `vehicle/pose`, and own `map -> <robot>/base_footprint` on standard `/tf`. It shall disable controller odometry TF, keep unavoidable controller odometry only on remapped internal topics, and expose no public `vehicle/odometry`, `vehicle/tf_odometry`, or `odom` root. Simulation shall use the same pose/TF outputs while deriving its deterministic plant truth from hidden controller odometry. `rear_axle_footprint`, wheel links, and `imu_link` shall remain in the URDF subtree (`VER-POSE-MAP-001`). | Inspection, Test |
| HOST-008 | Development simulation **shall** provide actuator-limited Ackermann and Mecanum `ros2_control` hardware plugins using the production controller, URDF visuals, pose adapter, namespace, public vehicle endpoints, and the HOST-005 tracker selection contract. It shall default to ADRC, support MPC and tracker-disabled direct control, and use deterministic 30 Hz wall time without `/clock`, Agent, supervisor, firmware endpoints, noise, slip, collision, terrain, or IMU models. Initial pose arguments shall specify the public geometry-center pose in `map` (`VER-SIM-001`). | Inspection, Test |

## 4. Transport and ROS interface requirements

| ID | Requirement | Verification |
|---|---|---|
| TRN-001 | The transport **shall** follow the verified path USB-C connector → CH9102F USB-to-UART bridge → MCU USART1. The host shall use the stable device symlink `/dev/mentor_pi_mcu` and shall not assume a `ttyUSB*` or `ttyACM*` basename. | Inspection, Test |
| TRN-002 | The onboard host **shall** run the pinned micro-ROS Agent as a non-root systemd service using `/dev/mentor_pi_mcu`. The service shall be independent of ROS application launch, restart after serial loss, and shall not start ROS applications. | Inspection, Test |
| TRN-003 | A loss of the Agent or USB transport **shall** be detected, shall disarm the DC motors, and shall enter the reconnect state without rebooting the MCU. | Test |
| TRN-004 | After transport recovery, the MCU **shall** recreate its ROS entities and resume publication without requiring a power cycle. DC motors shall remain stopped until a fresh valid bounded command arrives in the new active session. | Test |
| TRN-005 | A normal Agent loss **shall** leave PWM servos and bus servos holding their last valid target and torque state. | Test |
| TRN-006 | A transport RX overrun, any USART1 framing/noise/overrun/parity error, or transport TX timeout **shall** cause immediate safe transport teardown and motor disarm before reconnection is attempted. | Test |
| ROS-001 | The interface package **shall** be named `mentor_pi_interfaces`, and the MCU node **shall** have fully qualified name `/mentor_pi/controller`. | Inspection, Test |
| ROS-002 | The node **shall** create exactly the seven application publishers, seven application subscriptions, and seven application services defined by [ros-interface-contract.md](ros-interface-contract.md); middleware-internal discovery entities are not part of this count. | Inspection, Test |
| ROS-003 | Every custom array **shall** be fixed-size, and every custom string **shall** be bounded. No custom interface shall contain an unbounded sequence or string. | Inspection |
| ROS-004 | Topic and service names, types, field meanings, units, ranges, validation rules, and QoS **shall** conform exactly to [ros-interface-contract.md](ros-interface-contract.md). | Test |
| ROS-005 | Services **shall** return the common numeric `Result` model; topic validation failures shall be observable through diagnostics. | Test |
| ROS-006 | The controller **shall** use synchronized Agent epoch time for ROS stamps. Until synchronization succeeds, stamps shall be zero and heartbeat shall report time as unsynchronized. | Test |
| ROS-007 | The interface **should** remain directly usable from standard ROS 2 C++ publishers, subscribers, and service clients without a host-side translation node. | Test |

## 5. Functional control requirements

| ID | Requirement | Verification |
|---|---|---|
| CTRL-001 | A motor command **shall** independently update any subset of M1 through M4 and express target output-shaft speed in revolutions per second. | Test |
| CTRL-002 | Each motor channel **shall** have a 200 ms command lease. TIM7 shall release the safety check at 1 kHz, the qualified maximum interval between completed lease evaluations shall be 2 ms including scheduling jitter, and a nonzero target shall expire at an age of 198 ms or greater. This guarantees a zero target no later than 200 ms after acceptance. An invalid command shall not refresh any lease. | Test |
| CTRL-003 | The controller **shall** support JGB520, JGB37, JGA27, and JGB528 motor profiles with the limits and encoder constants in the ROS interface contract. | Test |
| CTRL-004 | A motor-profile change **shall** be rejected while any motor has a nonzero target; a successful change shall reset all motor ADRC state. | Test |
| CTRL-005 | PWM servo commands **shall** support subset updates, pulse widths from 500 through 2500 microseconds, and move durations from 20 through 30000 milliseconds. | Test |
| CTRL-006 | PWM servo offsets **shall** be readable in state and settable from -100 through +100 microseconds. | Test |
| CTRL-007 | A bus-servo move **shall** accept one through 16 unique IDs, one position per ID, and one shared duration. | Test |
| CTRL-008 | Bus-servo state query and configuration **shall** expose ID, position, offset, voltage, temperature, position limits, voltage limits, temperature limit, and torque state. | Test |
| CTRL-009 | When the shared bus-service slot is free, stop requests **shall** have dispatch priority over query/configuration, and any accepted bus service shall have priority over pending move traffic. An accepted service is non-preemptible; a stop received while another bus service owns the slot shall receive `BUSY` and may be retried. Accepting a stop shall interrupt an active move between frames and invalidate both its unsent remainder and every pending move generation accepted before that stop; only a post-stop move command may restart motion traffic. | Test |
| CTRL-010 | LED2/LED3 and buzzer pattern commands **shall** support steady off, steady on, finite repetition, and indefinite repetition. LED1 commands shall be rejected without changing its system heartbeat. | Test |
| CTRL-011 | RGB commands **shall** update RGB pixel 2 using 8-bit red, green, and blue components. Masks selecting firmware-owned RGB pixel 1, including the all-pixels mask, shall be rejected atomically. | Test |
| CTRL-012 | OLED commands **shall** independently replace or clear either host-controlled line and shall reject unsupported characters or oversized strings. | Test |
| CTRL-013 | Button event values **shall** preserve pressed, long-press, long-press-repeat, short/long release, click, double-click, and triple-click distinctions. | Test |
| CTRL-014 | The battery threshold **shall** be runtime configurable from 5000 through 20000 millivolts and shall default to 6300 millivolts after reset. | Test |
| CTRL-015 | Runtime motor model, PWM-servo offsets, and battery threshold changes **shall not** be persisted across reset. Bus-servo ID, position-limit, voltage-limit, and temperature-limit configuration writes are persistent in the addressed servo; an offset is persistent only after `SAVE_OFFSET`; torque is volatile. The MCU shall not store or replay any bus-servo configuration. | Test |

## 6. Real-time, overload, and safety requirements

| ID | Requirement | Verification |
|---|---|---|
| RT-001 | The motor control loop **shall** execute at 100 Hz independently of ROS callbacks, transport state, and bus-servo transactions. | Test |
| RT-002 | A micro-ROS callback **shall not** wait on a peripheral transaction, mutex owned by a slower worker, or an unbounded queue operation. Reliable publish/response operations and incremental ROS entity create/finalize operations shall use the bounded middleware timeouts in [architecture.md](architecture.md), so the micro-ROS task heartbeat interval never exceeds 100 ms. | Inspection, Analysis, Test |
| RT-003 | Motor and PWM subset commands **shall** merge into fixed latest-value state. Bus-servo move traffic shall use a single replace-latest slot. No command topic shall accumulate an unbounded FIFO backlog. | Inspection, Test |
| RT-004 | At 500 motor commands per second, the MCU **shall** remain responsive, continue its 100 Hz motor loop and required publications, honor the 200 ms lease, and recover without reset. Software-only transport tests shall use zero targets; powered alternating targets are permitted only in the guarded HIL procedure from SAFE-007. Intermediate best-effort commands may be coalesced. | Test |
| RT-005 | Topic input above its documented maximum rate **shall** be dropped or coalesced without blocking safety/control work or exhausting memory. | Test |
| RT-006 | Non-bus service work **shall** have a 50 ms local worker deadline. Shared bus-service work shall have a 200 ms MCU deadline, and host bus clients shall use a 250 ms response timeout. | Test |
| RT-007 | Only one bus service request **shall** be active at a time; excess requests shall receive `BUSY`. | Test |
| RT-008 | Each ACTIVE executor iteration **shall** start at most one 10 ms blocking operation across service response, reliable telemetry, Agent ping, and time synchronization. Service, reliable-telemetry, and maintenance opportunities shall rotate without starvation. Each iteration shall publish at most one due best-effort telemetry sample. | Inspection, Analysis, Test |
| SAFE-001 | Message validation **shall** be atomic: if any selected field is invalid, no selected output shall change and no motor lease shall refresh. | Test |
| SAFE-002 | On boot, reset, Agent loss, or fatal transport teardown, all DC motor drive outputs **shall** be disabled or commanded to zero before normal ROS operation proceeds. | Test |
| SAFE-003 | The watchdog and transport safety path **shall** execute without depending on receipt or processing of another ROS message. | Analysis, Test |
| SAFE-004 | Bus-servo configuration **shall** validate the complete request before the first write. If an I/O failure occurs after one or more writes, the response shall report `PARTIAL` and the exact applied-field mask. | Test |
| SAFE-005 | Service success for a write-only bus-servo operation **shall** mean that the command was transmitted successfully; it shall not claim physical application without readback. | Review, Test |
| SAFE-006 | `make -C firmware build`, `verify`, `package`, and `flash` **shall** select only the `NORMAL_CLOSED_LOOP_DEFAULT` ADRC artifact with `control_mode=CLOSED_LOOP`; no alternate motor-authority mode or runtime unlock is supported. The controller shall enforce the active model's RPS limit and the 6 RPS implementation ceiling, clamp output to +/-1000 permille, reject an invalid motor command atomically without refreshing any selected lease, and require the session-bound host configuration gate before project-owned host motion starts. | Inspection, Test |
| SAFE-007 | Before the first powered motor command on a board, checkout **shall** complete a passive, manually driven encoder direction test with motor outputs disabled. Wheels shall then remain raised or equivalently guarded and the board shall use a current-limited supply for every unqualified powered test. Firmware **shall** retain raw signed encoder direction, keep targets, measurements, and LADRC state in that coordinate, and apply one fixed bridge inversion for every channel and motor model. The host **shall** own the only ROS↔MCU chassis sign map. Neither the physical mapping nor any motor ADRC profile is release-qualified until the required per-vehicle motor HIL passes and its evidence is recorded. | Review, Test |

The detailed task ownership, queueing rules, and fault-state transitions are
defined by [architecture.md](architecture.md) and
[reliability-and-safety.md](reliability-and-safety.md).

## 7. Quality and verification requirements

| ID | Requirement | Verification |
|---|---|---|
| QUAL-001 | Every mandatory requirement in this document **shall** map to at least one verification case in [verification.md](verification.md). | Review |
| QUAL-002 | Interface serialization, boundary values, invalid masks, duplicate bus IDs, non-finite motor values, service timeout, Agent loss/recovery, and 500 Hz overload **shall** have automated or hardware-in-loop tests as appropriate. | Review, Test |
| QUAL-003 | The firmware **shall** expose heartbeat and bounded numeric diagnostics sufficient to distinguish validation errors, timeouts, transport faults, executor overruns, and motor watchdog trips. | Test |
| QUAL-004 | The project **shall** document the legacy defects and v2 disposition without copying legacy protocol behavior into the new runtime. | Review |
| QUAL-005 | Public documentation **should** include minimal `ros2 topic pub`, `ros2 topic echo`, and `ros2 service call` examples generated from the final interface definitions. | Review |

## 8. Resource and release acceptance requirements

| ID | Requirement | Verification |
|---|---|---|
| RES-001 | The release firmware image **shall** leave at least 20% of the configured MCU flash region unused, as reported from the linked ELF/map file. | Analysis |
| RES-002 | The release firmware **shall** leave at least 20% headroom in each independently allocated MCU RAM class defined by the linker script and [hardware-baseline.md](hardware-baseline.md), including normal SRAM, CCM RAM, and DMA-accessible RAM where distinct. | Analysis, Test |
| RES-003 | Every FreeRTOS task **shall** retain at least 25% of its configured stack under boundary, 500 Hz stress, reconnect-cycle, and 24-hour soak tests. | Test |
| RES-004 | Under the supported 500 Hz motor-command profile with all documented telemetry active and representative PWM, bus-servo, indicator, OLED, and service traffic from [verification.md](verification.md), combined bidirectional USART1 wire traffic in every complete one-second window **shall** be less than 70,000 bytes. | Analysis, Test |
| MEM-001 | While the allocation seal is active in `ACTIVE`, the firmware **shall** make zero dynamic allocation, reallocation, or free calls; the post-seal allocation-attempt diagnostic shall remain zero in every non-fault-injection acceptance run. | Inspection, Test |
| MEM-002 | Across 100 consecutive Agent disconnect/reconnect cycles, allocated-byte counts and all static-pool availability **shall** return to their pre-cycle baselines with zero leak or monotonic growth. | Test |
| PERF-001 | During a continuous 60-minute input at 500 valid motor commands per second, the maximum fresh accepted-field age at each valid current-session `MotorControlTask` mailbox consumption **shall** have p99 no greater than 20 ms. | Test |
| PERF-002 | The PERF-001 test **shall** complete with no deadlock, executor overrun, unexpected reset, memory/queue growth, stale command replay, or failure of the 100 Hz motor loop or 200 ms motor lease. | Test |
| SOAK-001 | The integrated MCU, Agent, and C++ supervisor **shall** complete a continuous 24-hour nominal-load soak with no unexpected reset, deadlock, resource growth, stale replay, missed safety transition, or loss of required ROS entities. | Test |
| REC-001 | The release **shall** pass 100 USB cable removal/restoration cycles, 100 Agent kill/restart cycles, and 100 MCU reset cycles. | Test |
| REC-002 | In every REC-001 cycle, all seven publishers, seven subscriptions, and seven services **shall** be present in the ROS graph within 5 seconds after the physical transport and Agent are available. | Test |
| REC-003 | No actuator command accepted before a disconnect or reset **shall** be replayed after entity recreation; DC motors shall require a new post-recovery command that is still subject to the active model limit, 6 RPS implementation ceiling, configuration gate, and independent lease. | Test |

## 9. Related documents

- [hardware-baseline.md](hardware-baseline.md) — verified active hardware and
  pin/peripheral ownership.
- [architecture.md](architecture.md) — host/MCU components, tasks, queues, and
  ownership boundaries.
- [ros-interface-contract.md](ros-interface-contract.md) — normative ROS names,
  IDL, QoS, validation, and timing contract.
- [reliability-and-safety.md](reliability-and-safety.md) — watchdogs, safe states,
  fault handling, and recovery.
- [legacy-audit.md](legacy-audit.md) — legacy behavior, defects, and v2
  disposition.
- [development-standards.md](development-standards.md) — C++ and embedded
  implementation standards.
- [verification.md](verification.md) — requirement-to-test traceability and
  acceptance procedures.
