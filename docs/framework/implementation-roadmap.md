# Implementation Roadmap

## 1. Purpose

This roadmap orders the RRCLite v2 rewrite so that transport feasibility,
electrical safety, and bounded behavior are proven before broad functionality is
added. A stage may be split into smaller changes, but its exit gate may not be
bypassed. Runtime implementation starts only after documentation Gate D0 is
accepted.

The intended repository products are:

- `mentor_pi_interfaces`: v2 messages and services;
- `mentor_pi_mcu`: STM32F407 firmware, FreeRTOS application, and micro-ROS client;
- `mentor_pi_bringup`: native Agent launch/service assets, udev rules, C++
  configuration supervisor, YAML schema, and system tests.

Package names and the public ROS contract are fixed by
[ROS interface contract](ros-interface-contract.md).

## 2. Stage and gate summary

| Stage | Primary result | Exit gate |
| --- | --- | --- |
| 0. Specification | Accepted framework documents and closed decisions | D0 |
| 1. Transport feasibility | Minimal MCU micro-ROS node with representative v2 traffic | D1 |
| 2. Platform foundation | Safe boot, static RTOS runtime, transport lifecycle, diagnostics, watchdog | D2 |
| 3. Hardware drivers | Tested retained hardware behind owner-task interfaces | D3 |
| 4. ROS contract | All v2 endpoints, validation, QoS, and asynchronous services | D4 |
| 5. Host integration | Native Agent deployment and C++ configuration supervisor | D4-H |
| 6. Qualification | Complete functional, stress, fault, reconnect, and soak evidence | D5 |
| 7. Release | Reproducible signed-off artifacts and rollback package | D6 |

## 3. Stage 0: accept the specification

### Work

- Review every document linked from [the framework index](README.md).
- Resolve inconsistent limits, rates, names, safe states, and ownership.
- Confirm the physical board revision and bench fixture.
- Assign an owner and verification ID to every shall-level requirement.
- Record accepted review revisions in the release notes or issue tracker.

### D0 exit criteria

- All planned documents exist and all relative links resolve.
- The legacy audit classifies every legacy ROS endpoint and every retained or
  excluded hardware path.
- There is no open decision concerning transport, public schema, safe stop,
  memory capacity, or test threshold.
- Hardware and controls reviewers approve the 200 ms motor lease and servo-hold
  behavior.

No firmware or host runtime implementation may be merged before D0.

## 4. Stage 1: micro-ROS feasibility

### Work

Build the smallest useful firmware on the target board and production compiler:

- FreeRTOS plus the Humble micro-ROS static library;
- custom USART1 transport with 8 KiB circular RX DMA;
- the final `mentor_pi_interfaces` type support;
- one representative best-effort motor subscription;
- one representative reliable diagnostics publisher;
- one largest/worst-case service request and response;
- connection, entity creation, teardown, and reconnection;
- allocation accounting, link-map reporting, and stack watermark reporting.

The feasibility report shall record the generated maximum size of every CDR and
XRCE sample. The pinned Humble generated type support is normative: it shall
report a 388-byte maximum message-field payload for diagnostics, while the
pinned RMW/CDR serialization path shall produce exactly 392 bytes including the
four-byte encapsulation. That sample shall fit without XRCE fragmentation, and
stream-framing callback writes shall fit the 1 KiB TX bounce buffer. A
conflicting hand-calculated size blocks D1.

Use the native Humble Agent on Ubuntu 22.04. Do not use the legacy Python bridge,
Docker, Snap, USB CDC, or a reduced placeholder interface set.

### D1 exit criteria

All feasibility conditions in
[ADR-0001](adr/0001-mcu-ros-transport.md#mandatory-feasibility-gate) pass,
including the memory headroom, 500 Hz representative traffic, line-rate budget,
post-seal allocation, twenty reconnect, motor-lease, and five-second recovery
criteria.

Archive the compiler version, micro-ROS commit/tag, configuration metadata,
linker map, traffic capture, test command, and results. If D1 fails, stop and
reopen ADR-0001; do not continue with a hidden custom transport.

## 5. Stage 2: platform foundation

### Work

Implement in dependency order:

1. safe clock/GPIO/peripheral initialization with all drive outputs disabled;
2. static FreeRTOS tasks, queues, event objects, and allocation seal;
3. monotonic time, bounded/degraded Agent time synchronization, and
   generation/session ID;
4. production USART1 RX/TX DMA transport and error recovery;
5. `SAFE_BOOT -> WAIT_AGENT -> CREATE_ENTITIES -> ACTIVE -> TEARDOWN -> BACKOFF`
   lifecycle;
6. safety supervisor, task heartbeats, motor kill path, and sole-owner IWDG
   refresh;
7. diagnostic counters, reset-cause capture, queue high-water marks, and last
   error record.

At this stage, actuator drivers remain disabled except for controlled test
hooks. Tests must independently simulate a withheld reliable XRCE ACK while
USART1 TX completes normally; a ping-only failure phase in which reliable
operations are acknowledged; missing TX-DMA completion; missing UART TC
completion; HAL TX error; RX overflow; task stalls; and reset. A reliable
operation/session-liveness failure shall not be counted as a low-level
TX-completion timeout.

### D2 exit criteria

- Static analysis and host-native state-machine tests pass.
- There are zero allocation attempts while an entity generation is sealed and
  active. Entity recreation uses only the resettable bounded arena before the
  new generation is sealed, and each teardown returns the arena to its exact
  baseline.
- Agent absence never causes an uncontrolled reset loop.
- A withheld reliable ACK returns through the 10 ms reliable-operation bound
  and tears down the session without waiting for ping failure. In a separate
  phase, the harness schedules no reliable application operation, forwards any
  unrelated traffic, and drops only ping replies; three failed 10 ms pings at
  the 500 ms cadence use the Agent-loss path. Neither phase increments
  `transport_tx_timeouts`.
- Injected TX-DMA/TC completion loss obeys the length-derived write deadline,
  increments the TX-timeout diagnostic, and kills motor outputs before session
  destruction.
- Fault-proxy tests cover executor spin, request take, reliable
  publish/response, ping/time sync, and incremental entity create/finalize;
  every configured wait is bounded and `MicroRosTask` heartbeat age never
  exceeds 100 ms.
- Stalling each critical task causes the specified safe response and watchdog
  reset bound.
- Resource and stack budgets remain within [Requirements](requirements.md).

## 6. Stage 3: retained hardware drivers

### Work

Add one owner-controlled subsystem at a time:

1. encoder counter sampling, the default motor lock, guarded commissioning
   output, provisional PID state/model polarity, model limits, and per-motor
   command leases;
2. PWM-servo pulse generation, offsets, and interpolation;
3. UART5 half-duplex bus-servo transactions and timeout/correlation handling;
4. QMI8658 acquisition, six-face/three-axis frame characterization, the
   recorded raw-to-`imu_link` signed permutation, and unit conversion;
5. battery ADC filtering and threshold state;
6. buttons and bounded event generation;
7. LEDs, buzzer, RGB DMA, and OLED/I2C state machines.

Driver APIs shall expose fixed-size values and explicit status; they must not
depend on ROS types. Each asynchronous driver operation has a deadline and a
generation token so a completion from an expired request cannot affect a newer
request.

### D3 exit criteria

- Every retained row in [Legacy audit](legacy-audit.md) has a passing driver or
  HIL test and an owner-task assertion.
- The normal image accepts zero/stop but rejects nonzero motor commands, and the
  build fails if commissioning is requested without both
  `RRCLITE_MOTOR_COMMISSIONING=1` and
  `RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED`. Before any powered test,
  each raised wheel passes a passive encoder-direction check with PWM disabled.
  Initial powered characterization stays within 0.25 RPS and 300 permille on a
  current-limited guarded fixture. JGA27's evidence-derived `-1` factor and all
  PID profiles remain provisional until the complete `VER-HIL-MOT-001` record
  qualifies or replaces them; D3 cannot close for nonzero production motion on
  commissioning evidence alone.
- Lease boundary tests prove no expiry before 198 ms, expiry no later than
  200 ms, 1 kHz TIM7 releases, and no more than 2 ms between completed lease
  evaluations including scheduling jitter.
- PWM interpolation proves 21/39 ms requests complete at 40 ms and 41 ms at
  60 ms, every intermediate pulse follows the normative nearest-integer
  equation, and exact half ties round away from zero for positive and negative
  motion. Bus tests prove dispatch priority, accepted-service non-preemption,
  stop-at-frame-boundary behavior, discard/no-resume of interrupted move frames,
  and the exact persistent/volatile restore matrix; the fixture's original ID,
  saved offset, limits, and torque state are restored and read back afterward.
  Reapplying the active battery threshold is idempotent and does not reset
  either debounce timer.
- Valid, boundary, and invalid values produce the documented result without
  exceeding electrical or software limits.
- No driver waits forever, calls an allocator while `ACTIVE`, or performs
  blocking work in an ISR or micro-ROS callback.
- Excluded Gamepad, SBUS, Bluetooth, USB-host, LCD/LVGL, and chassis code is not
  linked into the release image.

## 7. Stage 4: complete the ROS contract

### Work

- Generate and integrate every v2 message and service exactly as documented.
- Register subscriptions and publishers in deterministic order.
- Validate entire commands before updating mailboxes or leases.
- Implement depth-one latest-value mailboxes and the depth-16 button event FIFO.
- Implement nonblocking service polling, static pending slots, worker dispatch,
  deadlines, generation checks, and result codes.
- Publish bounded telemetry and diagnostics at the documented rates.
- Enforce QoS, names, namespaces, fixed capacities, units, and time-sync rules.

### D4 exit criteria

- Automated graph inspection finds exactly the documented endpoints and types.
- Serialization boundary and invalid-input tests pass for every schema.
- Flood tests show only the documented coalesce, overwrite, drop-oldest, or
  `BUSY` behavior.
- All service timeouts leave peripheral and ROS state usable.
- No ROS callback touches hardware or waits for a worker.

## 8. Stage 5: host integration

### Work

Implement `mentor_pi_bringup` in first-party C++ and declarative system assets:

- stable udev selection for the CH9102F;
- native Agent launch and systemd service for Ubuntu 22.04 `amd64` and `arm64`;
- a C++ `configuration_supervisor` node;
- a validated YAML schema for motor model, four PWM offsets, and battery low
  threshold;
- session detection using first discovery, graph/heartbeat reappearance,
  `Heartbeat.agent_session_id`, and wrap-aware uptime discontinuity;
- ordered service application after a new ready session, with a 100 ms call
  timeout, at most four attempts, and 100/200/400 ms retry backoff;
- configuration-generation and Agent-session tags on every future so late or
  stale responses cannot activate motion;
- health/status publication or logs identifying applied, rejected, and pending
  configuration.

The supervisor shall apply no actuator motion. It waits for the MCU `READY` or
`DEGRADED` state, applies motor model only while all targets are zero, then PWM
offsets and battery threshold. Only `BUSY`, returned `TIMEOUT`, and client
timeout are retryable; every other result is permanent for that generation.
After MCU reset it restores the validated non-default runtime configuration in
order. After Agent-only recovery it idempotently reapplies those values without
touching bus-servo persistence or replaying another command. A session change
invalidates every outstanding future and leaves motion disabled.

For Ubuntu 24.04 development, document the pinned Ubuntu 22.04/Humble Docker
image and keep ROS out of the native host OS. macOS-native deployment is not
supported.

### D4-H exit criteria

- The same bring-up package passes on Ubuntu 22.04/Humble `amd64` and `arm64`.
- Agent and supervisor run without root; project-owned nodes and the
  control/data path contain no Python. Pinned upstream ROS tooling may use its
  own Python implementation/dependencies.
- Replug, Agent restart, and MCU reset cause one successful ordered
  configuration transaction per new session. Tests prove the 100 ms timeout,
  four-attempt maximum, 100/200/400 ms backoff, retryable-result whitelist,
  stale-future rejection, runtime-value restoration, and bus-persistence
  noninterference.
- Invalid YAML prevents configuration activation and reports a precise error.

## 9. Stage 6: qualification

Run the complete [Verification](verification.md) matrix on the release candidate
with the final interface, middleware configuration, firmware optimization,
Agent build, system service, udev rule, YAML, and representative hardware load.
Copy and complete the
[qualification evidence ledger](qualification-evidence-ledger.md) inside the
immutable candidate evidence root; software-observed campaign artifacts and
independent physical records remain separately identifiable.

### D5 exit criteria

- Every mandatory verification ID passes with archived raw results.
- The one-hour 500 Hz stress test and 24-hour soak pass without exceptions.
- One hundred USB reconnects, Agent restarts, and MCU resets each pass.
- Fuzz, overload, UART-error, withheld-Agent-ACK/session, injected
  TX-DMA/TC-completion failure, service-timeout, bounded-middleware-call, and
  task-stall injection satisfy their acceptance criteria.
- Flash/RAM/stack, traffic, allocation, latency, recovery, and motor stop limits
  all pass simultaneously.
- No unexplained reset, endpoint disappearance, stale command, or unbounded
  resource growth remains.

Any change after qualification that affects code generation, optimization,
middleware pools, queues, timing, ISR behavior, or public interfaces invalidates
the affected qualification evidence.

## 10. Stage 7: release and rollback

### Release contents

- source revision and dependency lock information;
- reproducible build container or documented toolchain;
- firmware binary, ELF, linker map, checksum, and debug symbols;
- generated interface artifacts and ROS packages;
- udev, systemd, launch, and YAML files;
- completed requirement/verification trace matrix;
- stress/soak/resource/diagnostic reports;
- board revision, fixture, and calibration/configuration record;
- flashing, recovery, and rollback instructions.

### D6 exit criteria

- A clean machine reproduces all artifacts and checksums from the release source.
- A second operator can install, flash, configure, exercise, and roll back the
  controller using only the release instructions.
- Rollback restores the previously approved firmware and host packages without
  changing board hardware.
- Release notes enumerate public API version, supported platforms, known limits,
  and upstream dependency revisions.
- No release artifact accepts nonzero motor targets unless the exact binary has
  completed the motor PID/polarity HIL requirements. Until that evidence and a
  reviewed production-authority change exist, the released image remains
  motor-locked and any commissioning image is archived separately as non-release.

## 11. Change and issue discipline

Every implementation change must name the requirement IDs it satisfies and the
verification IDs that prove it. Changes spanning task ownership, safe state,
wire transport, ROS schema, or resource budgets require review from both the MCU
and host/ROS owners.

Temporary test hooks must be compile-time excluded from release builds. A stage
cannot close with an undocumented waiver; an accepted waiver must state its
scope, safety impact, owner, expiry, and compensating test.

## 12. Future ROS distribution migration

This roadmap releases only Ubuntu 22.04/ROS 2 Humble artifacts. Before Humble
reaches end of life in May 2027, schedule a separate migration review for
Ubuntu 24.04/ROS 2 Jazzy. That review shall create new immutable host, Agent,
and MCU dependency locks; rebuild generated types and serialization evidence;
and rerun every affected software, transport, reconnect, stress, and HIL gate.
Until that work passes, Jazzy is neither a supported runtime nor a fallback,
and mixed-distribution deployment is prohibited.
