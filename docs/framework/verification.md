# Verification and Acceptance Plan

## Unified image and tracking-controller gates

`VER-IMG-UNIFIED-001` proves that RDK setup selects one project image and one
unique Humble base pull, while a normal computer selects that project image
plus one Noble quality image and two unique base pulls. It verifies the native
default, the 2026-08-07 per-architecture ROS lock, exact installed versions,
one Arm GNU 13.2.1 installation, content identity, and micro-ROS cache reuse.
`VER-CROSS-HANDOFF-001` separately verifies the dedicated amd64-to-arm64
`make rdk-handoff` workflow, target-only inputs and metadata, rejection outside
that target, and the native-evidence boundary.

`VER-TRACK-UNIT-001` covers polynomial evaluation and numerical derivatives,
message bounds and validation, unwrapped yaw and shortest angle error, both
vehicle models, geometry/profile constraints, replacement scheduling,
cancellation, ROS-to-steady clock conversion, stale odometry, solver fallback,
and every zero-output condition. `VER-TRACK-INT-001` supplies synthetic
odometry and mocked controller inputs for both stable topic contracts.
`VER-TRACK-RDK-001` is a native RDK X5 benchmark requiring sustained 30 Hz
operation and recorded 25 ms solve-deadline behavior. The unit and integration
tests do not satisfy the RDK benchmark or any powered HIL gate.

## Purpose and release rule

This plan proves functional parity for the accepted scope and, specifically,
that high-rate traffic cannot wedge the MCU. It verifies the
[requirements](requirements.md), [legacy audit](legacy-audit.md),
[ROS interface contract](ros-interface-contract.md),
[architecture](architecture.md), and
[reliability and safety contract](reliability-and-safety.md).

Verification IDs use the normative `VER-<AREA>-NNN` form. A release candidate
passes only when every mandatory case applicable to the released hardware has
passed on the exact firmware binary and host package set being released.
Missing evidence is a failure. A mandatory result cannot be waived by raising a
queue limit, extending a deadline, or excluding a retained device.

## Test environments

| Environment | Required use |
|---|---|
| Host-native | Firmware domain logic compiled for the host, C++ ROS logic, deterministic fake clock/HAL, sanitizers, unit tests, and fuzzers. |
| Agent loopback | ROS 2 Humble and the micro-ROS Agent connected through a pseudo-terminal or serial fault proxy; no physical actuators. |
| Controller HIL | Production STM32F407VET6 board and CH9102F/USART1 path, instrumented reset and power control, logic analyzer, current-limited supply, and safe/disconnected actuator fixture. |
| Full peripheral HIL | Controller HIL plus four encoder motors or electrical simulators, four PWM loads, at least one supported bus servo, QMI8658, OLED, buttons, LEDs, buzzer, RGB, and programmable battery input. |

Motor/load tests shall use wheels lifted, electrically simulated outputs, or an
equivalent guarded fixture. Automation shall never depend on a person stopping
unsafe motion.

Before any powered motor case, archive and verify the exact default PID build,
then manually rotate each raised wheel with drive outputs disabled and record
raw and normalized encoder direction. A powered pre-qualification case shall
use a current-limited supply, raised wheels or equivalent guarding, deliberately
bounded commands, and continuous stop monitoring. Those precautions and the
firmware limits reduce checkout risk; none is evidence that any PID profile,
physical polarity, or regulated speed is release-qualified.

## Required evidence

Every run stores:

- source revision, firmware SHA-256, host package versions, ROS distribution,
  compiler versions, board serial/revision, and fixture revision;
- start/end times, complete console and Agent logs, random seeds, result, reset
  reason, heartbeat, and diagnostics capture;
- firmware map/size files, static-buffer and middleware-pool reports, task
  execution and stack high-water marks, and allocation counters;
- escaped USART1 RX and TX byte counts for every complete one-second window;
- rosbag or equivalent typed capture for ROS tests; and
- logic-analyzer or instrumented GPIO traces for motor lease, control release,
  reset-safe output, PWM, and other timing measurements where specified.

Results shall be machine-readable as JUnit plus JSON/CSV metrics. Endurance and
recovery tests retain the complete uninterrupted log, not only a failing
excerpt.

## Review, inspection, unit, integration, and fuzz cases

| Test ID | Exact acceptance |
|---|---|
| `VER-TRACE-001` | A documentation checker extracts every shall-level ID in `requirements.md` and every included audit ID in `legacy-audit.md`. Each maps to at least one defined `VER-*` case, every referenced case exists, all framework links resolve, and reviewer approval records no open safety/interface decision. |
| `VER-REVIEW-001` | Review confirms the product is a clean micro-ROS client/Agent design for STM32F407VET6, contains no duplicate legacy packet runtime, and implements the ownership, C/C++ boundaries, timeout, safe-state, and source-of-truth rules in the framework documents. |
| `VER-BUILD-HOST-001` | In architecture-native pinned Ubuntu 22.04/Humble containers on supported Ubuntu development hosts, debug, release, ASan/UBSan, and TSan jobs build and test all first-party host C++, generated interfaces, `mentor_pi_bringup`, and `mentor_pi_hardwares` with zero first-party warnings. Generated build metadata records `ubuntu=22.04`, `ros_distro=humble`, a content-addressed builder, and the automatically detected `amd64` or `arm64` architecture and rejects an OS, distribution, architecture, source-fingerprint, metadata-schema, or image-architecture mismatch. The hardened runtime passes only the reviewed CH9102F character device and never uses a privileged container. Project-owned production nodes and the control/data path contain no Python node, script, `rclpy` use, or serial translation bridge; ROS launch files may use the upstream Python launch framework. Pinned upstream ROS tools and their Python implementation/dependencies are permitted and reported separately. |
| `VER-IMG-UNIFIED-001` | RDK setup selects one project image and one unique Humble base pull; normal-computer setup selects that project image plus one Noble quality image and two unique base pulls. The native-architecture default, signed 2026-08-07 architecture-specific ROS lock, exact installed versions, one verified Arm GNU 13.2.1 installation, complete content identity, and micro-ROS cache reuse all pass. |
| `VER-CROSS-HANDOFF-001` | On amd64, `make rdk-handoff` preflights QEMU/binfmt without requiring another authorization or target variable, uses only `linux/arm64` base images, package locks, tools, binaries, and OCI descriptors, and produces a handoff loadable on a native arm64 Docker host without rebuilding the host workspace. With eight or more CPUs it prints and uses eight package workers; smaller hosts scale down, a valid override is honored, and nested package builds remain single-worker. Each of the eight output-checksummed stages is skipped only after validation; interruption preserves its release ID and work, incremental host state and failed tests resume, and changed source/image/dependency/runner inputs refresh the private context while invalidating only affected or dependent stages. A packaging-only edit preserves compiled host, Agent, firmware, and micro-ROS stages; a host-source edit preserves independent firmware and Agent stages and resumes colcon incrementally. Malformed/symbolic/tampered state fails, an explicit fresh reset is precise, and success alone clears active disposable state. Firmware packaging and packaged runtime smoke do not rebuild firmware or host code. Ordinary `make host` remains amd64-native and generic cross-architecture requests remain rejected. Handoff evidence records `build_execution=qemu-emulated`, `build_host_architecture=amd64`, `target_architecture=arm64`, and `native_target_validated=0`. On the RDK, the compact receive/install actions reject the wrong platform/profile, malformed or symbolic archives, bad outer/host checksums or symlinks, mismatched release/image/Agent identities, an unsafe device identity, and invalid systemd units while avoiding timestamp placeholders and manual deployment plumbing. Missing emulation, mixed-architecture input, an amd64 output member, an implicit fallback, or any claim of native RDK validation fails. Native RDK image-load, runtime, memory, 30 Hz/25 ms tracker, serial, flash, peripheral, HIL, and release cases remain independently required; native RDK compilation is optional diagnostic evidence. |
| `VER-TRACK-UNIT-001` | Unit tests cover polynomial evaluation and numerical derivatives, bounded message validation, unwrapped yaw and shortest angle error, both vehicle models, geometry/profile limits including every mecanum wheel bound, scheduling and cancellation, ROS-to-steady conversion, stale state, bounded feedback, solver failure/deadline handling, and all zero-output conditions. |
| `VER-TRACK-INT-001` | Integration tests supply synthetic odometry, motor profile, authorization, and scheduled trajectories to each tracker; verify its stable trajectory, controller-reference, cancel, and diagnostic contracts; observe bounded nonzero output; and prove stale odometry and cancellation return output to zero. |
| `VER-TRACK-RDK-001` | A native RDK X5 benchmark records project-image identity, memory, sustained 30 Hz operation, solve duration and every 25 ms miss/failure for both models. It must pass before hardware tracking is enabled and does not replace guarded HIL evidence. |
| `VER-BUILD-FW-001` | `make firmware` drives the authoritative CMake/Ninja graph in architecture-native pinned Docker. Two clean builds produce identical loadable sections. The target is STM32F407VET6 with the pinned Arm GNU 13.2.1 toolchain, HAL, FreeRTOS, and Humble-compatible micro-ROS client; 33 generated-workspace checkouts are detached at their checked-in Humble source-lock commits, the deferred `libyaml` checkout is required at the lock's 34th commit, and the temporary `geometry2/tf2_msgs` source is pinned separately. Missing or unexpected repositories fail the build. A generated-library cache is reused only when image, lock, interfaces, generator inputs, helper fingerprint, archive hash, and header-tree hash match; corrupt or stale caches are removed and regenerated. The reviewed archive hash, first-party warning policy, ELF/BIN/HEX/map metadata, and memory report remain mandatory. Verification rejects native-path, older, or other-distribution artifacts. |
| `VER-BUILD-MOTOR-GATE-001` | For SAFE-006, `make firmware`, `make flash`, and `make start` select the single `NORMAL_CLOSED_LOOP_DEFAULT` artifact with `control_mode=CLOSED_LOOP`. Build, metadata, verification, flashing, packaging, and runtime tests reject any alternate classification or control mode, stale metadata, changed source/dependency/artifact hashes, mutated snapshots, programmer failure, and missing read-back success. The artifact reports a 6 RPS implementation ceiling, each lower model limit, and a 1000-permille output clamp. Handoff packaging contains exactly one ELF/Hex/Bin/Map set plus `BUILD-METADATA` and `BUILD-MODE`. Software evidence does not qualify PID performance or powered motion. |
| `VER-SCOPE-001` | Source, link map, task inventory, and ROS graph contain no legacy `AA 55` protocol, legacy names/types, Gamepad/Joy, SBUS, Bluetooth, USB-host, native-MCU USB transport, LVGL/LCD, or chassis-kinematics runtime. The I2C OLED remains included and is not classified as LCD. |
| `VER-API-001` | The MCU node is exactly `/mentor_pi/controller` and owns exactly the seven publishers, seven subscriptions, and seven services in the ROS contract. `/rosout`, parameter, parameter-event, statistics, action, and `/clock` endpoints are absent. Names, generated types, fields, units, and direction match exactly; no legacy alias exists. Other expected processes, such as the Agent and C++ supervisor, may have their own nodes. Nominal publication rates measured over 60 seconds are within ±5%. |
| `VER-SERIALIZATION-001` | Host and firmware generate types from the same IDL. Introspection proves every endpoint type has a finite maximum serialized size, every array is fixed, every string is bounded, and no unbounded sequence/string exists. CDR round trips cover zero/default, every boundary, and maximum-size values for every message, request, and response with byte-for-byte field equality and no handwritten wire struct. The pinned Humble generated type support is normative and shall report a 388-byte maximum message-field payload for `ControllerDiagnostics`; serialization through the pinned RMW/CDR path shall produce exactly 392 bytes including the four-byte encapsulation. Generated CDR plus XRCE headers for every maximum sample is no greater than the 512-byte MTU and requires no endpoint-level fragmentation; conflicting handwritten size arithmetic fails the test. |
| `VER-CONFIG-001` | One production systemd service starts the content-addressed runtime image rather than using `ros2 run`. The container runs the compiled Agent and configuration supervisor as one fail-coupled launch, selects `/dev/mentor_pi_mcu`, mounts only reviewed artifacts/configuration and udev state read-only, uses host networking, a read-only root, dropped capabilities, temporary writable state, and an unprivileged service identity; systemd owns restart and stop behavior. The node `/mentor_pi/configuration_supervisor` accepts every valid boundary of the exact YAML schema and rejects unknown keys, wrong types/counts, invalid model strings, and adjacent out-of-range values while keeping its motion gate false. Each exact motor-model response profile passes; an `OK` response with a mismatched active model, tick count, adjacent representable finite speed, NaN, or infinity is converted to `IO_ERROR`, attempted once, and cannot enable motion. First discovery, graph/heartbeat reappearance, session-ID change, wrap-aware uptime discontinuity, and normal `uint32` uptime wrap are each induced; only the first four create a new configuration generation. For each ordered call, the documented retry policy and generation/session tags reject late responses. Permanent failure or exhaustion leaves the gate false. Across ten repetitions of each new-session trigger, configuration is restored in order after MCU reset and idempotently reapplied after Agent-only recovery; normal uptime wraps trigger no generation. No bus persistence or command is replayed. |
| `VER-HOST-COMMISSION-001` | Native core and ROS graph tests prove the guarded C++ utility accepts only a signed 0.01--0.25 RPS direction command and exactly one `/mentor_pi/configuration/motion_authorization` publisher whose identity is `/mentor_pi/configuration_supervisor`; zero, duplicate, wrongly named, disappearing, or changed generation/session authorization fails closed. A second `/mentor_pi/motors/command` publisher also prevents motion. A drive publish gap greater than 100 ms initiates stop and never resumes the drive phase. Heartbeat sequence/uptime or diagnostic uptime regression, any selected lease-expiry or watchdog-counter change, and a nonzero watchdog mask abort. Selected speed at or above 0.50 RPS, at least two encoder ticks in the wrong direction, unselected speed above 0.02 RPS, and unselected displacement of at least two ticks each abort; one isolated opposite tick remains inconclusive. Admission and target echo without a correctly directed response of at least `max(0.002 RPS, 10% of target)` and two correctly directed encoder ticks cannot pass. Post-stop success requires the latest state to report all four targets zero and all four `abs(measured_rps) < 0.002 RPS`; a later target invalidates an earlier zero-target latch, and residual speed with zero targets fails. `SIGINT`/`SIGTERM` exercise the bounded zero phase, while injected stop-control and exception/shutdown paths request an immediate all-motor zero; loss of ROS publication leaves the independent 200 ms MCU lease as the documented fallback. After software success the helper requires an explicit operator confirmation of physical direction. |
| `VER-UNIT-VAL-001` | Boundary tables cover every mask, count, ID, enum, scalar range, fixed-array limit, string byte/character rule, duration, duplicate bus ID, and NaN/Inf case. Invalid input produces the specified `Result` or topic diagnostic, changes no state, performs no hardware call, and refreshes no motor lease. Atomic multi-element rejection and exact `detail` values are checked. |
| `VER-UNIT-MOTOR-GATE-001` | For SAFE-006 and SAFE-007, the default closed-loop motor configuration accepts every valid selected zero or bounded nonzero subset, rejects a selected non-finite or out-of-range value atomically without applying output or refreshing any lease, and has no alternate mode branch. Closed-loop tests prove the positional P term, time-integrated I term, error-difference D term, exact period scaling, filtered-speed input, positive and negative +/-1000-permille saturation, conditional anti-windup recovery, deadband, all four legacy model limits, the 6 RPS implementation ceiling, independent leases, state resets, and session-loss disarming. `set_pid` tests cover every result code, atomic multi-channel application, state reset, reconnect retention, reset/model-change volatility, and failure masks. Portable tests assert the measured connector/encoder mapping while keeping unqualified physical polarity provisional. |
| `VER-UNIT-MBOX-001` | Motor, PWM, LED, and OLED tests first publish a complete state and then update one selected element in the same session; every selected element changes and every unselected element remains unchanged. RGB tests replace only host-owned RGB1 and prove masks selecting RGB2 are rejected without changing either pixel. Multiple writes to the same selected element before one read expose only the newest generation and increment overwrite accounting exactly as the ROS contract specifies. For each multi-field merged mailbox, leave a selected field unread, reset its session ownership, then publish a disjoint field: the old field-valid generation is zero, its pending value is absent, the new field is present, and the monotonic mailbox generation does not restart. The triple-buffer test races producer `Publish` plus producer `DiscardLatest` against `ConsumeLatest`; every consumed object remains whole and strictly newer, a discard winning after the consumer's initial unread observation returns no value, and the final non-discarded generation remains observable. Controller tests repeat the reset at a real session transition: unread gen-1 motor/PWM/LED/RGB1/OLED fields never arm, target, or render in gen 2; stale whole-command bus/buzzer work performs no hardware transaction. A separate contrast proves applied LED/OLED state and RGB1 hold across reconnect while RGB2 remains exclusively system-owned. Bus motion and buzzer complete-state slots otherwise expose only the newest same-session complete write. All storage remains at its documented depth. |
| `VER-UNIT-BTN-001` | The internal button FIFO returns events 1–16 in order. Inserting 1–17 without a read returns 2–17 and increments the drop counter once; inserting 32 into an empty paused queue returns 17–32 and increments it 16 times. All diagnostic counters, including button drops, saturate at their maximum and never wrap. ROS writer history is independently verified as depth 8. |
| `VER-UNIT-LEASE-001` | With a fake microsecond clock and every phase relative to the 1 kHz TIM7 release, the micro-ROS callback reads the exact microsecond value through its required runtime hook and the adapter/mailbox preserves it without millisecond rounding. A nonzero output remains eligible through every sampled age below 198 ms and becomes zero in the inclusive 198–200 ms window, never earlier than 198 ms and never later than 200 ms after firmware acceptance. The measured interval between completed lease evaluations, including scheduling jitter, is no greater than 2 ms. Each motor is independent; a valid subset refreshes only selected motors, invalid input refreshes none, zero-target aging creates no watchdog trip, and clock wrap is covered. |
| `VER-UNIT-SVC-001` | Each non-bus slot admits one request and returns `BUSY` to a concurrent request; local worker completion or typed failure occurs within 50 ms. The completion and request-group cursors give each continuously occupied/ready local or shared slot one turn per five service-class opportunities, including the PID slot. The shared bus slot is non-preemptible: while occupied, every additional bus request, including stop, returns `BUSY`, and the three busy endpoints rotate without starvation. When free with simultaneous work, stop dispatches first and get/configure alternate deterministically; every accepted service dispatches before pending motion, and only the newest pending move remains. A stop arriving during a move completes the current UART frame and interrupts before the next move frame; it never cuts an in-flight frame or preempts an accepted get/configure service. Every unsent frame from the interrupted move batch and every pending pre-stop move generation is discarded. The explicit sequence A active → B pending → stop transmits neither A's remainder nor B; only a move accepted after stop may restart traffic. Bus work finishes or returns `TIMEOUT` by 200 ms MCU time and the host returns by 250 ms. Generation-tagged late completions cannot satisfy a later request. |
| `VER-UNIT-MW-001` | Instrument every `MicroRosTask` middleware boundary: `rclc_executor_spin_some`, nonblocking `rcl_take_request`, best-effort and reliable publish, `rcl_send_response`, Agent ping/time sync, and each incremental create/finalize step. With the fault proxy withholding input, replies, or reliable ACKs at each boundary, spin waits are at most 1 ms; transport-read slices, ACTIVE ping/time sync, reliable publish/response, and remote finalizer calls are at most 10 ms each; creation-time sync is at most 20 ms; each entity-creation call is at most 40 ms and the complete creation phase at most 2 s; complete remote destruction is at most 500 ms; and each custom write obeys its length-derived deadline. No individual/incremental step creates a `MicroRosTask` heartbeat interval over 100 ms, waits for a peripheral, grows storage, or evades the documented error/teardown path. Portable tests prove strict service → reliable-telemetry → maintenance rotation and a single shared blocking-operation permit: across a response, reliable publication, ping, and active time sync, at most one starts per ACTIVE slice, and no class starves under continuous work in the other two. At most one due best-effort motor/PWM/IMU publication and, on its selected class, one due reliable publication starts per slice; both publisher sets rotate fairly. A deliberate second permit request tears down the test session. |
| `VER-UNIT-RXDMA-001` | Exhaustive small-ring and boundary-value tests prove epoch/cursor phase validation, half/full transitions, a complete lap returning to the same cursor, 32-bit epoch wrap, modulo producer subtraction, and overrun detection. Target-source and map inspection prove DMA2 Stream 2 enters standard `HAL_DMA_IRQHandler`, USART1 enters standard `HAL_UART_IRQHandler`, both standard HAL half/full callbacks are the only incrementing epoch writers while active, open resets the epoch atomically before `HAL_UART_Receive_DMA`, and every USART1/DMA IRQ is at the FreeRTOS-safe transport priority. Source checks reject the removed project-owned top half and UART-IDLE path. On target, line-rate traffic and critical-section instrumentation prove the maximum masked interval remains below 40.96 ms, producer progress stays exact across half/full boundaries and wraparound, and a deliberate overrun latches a fatal fault rather than aliasing or dropping bytes silently. |
| `VER-UNIT-QOS-001` | Programmatic endpoint inspection matches reliability, durability, history, and depth for all 21 endpoints exactly, including internal button FIFO 16 versus ROS writer history 8. Offered/requested compatibility with a standard `rclcpp` peer is exercised. |
| `VER-UNIT-STATE-001` | Every legal and illegal session/safety transition is covered. Reset establishes motors zero/disabled, PWM low then 1500 microseconds, LEDs/buzzer/RGB off, empty host OLED lines, and no bus frame. During startup grace, an adversarial first-generation nonzero command remains retained while one peer heartbeat is absent: neither arm nor duty is called across repeated MotorTask releases; after every peer is observed, the same generation proceeds without reconnect. A reentrant stop probe proves session deactivation is already observable as inactive and is inside the controller critical section when the physical stop occurs; a queued command from the preceding generation cannot arm after reactivation. Active-generation downgrade/replacement, stale deactivation, replay, and `UINT32_MAX` to `1` wrap cases prove the monotonic watermark contract. Each of the seven topic gateways is deterministically preempted after its fast session check but before its critical-section recheck; teardown plus activation of the next generation makes every callback return `BUSY`, publish nothing, and leave critical nesting balanced. Target-source checks require both motor arm and four-channel duty hooks to perform transport-latch pre-check → write → post-check ordering; each failed check must emergency-stop before returning `BUSY`, the duty post-check must follow the complete four-channel update, and the latch has no runtime clear outside normal transport open. Deterministic preemption tests inject `BUSY` transport inhibition and non-`BUSY` encoder/arm/duty failures, then run a pending MotorTask iteration: no output re-arms, the former admits only a different fresh generation, and controller/watchdog faults admit none before reset. A stale-task supervisor stop is tested with powered fake output and a queued command so the watchdog delay cannot become a re-arm window. Ordinary Agent loss holds PWM and bus-servo state but disarms motors; all old command and service generations are invalidated; recovery requires a fresh command that still satisfies the current model limit, 6 RPS ceiling, and session gate. |
| `VER-UNIT-TIME-001` | Before first synchronization all ROS stamps are exactly zero and the heartbeat synchronization flag is clear while monotonic lease timing remains active. After synchronization, stamps use Agent epoch time and never regress. Ten induced resynchronizations occur no more than 60 seconds apart without a lease or control discontinuity. |
| `VER-UNIT-DIAG-001` | One controlled injection for every subscription rejection, mailbox overwrite, button drop, publication error, service outcome, motor lease, executor, USART1, peripheral, timeout, allocation, reset, and watchdog category changes only the documented counter/source/detail. Motor-age tests prove one sample per valid current-session owner consumption, maximum age across merged fresh accepted fields, exactly 20,000 us passing, 20,001 us counting, monotonic high-water behavior, unsigned microsecond-clock wrap, and no sample from rejected fields or stale/session-discarded work. Watchdog-retention tests cover every valid task boundary, magic/complement/version/reserved/task corruption, torn write phases, reset-cause gating, unconditional consume/clear, first-offender one-shot behavior, and the rule that a live stall cannot change the published prior-boot offender. Saturation is tested; no counter wraps. Session, task heartbeat-age, and resource fields match independent instruments. |
| `VER-FUZZ-VAL-001` | At least 10,000,000 generated or mutated instances distributed across every v2 command/service type execute under ASan/UBSan. The corpus includes maximum counts, malformed masks/text, duplicate IDs, every enum byte, NaN/Inf, and signed/unsigned boundaries. Acceptance is zero crash, sanitizer finding, hang, state change after rejection, or invalid hardware call. |
| `VER-FUZZ-TRN-001` | At least 10,000,000 arbitrary input bytes plus every split point of representative valid XRCE frames, random drops/duplicates/bit flips, and 8 KiB ring-wrap boundaries execute in the project transport harness. Acceptance is zero crash, out-of-bounds access, unbounded loop, leak, stale-generation callback, or application callback caused by invalid data. |
| `VER-ANALYSIS-001` | Every mandatory tool and rule in `development-standards.md` passes; no unresolved high/critical result remains. First-party platform-independent logic reaches at least 90% line and 80% branch coverage. Review confirms callbacks/ISRs have bounded, nonblocking call graphs and all first-party queues/storage are statically bounded. |
| `VER-INT-TRN-001` | Inspection and logic-analyzer capture confirm USB-C data → CH9102F → PA9/PA10 USART1 at 1,000,000 baud, 8N1, no flow control, opened through `/dev/mentor_pi_mcu`. Firmware reports an 8 KiB circular RX DMA ring, 1 KiB TX bounce buffer, and XRCE MTU 512. Instrumentation proves generated XRCE messages fit that MTU, stream framing/CRC/stuffing flushes through bounded callback writes no larger than 1 KiB, escaped physical bytes are included in traffic counters, and standard HAL half/full callbacks plus the `ring_size - NDTR` cursor count exact progress across same-cursor full laps. Sub-half-ring replies are observed through polling within 1 ms. A ten-minute nominal typed-traffic run has zero framing/parity/overrun error, RX loss, transport timeout, endpoint loss, or MCU reset. |

Upstream micro-ROS/XRCE code is not requalified by project source coverage, but
every project-owned adapter, buffer, state transition, and boundary is.

## Functional HIL cases

| Test ID | Legacy trace | Exact acceptance |
|---|---|---|
| `VER-HIL-LED-001` | `ROS-LED-01`, `HW-LED-01` | LED1 and LED2 independently perform steady off/on, one finite cycle, multiple finite cycles, and indefinite repetition followed by replacement. Measured on/off intervals are within ±2% or 2 ms, whichever is larger. A ROS command for LED3 is rejected without changing it; LED3 toggles only after successful ROS heartbeat publication. |
| `VER-HIL-BUZ-001` | `ROS-BUZ-01`, `HW-BUZ-01` | Off, steady tone, finite, indefinite, and replacement behavior match the contract. Frequency and on/off intervals are within ±2% or 2 ms, whichever is larger. The exact battery-alarm priority/resume sequence is also exercised. |
| `VER-HIL-MOT-001` | `ROS-MOT-01`, `HW-MOT-01` | The test is staged with the exact verified default PID artifact. First, keep all bridge outputs at zero, manually rotate each raised wheel in the declared positive direction, and record raw counter and normalized `motors/state` signs. Second, use raised-wheel/equivalent guarding and a current-limited supply; exercise one channel and one sign at a time with deliberately bounded targets, require encoder and operator-observed physical direction to agree, verify output limiting and stop/lease/session kill paths, and stop on wrong sign, unexpected current, oscillation, or cross-channel response. Recorded HIL must qualify or replace every channel/model output sign, encoder constant, PID/filter/deadband profile, full documented RPS range, subset merge, zero behavior, model-change behavior, and independent state before powered-motion or PID-performance claims are made. |
| `VER-HIL-PWM-001` | `ROS-PWM-01`, `ROS-PWM-02`, `HW-PWM-01` | All four outputs exercise subset merge, synchronized targets, 500/1500/2500 us pulse values, and −100/0/+100 us offsets. Interpolation uses 20 ms frames and never finishes before the requested duration: 20 ms uses one frame, 21 and 39 ms each use two frames and finish at 40 ms, 41 ms uses three frames and finishes at 60 ms, and 30000 ms uses 1500 frames. At every intermediate boundary `Bk`, positive and negative trajectories equal `start + sign(q) * floor((abs(q) + N / 2) / N)` exactly. Half ties round away from zero in both directions—for example, with `N=2`, 1500→1501 installs 1501 at `B1` and 1500→1499 installs 1499. Reapplying the active offsets returns `OK` without changing an output, target, interpolation phase, or `moving_mask`. Pulse width is within ±10 us; state reports target/output/offset and `moving_mask` at each boundary; invalid requests change no channel. |
| `VER-HIL-BUS-001` | `ROS-BUS-01`, `ROS-BUS-02`, `HW-BUS-01` | Before mutation, power-cycle the fixture and record/read back its effective ID, saved offset, position/voltage/temperature limits, and torque state. Move and stop arrays of 1 and 16 unique devices work; dispatch priority, accepted-service non-preemption, stop-at-frame-boundary behavior, and absence of implicit resume are exercised exactly as in `VER-UNIT-SVC-001`, including A active → B pending → stop with no remaining A or B frame transmitted and a later post-stop C move succeeding. After a servo power cycle, changed ID and position/voltage/temperature-limit values persist, an offset persists only after `SAVE_OFFSET`, and an unsaved offset and torque command do not persist; a logic trace proves MCU reset/reconnect emits no implicit restore frame. At test end, restore the recorded ID and all persistent limits/offset in an order that keeps the device addressable, power-cycle and read back those persistent baseline fields, then restore and read back the recorded torque state. Missing/corrupt devices return the exact result/mask by deadline; a post-write fault returns `PARTIAL` with the exact applied mask; write-only success claims transmission only. Motor lease enforcement never misses. |
| `VER-HIL-RGB-001` | `HW-RGB-01` | RGB1 mask 1 produces exact off, primary, secondary, and white component values and a conforming serialized waveform. Zero, masks 2/3, and unknown masks are rejected without changing either pixel. On RGB2, red pulses on sampled RX progress, green pulses on sampled TX progress, blue remains off, simultaneous RX/TX appears yellow, and a closed transport clears red/green. The status path performs no allocation or UART-ISR work. |
| `VER-HIL-OLED-001` | `HW-OLED-01` | Each line and both-line mask independently replace/clear host lines while the controller battery indication remains unchanged on SSD1306 page 3 and page 2 remains blank. Empty, one-byte, and 23-byte values pass for all 95 printable-ASCII glyphs with distinct uppercase/lowercase rendering; a 24-byte value and every single-byte value outside 0x20–0x7e (including UTF-8 lead/continuation bytes) are rejected without a buffer or display change. |
| `VER-HIL-IMU-001` | `ROS-IMU-01`, `HW-IMU-01` | Before the transform is enabled, a successful raw QMI8658 acquisition precedes the single expected transform-`UNSUPPORTED` diagnostic; an injected raw-I2C error increments the IMU error/timeout diagnostics and cannot be reported as the expected characterization condition. Register readback proves accelerometer ±4 g and gyroscope ±128 degrees/s at 250 Hz with sensor LPFs disabled. Six-face gravity and positive rotation about each board axis establish and record the QMI8658-to-`imu_link` signed permutation before the endpoint is enabled. At rest, the vector magnitude of `linear_acceleration_m_s2` is 9.80665 m/s² ±5% and all three `angular_velocity_rad_s` components are within the QMI8658 documented zero-rate tolerance. Axes/signs match the fixed contractual frame, conversion is exactly g×9.80665 and degrees/s×π/180, range endpoints are bounded by ±39.2266 m/s² and ±2.234021 rad/s, `stamp` is monotonic after sync, `valid` is true for successful samples and false after an injected read failure, no orientation/covariance/string field exists, and rate is 50 Hz ±5%. |
| `VER-HIL-BTN-001` | `ROS-BTN-01`, `HW-BTN-01` | Both buttons obey the 30 ms scan/two-sample debounce, 1500 ms long-press threshold, 400 ms repeat interval, and 300 ms multi-click window within one scan period. Fixture sequences produce the exact `PRESSED`, `LONG_PRESS`, `LONG_PRESS_REPEAT`, release, `CLICK`, `DOUBLE_CLICK`, and `TRIPLE_CLICK` ordering in the contract. Boundary tests at one scan before/after every threshold pass; injected bounce adds no event; timestamps match the emission scans. |
| `VER-HIL-BAT-001` | `ROS-BAT-01`, `HW-BAT-01` | ADC triggering is 50 ms, first-valid initialization and subsequent step response match the 0.05 IIR rule, and the archived D3 record contains the VREFINT/divider calibration used by the binary. At five calibrated supply values spanning 5.0–12.6 V, reported millivolts are monotonic and within ±2%; publication is 1 Hz ±5%. Thresholds 5000, 6300, and 20000 mV pass; adjacent out-of-range values fail. Reapplying the active threshold returns `OK` without changing alarm state or restarting either debounce timer, including at 9 s of a pending assertion and 1 s of a pending clear; an actual threshold change resets both timers without an immediate state change. Alarm asserts only after 10 s below threshold, clears only after 2 s at threshold+200 mV (capped at 20000), repeats the exact 2100 Hz/800 ms/200 ms/five-cycle pattern every 10 s, resumes the retained host pattern afterward, and never changes a motor. Invalid ADC retains alarm/filter state and reports invalid/zero voltage. |
| `VER-SAFE-WDG-001` | `HW-WDG-01` | Each required task heartbeat is stalled independently. Motors stop, the sole watchdog refresh site withholds refresh, reset occurs within the characterized sub-one-second maximum, and after reboot `last_reset_reason` is `RESET_INDEPENDENT_WATCHDOG` with the exact first stalled task. Before that reset, diagnostics continue to show only the prior-boot offender. Inject reset during each retained-record write phase and prove torn data becomes `TASK_NONE`; then perform pin, software, power-on/brownout, and window-watchdog reset cases with an otherwise valid record and prove only IWDG exposes it. Every boot consumes/clears the record, so a following unrelated reset cannot replay it. ELF/map inspection proves the aligned record is exactly 12 bytes in retained `.noinit` SRAM. Maximum healthy load produces zero watchdog reset. |

## Performance and endurance gates

### `VER-LOAD-500-001`: mandatory 500 Hz qualification

Run controller HIL for 3,600 uninterrupted seconds and publish exactly
1,800,000 valid four-motor `MotorCommand` messages. Use a sequence-coded,
electrically safe alternating target so freshness is recoverable from the logic
trace. Concurrently run IMU and motor state at 50 Hz, PWM state at 20 Hz,
battery at 1 Hz, heartbeat at 2 Hz, diagnostics at 1 Hz, at least one button
event per second, PWM commands at 10 Hz, bus motion at 2 Hz on a safe fixture,
LED commands at 1 Hz, one finite buzzer pattern every 10 seconds, RGB1 commands
at 10 Hz, and OLED commands at 0.2 Hz. Exercise all seven services sequentially
once per minute, using current/reversible values and no bus-servo persistence;
schedule the motor-model call during a brief zero-target phase so its successful
path is exercised. All traffic shall remain at or below its endpoint maximum.
Before full motor release qualification, this run shall use the exact verified
default PID image on the guarded/current-limited fixture. A software-only
transport variant may use sequence-coded zero/stop subset commands, but it does
not satisfy the powered motor portions of this case. Passing either
pre-qualification variant does not qualify PID or polarity for release.

Acceptance:

1. Zero crash, deadlock, unexpected or watchdog reset, endpoint loss, RX
   overrun, executor overrun, missed 100 Hz motor release, buffer overrun,
   stale command application, or out-of-range hardware output.
2. For every current-session `MotorControlTask` mailbox consumption that
   applies at least one fresh accepted field, the conservative maximum field
   age is at most 20 ms at p99 and less than 100 ms for every sample.
   `motor_command_consumptions` is the sample count,
   `motor_command_age_over_20_ms` counts strict failures, and
   `motor_command_max_age_us` is the since-boot high-water value. Qualification
   starts after a fresh boot with all three fields zero; otherwise it fails as
   contaminated evidence. The strict-over-threshold delta shall be no greater
   than one percent of the sample-count delta, using nearest-rank p99; the
   high-water shall remain below 100,000 us. Coalescing is counted once using
   the maximum age across its fresh accepted fields; rejected and stale fields
   do not sample, and no older queued command is later applied. The separate
   PID/output update remains 100 Hz.
3. The `MotorControlTask` 1 kHz lease release continues without a missed
   release. When input stops, an explicit zero takes effect by the next 100 Hz
   update; without a final zero, `VER-SAFE-LEASE-001` still stops each motor
   within 200 ms of the last accepted command.
4. While the ACTIVE-session allocation seal is set, there are zero successful
   allocation/deallocation calls and `post_seal_allocation_attempts` remains
   zero. All resource and stack gates in `VER-RESOURCE-001` pass.
5. For every complete one-second window, escaped USART1 RX bytes plus escaped
   USART1 TX bytes are less than 70,000.

### `VER-SOAK-001`: 24-hour mixed-operation soak

Run 24 uninterrupted hours with motors at 50 Hz, PWM commands at 20 Hz, bus
motion up to 10 Hz on a safe fixture, LED patterns at 1 Hz, one finite buzzer
pattern per minute, RGB1 at 30 Hz, OLED at 1 Hz, and all nominal
state/heartbeat/diagnostic rates. Exercise all seven services once per minute
without overlapping bus requests or enabling persistent bus-servo writes.
Any nonzero motor traffic requires the motor HIL prerequisites above and the
exact default PID release candidate on a guarded/current-limited fixture;
otherwise the soak uses zero/stop motor traffic only.

Acceptance is zero crash, reset, deadlock, missed safety/control release,
endpoint loss, stale replay, post-seal allocation attempt, monotonic-counter
regression, unexplained message gap longer than twice the declared period,
resource growth, stack/resource gate violation, or one-second combined traffic
window of 70,000 bytes or greater. Without rebooting after the soak, the complete
functional HIL smoke suite shall pass.

### `VER-TRAFFIC-001`: nominal traffic budget

Run all publishers/subscriptions at nominal rates and one non-overlapping
service per second for ten minutes. In every complete one-second window after
the first ten seconds, the diagnostics and an independent serial counter shall
agree within one XRCE frame and escaped RX plus escaped TX shall be less than
70,000 bytes. The same combined gate applies to `VER-LOAD-500-001` and
`VER-SOAK-001`.

## Fault injection and recovery

| Test ID | Injection and exact acceptance |
|---|---|
| `VER-SAFE-LEASE-001` | Stop motor commands at every phase relative to the 1 kHz TIM7 release. Every addressed nonzero motor reaches zero no earlier than 198 ms and no later than 200 ms from firmware acceptance of its last valid command; the maximum measured interval between completed lease evaluations is 2 ms and unaddressed actuators do not change. |
| `VER-RECONNECT-USB-001` | During sequence-coded 500 Hz traffic, physically disconnect/reconnect USB 100 times, rotating 1 s, 2 s, and 5 s outages with at least 10 s connected between cycles. Each cycle stops motors within 200 ms, holds PWM/bus state, replays no old generation, resets no MCU, restores all 21 endpoints within 5 s of device and Agent availability, and moves only after a fresh command. |
| `VER-RECONNECT-AGENT-001` | Kill/restart the Agent 100 times during mixed traffic and service activity. Each cycle meets the same motor/hold/stale-work/5 s rules; an in-flight host bus call returns a defined failure within 250 ms; the MCU never reboots. |
| `VER-RESET-MCU-001` | Assert NRST or perform a controlled MCU-only power cycle 100 times while sequence-coded commands continue; the bus-servo fixture remains separately powered and its baseline is recorded. Motor pins remain disabled/zero; PWM pins remain low until the frame generator is ready and then output the 1500 microsecond reset default with zero offsets. No bus-servo frame is transmitted implicitly, so its ID, limits, saved or unsaved offset, and torque state remain exactly as the separately powered servo held them. Each transition records its published reset cause. Only `RESET_POWER_ON`, `RESET_PIN`, `RESET_SOFTWARE`, or `RESET_BROWNOUT` counts as the expected operator reset; `RESET_INDEPENDENT_WATCHDOG`, `RESET_WINDOW_WATCHDOG`, `RESET_LOW_POWER`, `RESET_UNKNOWN`, or any unrecognized value fails the campaign and does not satisfy a cycle. No old command is replayed, all 21 endpoints return within 5 s after physical transport and Agent availability, and every cycle completes without manual intervention. Runtime motor model, PID overrides, and battery threshold return to defaults, and PWM offsets reset to zero and are reapplied only by the C++ supervisor. |
| `VER-OVERFLOW-001` | Publish motors at 1,000 Hz plus every other command at maximum for 60 s, then independently stall RX consumption until the 8 KiB ring overruns. Memory stays bounded; merge/latest slots retain only newest state; excess services return `BUSY`/`TIMEOUT`; exact overwrite/overrun counts are visible. RX overrun disarms motors by the next 1 kHz release, tears down the session, and recovers within 5 s; normal latency returns within 1 s after load removal. |
| `VER-OVERFLOW-BTN-001` | Pause the event consumer and inject 32 ordered button events. The internal queue returns events 17–32 in order, drop count rises exactly 16, ROS history remains depth 8, and transport/motor deadlines are met. |
| `VER-FAULT-TX-001` | Keep USART1 DMA and UART TC completion normal and run independent Agent fault phases. In separate recovered sessions, withhold the ACK for one reliable publication and then one service response: each operation returns/fails within its 10 ms XRCE session timeout, reliable history stays within its fixed eight-by-512-byte capacity, and fatal session teardown occurs without waiting for pings. Independent XRCE capture records the attempted operation, withheld ACK, bounded history use, and teardown; no unsupported internal retry/occupancy metric is claimed. In a third session, acknowledge all application traffic normally and drop only ACTIVE ping replies: exactly three consecutive 10 ms failures at the 500 ms cadence trigger the Agent-loss teardown. In every phase `transport_tx_timeouts` is unchanged because physical writes complete, motors are safe, no stale generation is emitted, and all endpoints recover within 5 s after the Agent resumes. |
| `VER-FAULT-TX-002` | Independently suppress TX-DMA completion notification, suppress UART TC completion, and inject a HAL TX error for callback lengths 1, 512, and 1024 bytes. Missing completion reaches the `ceil(10 * length_bytes * 1000 / 1,000,000) + 2` ms deadline—3, 8, and 13 ms respectively—and increments `transport_tx_timeouts` exactly once. HAL error records `IO_ERROR` from `SOURCE_TRANSPORT` and does not increment the timeout counter unless the deadline also expires. Every phase disarms motors by the next 1 kHz release and enters safe teardown. There is no force-unlock, chained ISR transfer, leak, or confusion with missing Agent ACKs; a fresh session recovers within 5 s after fault removal. |
| `VER-FAULT-UART-001` | Independently inject USART1 framing, noise, parity, hardware-overrun, and DMA faults before the motor pre-check, between that check and the arm authority write, between duty-channel writes, between the final duty write and post-check, and immediately after the post-check while the controller critical section remains held. The standard HAL error callback maps each fault to the exact sticky error bit and counter. A pre-existing latch stops and rejects without a write; a fault already visible during arm or the four-channel update is caught by the post-check and every output is stopped before `BUSY` returns. A priority-6 callback delayed by the critical section runs immediately at its exit; after the post-check no write remains, so its emergency stop wins before the next release. Motors disarm by the next 1 kHz release, task-context teardown/reset occurs, the inhibit has no host unlock, and a clean session recovers within 5 s after fault removal through the normal transport-open path. |
| `VER-FAULT-BUS-001` | Inject missing servo, stuck-low/high bus, bad checksum, late reply, wrong ID, and a write failure after each configurable field. Response is `TIMEOUT`/`IO_ERROR` by 200 ms or `PARTIAL` with the exact applied mask; late/wrong data is discarded; the next request succeeds after removal; motor releases/leases never miss. |
| `VER-FAULT-I2C-001` | Inject NACK, held SDA/SCL, and sensor/OLED absence. The owning task exits within its documented heartbeat bound, records the correct source/timeout, retains the previous snapshot/display state, and recovers after release without MCU reset; transport and safety remain live. |
| `VER-SAFE-BAT-001` | Inject voltages around both debounce boundaries, invalid internal-reference/open/short readings, and noise. Exact 10 s assertion, 200 mV/2 s clear, invalid-sample hold, buzzer arbitration, diagnostic, and motor-independence behavior from the ROS contract is observed. |

## Resource gate

### `VER-RESOURCE-001`

This case is evaluated from the release ELF/map and during boundary HIL,
`VER-LOAD-500-001`, `VER-SOAK-001`, and all 100-cycle recovery tests.

| Resource | Hard gate |
|---|---|
| Flash | Used bytes are at most 419,430 (80% rounded down from 512 KiB); at least 104,858 bytes remain. |
| DMA-accessible SRAM | Used bytes are at most 104,857 (80% rounded down from 128 KiB), including every task stack assigned there, pools, and transport buffers; at least 26,215 bytes remain. |
| CCM | Used bytes are at most 52,428 (80% rounded down from 64 KiB), including the 48 KiB micro-ROS arena and 2 KiB motor-task stack; at least 13,108 bytes remain. |
| Task stacks | Minimum unused stack is at least 25%: safety 256 B, motor 512 B, micro-ROS 4,096 B, bus 768 B, sensor 1,024 B, peripheral 1,024 B. No stack overflow or canary corruption occurs. |
| Arena and allocation | The only arena is the statically backed 48 KiB CCM arena. Allocation/finalization/reset activity occurs only in `CREATE_ENTITIES`/`TEARDOWN`. While the ACTIVE seal is set there are zero successful allocation/deallocation calls and zero attempted post-seal allocations. After each recovery, arena usage and every pool availability value equal the pre-cycle baseline; there is no monotonic growth across 100 cycles. |
| Fixed transport storage | RX circular DMA ring is exactly 8 KiB, TX bounce buffer exactly 1 KiB, and XRCE MTU exactly 512 bytes. |
| Middleware/storage bounds | Entity counts are exactly one node, seven publishers, seven subscriptions, seven services, zero clients, and history depth eight. Mailbox, event FIFO, service-slot, message, sequence, and string backing capacities equal the architecture/ROS contract and never grow. |
| Wire traffic | Escaped USART1 RX plus TX is less than 70,000 bytes in every supported measured one-second window. |

The stack allocations are 1 KiB `SafetySupervisorTask`, 2 KiB
`MotorControlTask`, 16 KiB `MicroRosTask`, 3 KiB `BusServoTask`, 4 KiB
`SensorTask`, and 4 KiB `PeripheralTask`.

## Mandatory requirement traceability

Each shall-level requirement has an explicit verification mapping below.

| Requirement | Verification case(s) |
|---|---|
| `SCOPE-001` | `VER-REVIEW-001`, `VER-INT-TRN-001` |
| `SCOPE-002` | `VER-SCOPE-001`, `VER-API-001` |
| `SCOPE-003` | `VER-BUILD-HOST-001`, `VER-SCOPE-001` |
| `SCOPE-004` | `VER-ANALYSIS-001` |
| `SCOPE-005` | `VER-SCOPE-001`, `VER-HIL-MOT-001`, `VER-HIL-PWM-001`, `VER-HIL-BUS-001`, `VER-HIL-LED-001`, `VER-HIL-BUZ-001`, `VER-HIL-RGB-001`, `VER-HIL-OLED-001`, `VER-HIL-IMU-001`, `VER-HIL-BTN-001`, `VER-HIL-BAT-001` |
| `PLAT-001` | `VER-BUILD-HOST-001` |
| `PLAT-002` | `VER-BUILD-FW-001`, `VER-INT-TRN-001` |
| `PLAT-003` | `VER-ANALYSIS-001`, `VER-RESOURCE-001` |
| `PLAT-004` | `VER-BUILD-FW-001` |
| `PLAT-005` | `VER-IMG-UNIFIED-001` |
| `PLAT-006` | `VER-CROSS-HANDOFF-001` |
| `HW-001` | `VER-HIL-MOT-001` |
| `HW-002` | `VER-HIL-PWM-001` |
| `HW-003` | `VER-HIL-BUS-001` |
| `HW-004` | `VER-HIL-LED-001` |
| `HW-005` | `VER-HIL-BUZ-001` |
| `HW-006` | `VER-HIL-RGB-001` |
| `HW-007` | `VER-HIL-OLED-001` |
| `HW-008` | `VER-HIL-IMU-001` |
| `HW-009` | `VER-HIL-BTN-001` |
| `HW-010` | `VER-HIL-BAT-001`, `VER-SAFE-BAT-001` |
| `HOST-001` | `VER-BUILD-HOST-001`, `VER-CONFIG-001` |
| `HOST-002` | `VER-CONFIG-001`, `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001`, `VER-RESET-MCU-001` |
| `HOST-003` | `VER-CONFIG-001`, `VER-UNIT-SVC-001` |
| `HOST-004` | `VER-CONFIG-001`, `VER-SCOPE-001` |
| `HOST-005` | `VER-TRACK-UNIT-001`, `VER-TRACK-INT-001`, `VER-TRACK-RDK-001` |
| `TRN-001` | `VER-INT-TRN-001`, `VER-CONFIG-001` |
| `TRN-002` | `VER-CONFIG-001`, `VER-INT-TRN-001` |
| `TRN-003` | `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001` |
| `TRN-004` | `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001` |
| `TRN-005` | `VER-UNIT-STATE-001`, `VER-RECONNECT-AGENT-001` |
| `TRN-006` | `VER-OVERFLOW-001`, `VER-FAULT-TX-001`, `VER-FAULT-TX-002`, `VER-FAULT-UART-001` |
| `ROS-001` | `VER-API-001` |
| `ROS-002` | `VER-API-001`, `VER-UNIT-QOS-001` |
| `ROS-003` | `VER-SERIALIZATION-001` |
| `ROS-004` | `VER-API-001`, `VER-UNIT-QOS-001`, `VER-UNIT-VAL-001` |
| `ROS-005` | `VER-UNIT-VAL-001`, `VER-UNIT-SVC-001`, `VER-UNIT-DIAG-001` |
| `ROS-006` | `VER-UNIT-TIME-001` |
| `CTRL-001` | `VER-HIL-MOT-001`, `VER-UNIT-MBOX-001` |
| `CTRL-002` | `VER-UNIT-LEASE-001`, `VER-SAFE-LEASE-001` |
| `CTRL-003` | `VER-HIL-MOT-001` |
| `CTRL-004` | `VER-HIL-MOT-001`, `VER-UNIT-SVC-001` |
| `CTRL-005` | `VER-HIL-PWM-001`, `VER-UNIT-VAL-001` |
| `CTRL-006` | `VER-HIL-PWM-001` |
| `CTRL-007` | `VER-HIL-BUS-001`, `VER-UNIT-VAL-001` |
| `CTRL-008` | `VER-HIL-BUS-001` |
| `CTRL-009` | `VER-HIL-BUS-001`, `VER-UNIT-SVC-001` |
| `CTRL-010` | `VER-HIL-LED-001`, `VER-HIL-BUZ-001` |
| `CTRL-011` | `VER-HIL-RGB-001`, `VER-UNIT-MBOX-001` |
| `CTRL-012` | `VER-HIL-OLED-001`, `VER-UNIT-VAL-001` |
| `CTRL-013` | `VER-HIL-BTN-001`, `VER-UNIT-BTN-001` |
| `CTRL-014` | `VER-HIL-BAT-001`, `VER-UNIT-SVC-001` |
| `CTRL-015` | `VER-CONFIG-001`, `VER-RESET-MCU-001`, `VER-HIL-BUS-001` |
| `RT-001` | `VER-HIL-MOT-001`, `VER-LOAD-500-001` |
| `RT-002` | `VER-ANALYSIS-001`, `VER-UNIT-MW-001`, `VER-FAULT-TX-001`, `VER-FAULT-TX-002` |
| `RT-003` | `VER-UNIT-MBOX-001`, `VER-LOAD-500-001` |
| `RT-004` | `VER-LOAD-500-001` |
| `RT-005` | `VER-OVERFLOW-001` |
| `RT-006` | `VER-UNIT-SVC-001`, `VER-FAULT-BUS-001` |
| `RT-007` | `VER-UNIT-SVC-001` |
| `RT-008` | `VER-UNIT-MW-001`, `VER-LOAD-500-001` |
| `SAFE-001` | `VER-UNIT-VAL-001`, `VER-UNIT-LEASE-001` |
| `SAFE-002` | `VER-UNIT-STATE-001`, `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001`, `VER-RESET-MCU-001` |
| `SAFE-003` | `VER-SAFE-WDG-001`, `VER-LOAD-500-001` |
| `SAFE-004` | `VER-HIL-BUS-001`, `VER-FAULT-BUS-001` |
| `SAFE-005` | `VER-HIL-BUS-001`, `VER-UNIT-SVC-001` |
| `SAFE-006` | `VER-BUILD-MOTOR-GATE-001`, `VER-UNIT-MOTOR-GATE-001`, `VER-HOST-COMMISSION-001`, `VER-HIL-MOT-001` |
| `SAFE-007` | `VER-UNIT-MOTOR-GATE-001`, `VER-HOST-COMMISSION-001`, `VER-HIL-MOT-001`, `VER-REVIEW-001` |
| `QUAL-001` | `VER-TRACE-001` |
| `QUAL-002` | `VER-SERIALIZATION-001`, `VER-FUZZ-VAL-001`, `VER-FUZZ-TRN-001`, `VER-UNIT-VAL-001`, `VER-UNIT-SVC-001`, `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001`, `VER-LOAD-500-001` |
| `QUAL-003` | `VER-UNIT-DIAG-001`, `VER-FAULT-TX-001`, `VER-FAULT-TX-002`, `VER-FAULT-UART-001`, `VER-FAULT-BUS-001`, `VER-FAULT-I2C-001`, `VER-SAFE-WDG-001`, `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001` |
| `QUAL-004` | `VER-REVIEW-001`, `VER-TRACE-001` |
| `RES-001` | `VER-RESOURCE-001` |
| `RES-002` | `VER-RESOURCE-001` |
| `RES-003` | `VER-RESOURCE-001`, `VER-LOAD-500-001`, `VER-SOAK-001` |
| `RES-004` | `VER-TRAFFIC-001`, `VER-LOAD-500-001` |
| `MEM-001` | `VER-RESOURCE-001` |
| `MEM-002` | `VER-RESOURCE-001`, `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001` |
| `PERF-001` | `VER-LOAD-500-001` |
| `PERF-002` | `VER-LOAD-500-001` |
| `SOAK-001` | `VER-SOAK-001` |
| `REC-001` | `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001`, `VER-RESET-MCU-001` |
| `REC-002` | `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001`, `VER-RESET-MCU-001` |
| `REC-003` | `VER-UNIT-STATE-001`, `VER-RECONNECT-USB-001`, `VER-RECONNECT-AGENT-001`, `VER-RESET-MCU-001` |

The nonblocking should-level objectives map as follows: `SCOPE-006` to
`VER-SCOPE-001`, `ROS-007` to `VER-API-001` and `VER-UNIT-QOS-001`, and
`QUAL-005` to a documentation review recorded under `VER-TRACE-001`. A deviation
requires the rationale specified by `requirements.md`.

## Release acceptance report

Use the machine-generated campaign directories from
[Tutorial 07](../tutorials/07-run-stress-soak-and-release-gates.md) as the
immutable candidate results. Each output binds revision, environment,
start/end time, result, JSON/CSV metrics, JUnit, session transitions, and
instrument-generated measurements to the candidate. Requirement/audit trace
matrices, resource arithmetic, and nonblocking should-level deviations remain
part of release review. A failed endurance/recovery case shall be rerun in
full; rerunning only the failing interval is not acceptance. A campaign pass
with an absent or `NOT_OBSERVED` physical metric remains incomplete and cannot
support a release claim.
