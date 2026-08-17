# RRCLite v2 Runtime Architecture

Status: normative implementation architecture  
Runtime host: ROS 2 Humble on Ubuntu 22.04, amd64 or arm64

Development host: native Ubuntu 22.04/Humble, or the VS Code Dev Container on
macOS and every other Linux distribution
MCU: STM32F407VET6, STM32 HAL, FreeRTOS, and a C++17 application framework
over the micro-ROS C client library

## System boundary

RRCLite v2 replaces both the Python serial bridge and the MCU proprietary packet
dispatcher. The MCU is a micro-ROS client and exposes the bounded endpoints in
the [ROS interface contract](ros-interface-contract.md) through the separately
installed compiled micro-ROS Agent:

```text
ROS 2 graph
  <-> native non-root micro-ROS Agent systemd service
  <-> /dev/mentor_pi_mcu at 1,000,000 baud
  <-> USB-C -> CH9102F -> USART1
  <-> MicroRosTask
  <-> fixed mailboxes/snapshots/service slots
  <-> single-owner hardware tasks
```

The physical path and excluded alternatives are fixed by the
[hardware baseline](hardware-baseline.md) and
[transport ADR](adr/0001-mcu-ros-transport.md). There is no application-level
framing protocol above Micro XRCE-DDS and no host translation node.

## Language boundary

Project-owned host runtime and the MCU framework/ownership orchestration shall
be C++17 and follow [development-standards.md](development-standards.md).
Individual MCU application or driver modules may use C11 or C++17 behind a
documented boundary to the C-based HAL, FreeRTOS, and micro-ROS libraries; C11
modules do not replace the C++ framework. STM32 HAL, generated ROS type support,
and the micro-ROS client expose C APIs; C++ owners call them through narrow
adapters. Interrupt vectors and RTOS task entry points use thin C-linkage
trampolines and
immediately delegate without storing application state in the trampoline. No
first-party Python process participates in transport, configuration, or
runtime control.

## Ownership rules

1. Exactly one task shall own each peripheral and its mutable driver state.
2. `MicroRosTask` shall be the only task that calls rcl, rmw, rclc, ROS message
   serialization, publisher, service-take, or service-response APIs.
3. An ISR may acknowledge hardware, capture a bounded datum, and notify a task.
   It shall not allocate, parse ROS, wait, invoke application callbacks, free
   buffers, drain queues, or start the next application transaction.
4. ROS callbacks shall validate and copy into fixed internal objects, then
   return. They shall not touch actuator registers or wait for a worker.
5. A safety shutdown may override normal peripheral ownership only through the
   small, idempotent `EmergencyStopMotors()` register-level primitive described
   in [reliability-and-safety.md](reliability-and-safety.md).

These rules remove the legacy races documented in the
[legacy audit](legacy-audit.md), including UART state forcing, ISR-side heap
operations, and blocking servo transactions in the receive path.

## Static FreeRTOS tasks

FreeRTOS shall use native static creation APIs. Priority 0 is idle and
`configMAX_PRIORITIES` shall be at least 8.

| Priority | Task | Static stack | Responsibility and release |
| ---: | --- | ---: | --- |
| 7 | `SafetySupervisorTask` | 1 KiB | Every 20 ms: validate task heartbeats, invoke emergency motor shutdown on a critical stall, and conditionally refresh IWDG. |
| 6 | `MotorControlTask` | 2 KiB | A 1 kHz safety release checks per-motor command leases; every tenth release performs encoder sampling and ADRC at 100 Hz. Sole normal owner of motor state and PWM. |
| 5 | `MicroRosTask` | 16 KiB | Own USART1 transport, allocator lifecycle, rmw/rcl/rclc entities, executor, publications, services, and Agent state. It shall never wait for hardware other than bounded USART1 transport waits. |
| 4 | `BusServoTask` | 3 KiB | Own UART5 half-duplex direction, request/response buffers, timeouts, and all bus-servo movement/configuration/read/stop transactions. |
| 3 | `SensorTask` | 4 KiB | Own QMI8658 acquisition, button debounce/event generation, battery ADC conversion/filtering, and sensor snapshots. |
| 2 | `PeripheralTask` | 4 KiB | Own PWM-servo interpolation, LED and buzzer patterns, RGB SPI DMA, and OLED I2C updates. |

Tasks shall block on notifications or use `vTaskDelayUntil`; none shall busy
poll. HAL and worker waits shall have explicit deadlines. The design does not use
FreeRTOS software-timer callbacks for hardware or ROS work.

TIM7 releases `MotorControlTask` every 1 ms. Qualification shall prove that the
maximum interval between completed lease evaluations, including scheduling
jitter, is no greater than 2 ms. The task expires a nonzero motor lease when its
age is at least 198 ms. This early threshold ensures that even the latest
qualified next evaluation cannot leave motor PWM nonzero beyond 200 ms from
command acceptance.

## Motor build-time safety gate

Motor motion authority is a compile-time property and has no ROS unlock. The
only supported build is the normal ADRC release artifact:

```sh
make -C firmware build
```

It is classified `NORMAL_CLOSED_LOOP_DEFAULT` with
`control_mode=CLOSED_LOOP`. It enforces the active model's RPS limit, the 6 RPS
implementation ceiling, the +/-1000-permille output limit, independent 198 ms
lease expiry, session-loss disarming, and transport-failure shutdown. No build
alias or ROS value selects another control mode. These limits do not make
unguarded motor motion safe: all wheels shall remain raised or on an equivalent
guarded fixture and the board shall use a current-limited supply until the
required HIL evidence is recorded.

Before its first powered command, each channel shall pass a passive encoder
direction test with motor PWM disabled. Firmware uses raw signed encoder delta
directly for every motor model. Targets, measurements, and LADRC state use that
coordinate, while one fixed inversion converts the semantic LADRC output to
physical bridge duty for every channel and model. The host owns the only sign
conversion between the MCU coordinate and positive ROS wheel rotation:
`{-1,+1,-1,+1}` in logical FL,FR,RL,RR order. All
motor ADRC constants and physical model/channel mappings remain provisional
until D3 HIL records qualify or replace them. A later production-motion enable
requires those records and reviewed change control; host configuration success
alone cannot enable it.

`BusServoTask` implements UART5 work as a nonblocking transaction state
machine. No poll/DMA wait slice exceeds 10 ms, and it completes a bounded step
and advances its task heartbeat at least every 20 ms even though an entire bus
service may use its 200 ms deadline. A transaction deadline therefore cannot
look like a stalled task to the watchdog.

Stack values in the table are byte budgets. Static `StackType_t` arrays shall
round those budgets up by `sizeof(StackType_t)` rather than treating the values
as FreeRTOS stack-depth units.

### Interrupt split

- The TIM7 1 kHz motor-timer ISR clears the timer flag and releases
  `MotorControlTask`. Encoder velocity uses signed modular counter deltas, so no
  task-visible multiword overflow accumulator is required.
- The PWM-servo timer ISR toggles pins and copies precomputed shadow compare
  values only. `PeripheralTask` calculates interpolation outside the ISR and
  swaps the shadow set at a frame boundary.
- The USART1 RX/TX DMA streams and USART1 interrupt run at priority 6, at or
  below the FreeRTOS syscall ceiling. DMA and UART IRQs enter the standard
  `HAL_DMA_IRQHandler` and `HAL_UART_IRQHandler` state machines. Standard RX
  half/full callbacks increment a single boundary epoch and notify
  `MicroRosTask`; the task reconstructs producer progress from the epoch and
  DMA `NDTR`. Standard TX-complete and UART-error callbacks perform bounded,
  FreeRTOS-safe completion or fail-closed notification.
- UART5, SPI, ADC, IMU, and the remaining USART1 interrupts publish only flags,
  cursor progress, or task notifications using interrupt-safe APIs.
- Every ISR that calls FreeRTOS shall use an NVIC priority numerically equal to
  or greater than `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`. The motor
  release ranks above USART1; USART1 ranks above UART5 and low-rate peripheral
  interrupts.

## USART1 custom transport

USART1 is dedicated to Micro XRCE-DDS. No logger, shell, `printf`, or other
driver may transmit on it.

### Receive path

- Allocate an 8 KiB, power-of-two circular RX DMA buffer in the linker DMA
  section in SRAM.
- Start RX DMA once in transport `open`. Do not abort and restart it for normal
  IDLE, half-transfer, or transfer-complete events.
- Standard `HAL_DMA_IRQHandler` dispatches the circular stream's half-transfer
  and transfer-complete events without a project-owned DMA flag handler. RX DMA
  remains active in circular mode until transport close or fatal teardown.
- Every standard HAL half/full callback increments a 32-bit boundary epoch and
  notifies `MicroRosTask` using only FreeRTOS-safe `FromISR` operations.
- `MicroRosTask` polls `ring_size - NDTR` at intervals no longer than 1 ms while
  waiting for data, so short XRCE frames do not depend on a half-ring IRQ. The
  task alone advances the consumer position.
- Sample the boundary epoch, DMA `NDTR`, and epoch again until the two epoch
  reads match and its odd/even phase agrees with the cursor half. The
  reconstruction counts a complete 8 KiB lap even when the cursor returns to
  the same offset; cursor-only modulo subtraction is forbidden.
- If producer minus consumer exceeds 8 KiB, latch `rx_overrun`, stop accepting
  the stream, disarm motors, and enter session teardown. Dropping bytes while
  continuing XRCE parsing is forbidden.
- FE, NE, ORE, and PE are mapped by the standard HAL error callback to sticky
  transport error bits, motor disarm, and bounded task-context teardown.

At 1,000,000 baud 8N1, the physical line carries at most 100,000 bytes/s, so an
8 KiB ring represents at least 81.92 ms at line saturation. Qualification still
requires zero overruns; the ring is recovery margin, not a substitute for
bounded execution. Because the priority-6 HAL callbacks can be deferred by a
FreeRTOS critical section, every critical section that masks transport IRQs
shall remain below the 40.96 ms half-ring interval at line saturation. The
1 ms `NDTR` polling path makes sub-half-ring XRCE traffic visible without a UART
IDLE interrupt.

### Transmit path

- Set the XRCE transport MTU to 512 bytes and enable serial stream framing.
- Copy a write into a 1 KiB DMA-safe bounce buffer before starting normal-mode
  TX DMA. This makes transport correctness independent of the caller's memory
  region.
- The TX-complete ISR only records status and notifies `MicroRosTask`.
- For a custom-transport callback of `length_bytes`, the write deadline is
  `ceil(10 * length_bytes * 1000 / 1,000,000) + 2` milliseconds. A timeout,
  HAL error, or attempted callback write larger than the bounce buffer is fatal
  to the current session and enters safe teardown.
- No TX queue, per-frame allocation, ISR-side free, or ISR-side chained DMA is
  permitted.

Transport reads may block on an RX notification for at most 10 ms at a time,
even if the middleware supplies a longer timeout. They then return available
bytes or zero so session health and task heartbeat handling continue.

Every reliable rcl/rmw operation in `ACTIVE`, including a reliable publication
or service response, shall use a 10 ms XRCE session timeout and return control to
`MicroRosTask` within that bound. An acknowledgement timeout or reliable-history
exhaustion is a fatal error for the current session; it is not converted into an
unbounded retry inside the call. Entity creation and destruction use the
separate lifecycle bounds below.

Every `ACTIVE` iteration resets one blocking-operation permit and advances a
persistent three-class scheduler in strict order: service, reliable telemetry,
maintenance. A service response, reliable publication, Agent ping, or active
time-sync attempt must acquire that same permit before entering middleware, so
at most one such operation can begin in a slice. Classes do not borrow an idle
turn; therefore request/publication floods cannot starve ping or time sync, and
maintenance cannot starve services or telemetry. A second permit request is a
firmware invariant violation and causes safe session teardown.

## ROS execution model

`MicroRosTask` initializes one node, `/mentor_pi/controller`, and one
`rclc_executor`. Node options disable `/rosout`; no parameter, parameter-event,
statistics, action, or `/clock` entity is created. It calls
`rclc_executor_spin_some` with a maximum 1 ms wait. The seven subscriptions are
registered in this order:

1. motor command;
2. PWM-servo motion;
3. bus-servo motion;
4. LED command;
5. buzzer command;
6. RGB command;
7. OLED command.

High-rate motion subscriptions use best-effort, volatile, keep-last depth one.
The latest sample replaces stale work before callback execution. Low-rate
peripheral commands use the reliable depth defined by the interface contract,
but still hand off to bounded mailboxes.

### Asynchronous service pump

The seven service servers shall not use synchronous executor callbacks that wait
for hardware. On each service-class ACTIVE iteration, `MicroRosTask` performs
this bounded slice:

1. inspect at most one occupied completion slot, selected by a persistent
   round-robin cursor;
2. if that inspection sends a response, end the service slice;
3. otherwise scan the four request groups nonblockingly from a second
   round-robin cursor and take at most one ready request;
4. copy that request and its `rmw_request_id_t` into static storage, then
   dispatch fixed work to the owning task without waiting for hardware; or send
   `BUSY` when its slot is occupied.

The four completion slots are motor model, PWM offsets, battery threshold, and
the shared bus slot. The four request groups are the same three local services
plus the bus group. Empty `rcl_take_request` probes do not wait. When the shared
bus slot is free, its bounded scan always probes stop first and then alternates
get/configure; while it is occupied, stop/get/configure busy requests use their
own round-robin cursor. Thus continuous traffic in one local service or one
occupied-bus endpoint cannot starve the other groups.

At most one `rcl_send_response` may begin in an ACTIVE iteration. With a 1 ms
task release, an occupied completion slot is inspected at least once per four
service-class slices, and a continuously ready request group is admitted at
least once per four request-admission opportunities. This fixed scheduling
delay is included when measuring the service deadlines below; it never creates
another request FIFO in firmware.

Motor-model, PWM-offset, and battery-threshold services have 50 ms worker
deadlines. Bus get/configure/stop share one pending slot because UART5 is serial;
their MCU deadline is 200 ms and the host service timeout is 250 ms. Every
request carries a generation value so completion after a timeout or reconnect
is discarded rather than applied to a reused slot.

Stop priority is an admission and dispatch priority among services; it does not
preempt an accepted service. When the shared bus slot is idle, a pending stop is
admitted before get/configure. Once a bus service has been admitted, its
transaction is nonpreemptive; every additional bus request, including stop,
receives `BUSY` and its client must retry. An accepted stop does preempt move
traffic: `BusServoTask` finishes the UART5 frame currently in progress, abandons
all unsent frames from the active move batch, and starts the stop frames at the
next frame boundary. At stop acceptance, `MicroRosTask` records the current
bus-motion generation as a stop watermark and invalidates the pending mailbox;
`BusServoTask` rejects any active or pending move at or below that watermark.
Only a command accepted after the stop has a newer generation and may run after
the stop completes. Stop never truncates a frame or interrupts an accepted
get/configure transaction. A bus service may return partial field results, but
shall never block the executor. Exact result codes and field masks are in the
[ROS interface contract](ros-interface-contract.md).

Bus-servo configuration crosses an explicit persistence boundary in the servo,
not in MCU flash. Selecting `SET_ID`, `SET_POSITION_LIMITS`,
`SET_VOLTAGE_LIMITS`, or `SET_TEMPERATURE_LIMIT` authorizes the corresponding
nonvolatile servo write. `SET_OFFSET` changes the runtime offset only;
`SAVE_OFFSET` explicitly commits the servo's current offset to nonvolatile
storage. `SET_TORQUE` is runtime-only. MCU reset, Agent reconnect, and supervisor
startup neither replay nor revert any of those values. A `PARTIAL` result means
that every bit in `applied_mask`, including a nonvolatile bit, may already have
taken effect and must not be silently rolled back.

## Internal communication

ROS messages are converted to fixed internal structs before handoff. A worker
never retains a pointer into an executor message.

For an interface with an update mask, `MicroRosTask` merges only the selected
fields of each taken, validated sample into a fixed complete command shadow and
advances its generation. It retains unselected fields and their per-field
receive ticks. The owner consumes that complete shadow through a sequence
counter or atomic index, so two adjacent accepted callbacks for disjoint
channels cannot erase one another even if the worker has not run between them.
Best-effort middleware may discard a sample before its callback; no mailbox can
merge a sample the MCU did not receive. Commands without subset semantics
replace their entire shadow.

That merge scope is one ROS session, never the lifetime of the MCU. Every
command gateway first snapshots the active session generation, then rechecks
both `ACTIVE` and the same generation inside the controller critical section.
A callback which overlaps deactivation or replacement by a fresh session
returns `BUSY` and publishes nothing. Before the first motor, PWM, LED, RGB, or
OLED callback admitted for a different session is validated/published, the
gateway drains any unread old snapshot and clears every per-field-valid
generation while preserving the mailbox's monotonic generation counter.
Subsequent disjoint updates merge only with fields accepted in that same
session. The gateway's producer-side discard and the corresponding worker's
`ConsumeLatest` both execute under the same controller critical section, so a
session reset has strict discard-before-publish ordering without using the
mailbox's consumer operation from `MicroRosTask`.

Motor, PWM, and LED workers ignore fields whose per-field generation is zero.
The RGB worker consumes the host-owned RGB2 generation and merges it with the
firmware-owned RGB1 heartbeat/RX/TX status color; the discrete-output worker
overrides LED1 with the time-based system heartbeat. The OLED worker consumes per-field
generations and merges selected new-session fields onto the last successfully
rendered state. Therefore an already applied host output holds across
reconnect, while an old unread or merely pending field is never relabelled as
new-session work. Bus and buzzer shadows are whole-command objects; their
session tags make an unread old object ineligible without any partial-state
merge.

| Object | Capacity | Full behavior |
| --- | ---: | --- |
| `motor_command_mailbox` | 1 complete four-motor shadow plus per-field validity | Atomically merge selected motors within one session; newest selected value and receive tick win. |
| `pwm_command_mailbox` | 1 complete four-channel shadow plus per-field validity | Atomically merge selected channels within one session; newest selected value wins. |
| `bus_motion_mailbox` | 1 command containing at most 16 IDs/positions | Atomic overwrite. |
| `led_command_mailbox` | 1 complete LED state plus per-field validity | Merge selected LEDs within one session, then atomically publish the shadow. |
| `buzzer_command_mailbox` | 1 pattern | Atomic overwrite. |
| `battery_alarm_shadow` | 1 alarm request/generation | `SensorTask` replaces it after debounce/repeat decisions; `PeripheralTask` consumes it and remains the sole buzzer-hardware owner. |
| `rgb_command_mailbox` | 1 fixed two-pixel wire-shaped state; only RGB2 has a valid host generation | Replace host RGB2 within one session; `PeripheralTask` composes firmware status into RGB1 immediately before bounded SPI DMA. |
| `oled_command_mailbox` | 1 two-line state plus per-field validity | Merge selected lines within one session; the owner preserves already rendered unselected lines. |
| `button_event_queue` | 16 events | Remove the oldest, insert the newest, increment `button_event_drop_count`. |
| Non-bus service slots | 1 per service | Respond `BUSY` to an additional request. |
| Shared bus service slot | 1 across get/configure/stop | Respond `BUSY` to every additional request, including stop; an accepted service is nonpreemptive. When idle, admit stop before get/configure. |

For LED and OLED commands, “complete state” means the complete post-merge
shadow: selected elements take their new values and every unselected LED or
line remains unchanged. RGB commands replace only RGB2; RGB1 is never copied
from a ROS command. A buzzer command has no subset mask and replaces its whole
pattern. A command rejected by validation, lost before its callback, or
refused because of overload changes no discrete hardware output.

Latest telemetry uses a single-writer snapshot with a sequence counter or a
two-buffer atomic index. Motor, PWM, IMU, and battery producers never wait for
ROS. `MicroRosTask` publishes motor and IMU snapshots at 50 Hz, PWM state at
20 Hz, heartbeat at 2 Hz, and battery and diagnostics at 1 Hz. It drains button
events without exceeding the reliable history in the interface contract. These
public rates do not change the independent 100 Hz motor-control loop. Every
ACTIVE iteration may publish at most one due best-effort motor/PWM/IMU snapshot,
selected by a persistent round-robin cursor. A reliable-telemetry-class
iteration may additionally begin at most one due button/battery/heartbeat/
diagnostics publication, selected by a separate round-robin cursor. Spreading
simultaneously due state does not change the source rates and prevents three
consecutive physical TX waits in one slice.

The closed-loop motor calculation is first-order linear ADRC over filtered
measured RPS. With observer error `e = z1 - measured_rps`, each 100 Hz update
uses the previously applied post-floor motor output:

```text
z1 += dt * (z2 + b0 * applied_output - 2 * wo * e)
z2 += dt * (-wo * wo * e)
output = (wc * (target_rps - z1) - z2) / b0
```

`b0` is the input gain in RPS/s/permille, `wc` is controller bandwidth, and
`wo` is observer bandwidth. The output is clamped to +/-1000 permille. The
minimum-drive floor is zero, so the calculated nonzero magnitude is not raised
to a fixed duty. ADRC state is owned exclusively by `MotorControlTask`; ROS callbacks
can only submit validated parameter updates to the motor owner. Non-finite
state or `wo * dt > 0.5` fails closed.

Every overwrite, drop, invalid command, busy response, timeout, UART error, DMA
overrun, reconnect, missed release, and high-water mark shall be counted.

## Session lifecycle

The state machine is:

```text
SAFE_BOOT -> WAIT_AGENT -> CREATE_ENTITIES -> ACTIVE
                 ^                              |
                 +---- BACKOFF <- TEARDOWN <----+
```

### `SAFE_BOOT`

Initialize clocks and GPIO with all motor compares zero and motor outputs
disabled. Create all static RTOS objects and start hardware owners. Configure
the custom transport, but do not arm motors. The default ADRC image retains its
compile-time model limit, implementation ceiling, and output clamp after
entering `ACTIVE`; a session transition does not change build authority.
PWM-servo GPIO remains low until
`PeripheralTask` starts the validated TIM13 frame generator; it then produces
the documented 1500 microsecond reset default without consulting a ROS
mailbox. `PeripheralTask` also establishes the reset defaults of LEDs off,
buzzer off, both RGB pixels initially off, and empty host OLED lines. LED1 then
blinks as the 1 Hz system heartbeat, while RGB1 reports successful micro-ROS
heartbeats and RX/TX activity according to the
[verified board profile](verified-hardware-profile.md). `BusServoTask` sends no
frame merely because it started.

A one-way startup motor inhibit remains set until `SafetySupervisorTask`
observes a first on-time heartbeat from every required worker. Motor arm and
duty authority checks include this inhibit under the controller critical
section. It may retain the first session's latest motor command, but cannot
consume or apply it before readiness; clearing the inhibit preserves that
session generation and requires no reconnect.

### `WAIT_AGENT`

Attempt one Agent ping with a 20 ms timeout. Failed attempts back off for 100,
200, 400, 800, 1600, then at most 2000 ms. Tasks and watchdog supervision remain
alive while disconnected. `MicroRosTask` implements each backoff as bounded
wait slices of at most 10 ms so it continues publishing its task heartbeat; it
shall not sleep once for the full backoff interval.

### `CREATE_ENTITIES`

Reset the micro-ROS arena, initialize support/node/publishers/subscriptions/
services/executor, and preassign all bounded string buffers. Make one bounded
initial time-synchronization attempt of at most 20 ms, then seal allocation.
Failure of time synchronization is nonfatal: enter `ACTIVE` in `DEGRADED` with
zero ROS stamps and retry there. Track construction bits so any entity failure
can finalize only constructed objects in reverse order. Motors remain disarmed.

Entity construction is an incremental state machine. Each support, node,
publisher, subscription, service, or executor creation call has a 40 ms maximum
XRCE/middleware timeout. Each `RunOnce()` slice starts at most one middleware
boundary, returns to `MicroRosTask`, and advances its heartbeat before the next
boundary. The fixed graph is created in 49 ordered boundaries, including the
bounded initial time-sync attempt. The complete `CREATE_ENTITIES` phase has a
2 s deadline. A per-call failure or the phase deadline enters bounded teardown;
it shall not wait indefinitely for an unavailable Agent.

### `ACTIVE`

Spin at the bounded interval, run the selected service/reliable-telemetry/
maintenance class, and publish at most one due best-effort snapshot. On a
maintenance-class turn, ping the Agent if its 500 ms period is due; otherwise
perform time synchronization if due. Each uses the common 10 ms ACTIVE permit,
and a due ping takes priority for that turn. Three consecutive ping failures, a
fatal rcl/rmw return, any USART1 FE/NE/ORE/PE error, RX overrun, or TX timeout
moves immediately to `TEARDOWN`. Until the first successful time sync, make a
bounded 10 ms retry every 5 s and remain `DEGRADED`; after success, resynchronize
at least once every 60 s. A failed periodic resync retains the last valid epoch
offset, records an error, and retries after 5 s without affecting monotonic
control time. The independent 200 ms motor lease remains the faster protection
when commands stop without a detectable session failure.

### `TEARDOWN` and `BACKOFF`

Disarm all motors first. Invalidate pending ROS command and service generations
without altering the PWM-servo or bus-servo owner's active target/torque state.
Finalize ROS objects in reverse order, close/reset USART1 in task context, and
reset the allocation arena. Never replay a mailbox from the old session. Return
to the same capped Agent backoff without resetting the MCU for a recoverable
transport/session fault.

The mailbox rule is enforced lazily at the first admitted callback of the next
session, under the same controller critical section used for its session check.
This keeps fixed storage and same-session subset coalescing while ensuring an
old unread field cannot be retagged by a disjoint new-session callback.

The controller publishes the inactive session state before its physical stop,
with both operations in one critical section shared with normal motor arm and
duty writes. USART1/DMA fatal errors additionally remain latched across
teardown. Target arm and duty hooks check a pre-existing latch before and after
their writes. If a priority-6 HAL callback is delayed by that critical section,
it runs immediately after the section exits, latches the fault, and
emergency-stops every channel before any later motor release can write again.
The arm post-check follows its authority-mask write and the duty post-check
follows the complete four-channel update, preserving the defense against a
fault already visible to the task. Only the next normal transport `open` clears
the latch; there is no host-accessible force-unlock path.

The same fail-closed transition runs at the point where `MotorControlTask`
detects an encoder, arm, or duty failure, rather than waiting for the next
20 ms supervisor release. Recoverable transport inhibition requires a new
session generation. Controller-output faults and a supervisor-withheld
watchdog remain inhibited until reset and safe boot, even if micro-ROS creates
another generation before IWDG expires.

Session generations form a nonzero, wrap-aware monotonic watermark. Only an
exact active-generation repeat is idempotent. A different active value cannot
replace a live owner; after explicit deactivation only a strictly newer value
may activate. A stale deactivation still stops outputs but does not lower the
watermark.

Each remote entity-destroy/finalizer call has a 10 ms middleware timeout, and
the complete remote-destruction phase has a 500 ms deadline. Teardown uses the
exact reverse construction order and each `RunOnce()` slice starts at most one
middleware boundary. Before starting a finalizer, the runtime reserves that
call's complete 10 ms budget inside the 500 ms phase. If it no longer fits, the
next slice first sets the context destroy timeout to zero as its one boundary;
remaining finalizers then run without remote waits. If the 500 ms deadline is
already reached, the same timeout-zero boundary is inserted immediately.
`MicroRosTask` advances its heartbeat between boundaries. A finalizer failure
is recorded but does not stop reverse local cleanup. Local construction-bit
cleanup, USART1 close/reset, arena restoration, and transition to `BACKOFF`
therefore continue even when the Agent is absent.

The arena allocator has three explicit modes. `CREATE_ENTITIES` permits
bounded allocate/reallocate/deallocate calls within the reset arena. The seal
is active only in `ACTIVE`, where every allocator entry point is rejected and
counted. After motors are disarmed and all ROS execution is quiescent,
`TEARDOWN` clears the seal and permits deallocation calls made by `*_fini`
only; new allocation or reallocation remains a fault. The final arena reset
shall restore the exact pre-creation offset and pool availability before
`BACKOFF`.

An allocator call while sealed is a firmware-invariant violation, not an
ordinary reconnect condition. It immediately stops motors and latches the
fault; `MicroRosTask` performs only best-effort teardown after clearing the seal,
and `SafetySupervisorTask` withholds IWDG refresh so the controller resets into
safe boot. Normal session recreation is blocked until that reset.

## Static resource configuration

All tasks, queues, semaphores, messages, transport buffers, and service slots
shall be statically declared. FreeRTOS dynamic allocation is disabled. The ROS
allocator is a resettable 48 KiB arena in CCM used only for bounded entity
creation and destruction as defined above; allocator use while `ACTIVE` is a
fatal fault. The CPU-only 2 KiB `MotorControlTask` stack is also placed in CCM;
all other task stacks remain in normal SRAM. Message arrays are fixed; the two
OLED strings use assigned 24-byte backing buffers and never allocate while
active. A 12-byte, four-byte-aligned watchdog-offender record is the only
first-party `.noinit` object; the linker retains it in normal SRAM and startup
code neither initializes nor clears it.

Generate the Humble micro-ROS library with these exact maxima:

| Resource | Limit |
| --- | ---: |
| Nodes | 1 |
| Publishers | 7 |
| Subscriptions | 7 |
| Services | 6 |
| Clients | 0 |
| RMW maximum history | 8 |
| Reliable stream history | 8 |
| XRCE transport MTU | 512 bytes |

Disable unused UDP, TCP, and discovery client profiles. Pin the upstream Humble
source revision used to generate the static library. Select only
`rosidl_typesupport_microxrcedds_c` and globally set
`ROSIDL_GENERATOR_C_DISABLE_TYPE_DESCRIPTION_CODEGEN=ON` for the MCU static
build. Runtime type-description construction is not used by this static wire
path; omitting it prevents desktop metadata tables from consuming MCU SRAM.
The host package remains generated normally from the same IDL.

Build and qualification gates are:

- flash use at most 80% of 512 KiB;
- SRAM use at most 80% of 128 KiB, including all stacks and DMA buffers;
- CCM use at most 80% of 64 KiB;
- at least 25% measured stack headroom for every task under full stress;
- zero allocation/reallocation calls after the entity-creation seal;
- worst-case measured combined RX and TX traffic in every complete one-second
  window less than 70,000 bytes under the supported 500 Hz stress profile,
  leaving at least 30% of the 1 Mbps link for framing bursts, services, and
  reliable retries.

A gate failure stops implementation or requires a reviewed architecture/
interface change. It shall not be resolved by silently increasing a queue,
dropping a retained endpoint, or enabling heap allocation.

## Host deployment

### C++ configuration supervisor

The host application shall include an `rclcpp` configuration supervisor. It is
not a serial bridge and does not translate the public interface: the native
Agent remains the only process that opens `/dev/mentor_pi_mcu`, and application
nodes still communicate directly with the MCU endpoints.

The supervisor node is `/mentor_pi/configuration_supervisor` and consumes a ROS
parameter YAML file with this exact schema:

```yaml
/mentor_pi/configuration_supervisor:
  ros__parameters:
    motor_model: "JGA27"
    input_gain_rps_per_second_per_permille: [0.03, 0.03, 0.03, 0.03]
    controller_bandwidth_rad_s: [4.0, 4.0, 4.0, 4.0]
    observer_bandwidth_rad_s: [12.0, 12.0, 12.0, 12.0]
    velocity_filter_new_weight: [0.5, 0.5, 0.5, 0.5]
    pwm_servo_offsets_us: [0, 0, 0, 0]
    battery_low_threshold_mv: 6300
```

`motor_model` is exactly `JGB520`, `JGB37`, `JGA27`, or `JGB528`;
`input_gain_rps_per_second_per_permille` contains exactly four finite positive
doubles no greater than 1000. `controller_bandwidth_rad_s` and
`observer_bandwidth_rad_s` each contain exactly four finite positive doubles no
greater than 50, and each controller bandwidth shall not exceed its matching
observer bandwidth. `velocity_filter_new_weight` contains exactly four finite
doubles from 0 through 1. All four arrays use firmware connector order M1, M2,
M3, M4;
`pwm_servo_offsets_us` contains exactly four integers from -100 through +100 in
connector order; and `battery_low_threshold_mv` is 5000 through 20000. Unknown
keys, wrong types/counts, and out-of-range values are startup errors. The Agent
device path is a separate bring-up/Agent argument and is not opened by this
node.

The supervisor owns a host-local motion-enable gate. The gate starts false, and
invalid deployment YAML prevents activation with a precise diagnostic. It
treats first discovery, graph or heartbeat reappearance, an
`agent_session_id` change, or an uptime regression as a new session. Uptime is
compared modulo `uint32`: define `delta = uint32(new_uptime - old_uptime)` for
consecutive samples. A delta from 0 through `0x7fffffff` is equal/forward
progress, including `UINT32_MAX` to zero wrap; only a delta from `0x80000000`
through `0xffffffff` is a regression. On each new session event the supervisor
clears the gate, increments a host-local configuration generation, waits for
heartbeat state `READY` or `DEGRADED`, and idempotently applies the validated
deployment configuration in this order:

1. call `motors/set_model`;
2. call `motors/set_adrc` with `ALL_MOTORS` and all configured ADRC and filter
   arrays;
3. require its `OK` response to report `applied_mask == ALL_MOTORS`;
4. call `pwm_servos/set_offsets`;
5. call `battery/set_low_threshold`;
6. set the host-local motion-enable gate true only after all four calls are
   contract-consistent.

The supervisor publishes that state with transient-local reliable QoS on both
`/mentor_pi/configuration/motion_enabled` (`std_msgs/msg/Bool`) and
`/mentor_pi/configuration/motion_authorization` (`std_msgs/msg/UInt64`). The
authorization value is zero while disabled. While enabled, its upper 32 bits
are the low 32 bits of the host-local configuration generation and its lower 32
bits are `agent_session_id`. The supervisor is the only permitted publisher of
the authorization topic. The guarded motor utility requires exactly
one discovered publisher with node identity `/mentor_pi/configuration_supervisor`
and locks the nonzero generation/session pair for the complete run; publisher
loss, duplication, identity mismatch, or token change initiates the stop phase.
This token prevents a retained Boolean from a different publisher or session
from authorizing a new guarded run; it is not MCU motor authority.

The supervisor republishes the current authorization after every received
heartbeat. Project-owned hardware adapters locally invalidate their accepted
token on first heartbeat discovery, session change, non-ready state, or
wrap-aware heartbeat sequence/uptime regression and require a token observed
after that event. Consequently, a retained token cannot authorize a new MCU
boot whose first `agent_session_id` collides with an earlier boot.

Project-owned host motion publishers shall not publish while this gate is
false. A true host-local gate means only that the four configuration services
completed; it neither overrides the firmware motor limits nor qualifies
ADRC/polarity. The MCU independently applies its compile-time authority and
requires a fresh valid command, so the host gate is an additional sequencing
rule, not the motor safety mechanism.

Each supervisor service attempt has a 100 ms host response timeout. For either
`BUSY` or `TIMEOUT`, it makes at most four attempts total: the initial attempt,
then retries after 100, 200, and 400 ms. Exhausting the fourth attempt, or
receiving any other non-`OK` result, keeps the gate false until a later session
event or operator action restarts the configuration sequence. Each future is
tagged with both the host-local configuration generation and the observed
`agent_session_id`; a response received after its timeout or with a generation
or Agent session other than the current pair is discarded and cannot advance
configuration or enable motion. The supervisor exposes or logs its pending,
applied, or rejected state.
It reapplies all four values after an Agent-only reconnect as well as an MCU
reset; values read from a previous session are not assumed to remain effective.
Reapplying an already-active battery threshold shall be an idempotent no-op that
does not restart battery-alarm debounce state.

The supervisor shall not automatically modify bus-servo EEPROM/configuration or
replay motor, PWM-servo, bus-servo, LED, buzzer, RGB, or OLED commands. MCU
heartbeat state `READY` means that the MCU session and peripherals are ready;
it does not mean host deployment configuration has been applied. Only the
supervisor's host-local state represents that condition. The supervisor and any
project-owned host controller code shall be C++ and follow
[development-standards.md](development-standards.md).

### Agent device and service

Render a udev rule only after measuring a unique CH9102F serial number or
physical `ID_PATH`. Production uses a dedicated group rather than the broad
interactive `dialout` group:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", \
  ATTRS{serial}=="<verified-unique-serial>", \
  SYMLINK+="mentor_pi_mcu", GROUP="mentor-pi-serial", MODE="0660", \
  ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_PORT_IGNORE}="1", TAG+="systemd"
```

Do not assume `ttyACM0`; a CH9102F normally binds as a USB serial adapter. If
the adapter has no unique serial, match its verified physical `ID_PATH` instead.
The production installer shall reject a generic vendor/product-only rule and
shall observe exactly one connected adapter matching the selected identity
before installation or upgrade.

A single hardened systemd unit shall execute the versioned native Agent with
the deployment's authoritative `ROS_DOMAIN_ID`. It runs as the unprivileged
`mentor-pi` user with only the `mentor-pi-serial` supplementary group. The
unit retains `ProtectClock=true` and explicitly grants `char-ttyACM` read/write
access so the clock protection's implicit device allowlist does not block the
serial transport. Unix ownership and mode still restrict the selected device
to `mentor-pi-serial`. The service launcher receives these arguments and
passes them unchanged to the compiled Agent:

```sh
/opt/mentor_pi/agent/current/bin/mentor-pi-agent serial \
  --dev /dev/mentor_pi_mcu --baudrate 1000000 -v4
```

On serial initialization, that opt-in makes the Agent use its sole open file
descriptor to set RTS and clear DTR, wait 100 ms, then clear RTS and wait 100
ms before configuring and flushing the port. This converts the CH9102F
auto-download circuit's first-open line state into a deterministic normal-boot
reset. Any modem-control operation failure aborts Agent initialization. The
Agent remains the sole serial reader/writer; this reset handling neither uses
hardware flow control nor changes the 1,000,000-baud 8N1 application link.

The Agent service shall not start the configuration supervisor or any ROS
application. Applications start manually from `ros2_ws`; their Python launch
descriptions are build/orchestration code outside the first-party data path.
Use `Restart=always`; systemd owns Agent reconnect/restart and stop behavior.
No `chmod 777`, broad `dialout` grant, Python serial bridge, or second process
may open the port.
Deployment and stress
tests are specified in
[verification.md](verification.md); language and build rules are in
[development-standards.md](development-standards.md).

Production deployment is native Ubuntu 22.04/Humble on amd64 or arm64. Native
component builds and tests are supported only there. macOS and other Linux
distributions use the VS Code Dev Container for development builds/tests only;
native macOS Agent deployment and Dev Container runtime/device pass-through
are unsupported. Firmware flashing remains a physical-host operation.
