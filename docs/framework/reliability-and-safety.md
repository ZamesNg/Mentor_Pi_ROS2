# RRCLite v2 Reliability and Safety Contract

Status: normative failure behavior  
Scope: firmware, micro-ROS session, host Agent, and retained actuators

## Safety objective

Loss, overload, malformed input, or a recoverable software fault shall not leave
an encoder motor running indefinitely and shall not make overload unbounded.
The controller shall recover communication without replaying stale actuator
commands. This is a product safety mechanism, not a claim of certification for
human-rated or functional-safety use.

This contract refines the [requirements](requirements.md) and is implemented by
the [runtime architecture](architecture.md) on the wiring in the
[hardware baseline](hardware-baseline.md). Exact command ranges and result codes
are defined in the [ROS interface contract](ros-interface-contract.md). Required
tests are in [verification.md](verification.md).

## Safe startup and authority

Before the scheduler starts, firmware shall configure every motor PWM compare
to zero and keep drive outputs disabled. `MotorControlTask` is the sole normal
writer of motor PWM. `SafetySupervisorTask` is the only additional writer and
may call `EmergencyStopMotors()`, which shall:

1. be callable repeatedly without side effects;
2. directly set all eight motor drive compares to zero and disable their output
   enables;
3. require no mutex, queue, allocation, HAL delay, or functioning ROS session;
4. clear the software arm state before normal control can resume.

A startup motor inhibit shall remain set from safe boot until every required
worker has published an on-time first heartbeat and `SafetySupervisorTask` has
observed that complete ready set. While inhibited, `MotorControlTask` shall
neither arm a channel nor invoke the four-channel duty hook. The latest motor
mailbox value may remain retained, and ROS entity creation may establish the
first session generation; clearing the inhibit neither changes that generation
nor requires reconnect. This makes the bounded startup grace a continuous safe
state rather than a motor stop sampled only every 20 ms.

The normal firmware image is motor-locked. In `ACTIVE` it accepts a valid
selected zero target as a stop command, but a command containing any selected
nonzero target is rejected atomically as `UNSUPPORTED`, changes no target, and
refreshes no motion lease. No ROS service, host parameter, Agent state, or true
`/mentor_pi/configuration/motion_enabled` value can remove this build-time lock.

A nonzero target may arm only in an explicitly built commissioning image, in
the `ACTIVE` session state, after a fresh valid command for that motor. The
supported public command is
`make firmware-commissioning COMMISSIONING_BUILD_ACK=MOTORS_RAISED`; it supplies the
two exact internal CMake gates and fails closed otherwise. The image
rejects magnitudes above 0.25 RPS and caps absolute output at 300 permille.
Entity creation, an Agent ping, or a command retained from an old session is not
authority to arm. The commissioning image shall be used only with every wheel
raised or equivalently guarded and a current-limited supply.

The project-owned host commissioning utility adds a fail-closed operational
guard around that firmware authority. It accepts targets only from 0.01 through
0.25 RPS, requires the sole publisher of the session-bound host authorization
token to be `/mentor_pi/configuration_supervisor`, and locks that token before
the first zero burst. It publishes all four fields every 50 ms and aborts if a
drive interval exceeds 100 ms, so a command cannot arrive after the 198 ms MCU
expiry boundary and re-arm as though the run were continuous. It also aborts
on MCU uptime/sequence regression, changed watchdog/lease counters, a nonzero
watchdog mask, selected speed above 0.50 RPS, wrong selected response direction,
or unselected response above 0.02 RPS or two encoder ticks. A pass requires a
correctly directed selected response of at least the greater of 0.002 RPS or
10% of the target and at least two correctly directed encoder ticks.

The utility sends repeated all-motor zero commands before and after motion.
Post-stop success requires the latest state to report all four targets as zero
and every `abs(measured_rps)` strictly below 0.002 RPS. A target that returns
after the zero-target latch was first observed invalidates that earlier latch;
continued coasting or driven motion cannot pass merely because targets are
zero. `SIGINT` and `SIGTERM` enter the bounded stop phase; an exception or
abnormal ROS shutdown requests an immediate best-effort all-motor zero. If the
host can no longer publish, the independent MCU lease remains the stop
authority and must still satisfy the 200 ms bound.
These host checks reduce commissioning risk but neither unlock a normal image
nor replace MCU safety or HIL qualification.

Before any powered command, the normal locked image shall be used for a passive
manual encoder-direction check while all bridge outputs remain disabled. JGA27
currently applies a provisional model polarity factor of `-1` derived from
legacy negative-gain evidence. That factor and every PID profile remain
unqualified until motor HIL measures and records the physical result; the
commissioning build and its caps are not release qualification.

Other reset defaults are deterministic: PWM-servo GPIO is low until the frame
generator is ready and then each channel outputs 1500 microseconds with zero
offset; all three LEDs and the buzzer are off; both RGB pixels are driven off;
both host OLED lines are empty; and no bus-servo frame is sent. None of these
defaults is restored from an earlier ROS session.

## Per-motor command lease

Each of the four motors has independent `last_valid_command_tick`, target, and
armed state.

- The lease duration is 200 ms.
- A 1 kHz safety release in `MotorControlTask` evaluates the lease; PID and
  encoder updates remain 100 Hz.
- TIM7 releases the check every 1 ms. Qualification shall prove that the
  maximum interval between completed lease evaluations, including scheduling
  jitter, is no greater than 2 ms. The expiry threshold is 198 ms, allowing one
  qualified worst-case interval while ensuring motor PWM is zero no later than
  the acceptance timestamp plus 200 ms.
- Only a message that passes complete structural validation may update any
  lease. Within it, only motors selected by `update_mask` refresh their own
  ticks.
- A zero mask, invalid ID/mask, NaN, infinity, invalid model, malformed message,
  or rejected value refreshes no lease.
- A finite speed outside the active motor model's documented range, or outside
  the commissioning image's stricter 0.25 RPS cap, rejects the complete message
  atomically and refreshes no lease. It is never clamped.
- When age reaches 198 ms, that motor's target and PWM become zero, its PID
  integrator and accumulated output are cleared, and it becomes disarmed.
- Session failure, transport failure, RX overrun, or safety-supervisor action
  invalidates all four leases immediately rather than waiting for individual
  expiry.

Qualification measures the interval from firmware acceptance of the last valid
per-motor command to zero PWM. The timer release remains 1 kHz, and the measured
inter-evaluation bound is 2 ms. Missing the 200 ms stop boundary is a
release-blocking failure.

Firmware acceptance is stamped directly from the same modulo-`uint32`
microsecond monotonic clock used by `MotorControlTask` for mailbox age and lease
evaluation. The callback path shall not round a millisecond timestamp to
microseconds or use the ROS epoch clock.

## Servo behavior on communication loss

An ordinary Agent loss or expired motor lease shall not issue an implicit PWM-
servo movement, bus-servo torque release, or bus-servo stop. PWM and bus servos
hold the last valid target and bus torque state. This avoids an uncontrolled
mechanical drop when maintaining position is safer than releasing it.

The hold rule applies to communication loss, not loss of board power or MCU
reset. After MCU reset, PWM outputs use the documented 1500 microsecond default
rather than replaying the previous session; a bus servo receives no implicit
move, stop, or torque command. Applications that require a servo release shall
call the acknowledged `bus_servos/configure` service with the torque field
selected before shutdown and verify its result. `bus_servos/stop` stops motion
but does not imply torque release. A future change to automatic servo release
requires a new safety analysis, updated requirements, interface contract, and
verification.

## Input validation and atomicity

Every command shall be validated before it reaches an owner task:

- fixed count is within its interface capacity;
- masks contain only defined bits and IDs map to populated channels;
- all numeric values are finite and within the contract range; out-of-range
  command values are rejected rather than clamped;
- durations, frequencies, thresholds, positions, offsets, and text lengths are
  bounded;
- bus configuration is internally consistent, including present/target ID;
- unused fixed-array elements are ignored and cannot affect hardware.

A multi-device motion command is accepted atomically after validation. A bad
element rejects the whole message; it shall not partially refresh leases or
partially overwrite a mailbox.

Bus configuration is different because physical writes can fail after earlier
writes succeeded. `BusServoTask` validates the whole request first, applies the
documented field order with ID last, and returns `applied_mask`, effective ID,
and `PARTIAL`, `TIMEOUT`, or `IO_ERROR` when appropriate. It shall never report
full success merely because the request was queued.

Selecting bus-servo ID, position-limit, voltage-limit, or temperature-limit
updates explicitly authorizes a write to the servo's nonvolatile storage.
Offset adjustment is runtime-only until `SAVE_OFFSET` is selected; torque state
is runtime-only. MCU reset or ROS reconnection does not replay, revert, or erase
servo-owned persistent values. If a later write fails, every persistent bit
already reported in `applied_mask` may remain changed and requires explicit
operator correction if that was not the desired final configuration.

## Bounded overload behavior

The overload policy is part of the public behavior:

| Resource | Policy | Safety effect |
| --- | --- | --- |
| Motor, PWM, LED, RGB, and OLED merged shadows | Selected fields from each taken, validated sample merge into one fixed shadow only within the current ROS session; the newest accepted selected value replaces any unread same-session value. On the first callback admitted for a new session, firmware drains the unread old snapshot and clears its per-field-valid generations before publishing. RGB/OLED owners merge only the new-session fields onto their last completed/rendered state. Best-effort loss before callback remains possible. | Accepted same-session disjoint updates cannot erase one another, backlog cannot delay the motor lease, already applied unselected peripheral state holds across reconnect, and unread or pending old-session fields cannot be relabelled as new work; hosts repeat every motor channel whose lease they intend to maintain. |
| Bus-motion mailbox | The newest complete valid request overwrites the old request. | The serial bus converges to current intent without a motion FIFO. |
| Buzzer mailbox | The latest complete validated pattern replaces an unread pattern; rejected input changes no output. | Buzzer traffic remains bounded without subset ambiguity. |
| Button queue | Capacity 16; drop oldest and count when full. | Recent physical events remain observable; loss is explicit. |
| Any occupied local service slot | Respond `BUSY`. | No hidden request backlog or executor wait. |
| Occupied shared bus service slot | Respond `BUSY` to every additional request, including stop; capacity remains one and an accepted service is nonpreemptive. When idle, stop is admitted before get/configure. An accepted stop finishes the current UART5 frame, abandons the active unsent remainder, invalidates every pending move generation accepted before the stop, and starts at the next frame boundary. | UART5 work stays bounded; stop interrupts move traffic without truncating a frame or corrupting an accepted get/configure transaction. Only a post-stop move can restart traffic. |
| Service traffic | On a service-class slice, poll at most one occupied completion slot and take at most one ready request, using persistent round-robin cursors. Begin at most one service response. | A request flood cannot turn one executor iteration into a FIFO drain or starve another service group. |
| Best-effort telemetry due together | Publish at most one of motor, PWM, or IMU per ACTIVE slice using persistent round-robin selection. | State rates remain available without stacking three bounded physical TX waits. |
| Reliable telemetry due together | On a reliable-telemetry-class slice, publish at most one of button, battery, heartbeat, or diagnostics using persistent round-robin selection. | Missing XRCE ACKs consume at most one reliable-publication timeout rather than four consecutive timeouts. |
| USART1 RX ring overrun | Stop motors and tear down the session. | Corrupt XRCE framing is never treated as commands. |
| Traffic budget exceeded | Fail qualification. | Do not compensate by adding unbounded buffering. |

No enqueue operation in a ROS callback may block. Every overwrite, drop, busy
response, and rejection increments a diagnostic counter.

USART1 RX half/full boundary accounting runs above the FreeRTOS syscall
ceiling and is therefore not delayed by task critical sections. That top half
may only clear the RX-DMA flags, advance its single-writer epoch, latch a sticky
error, and pend the lower USART1 IRQ. The deferred IRQ reconstructs producer
progress from epoch plus `NDTR`; a complete ring lap cannot alias to zero
progress. Any inconsistent epoch/cursor snapshot or DMA error is fatal and
uses the same stop-and-teardown path as an overrun.

## Transport and session faults

The following events shall enter safe teardown:

- any USART1 framing, noise, parity, or overrun error;
- circular DMA producer overtaking its consumer;
- TX DMA timeout or HAL error;
- fatal rcl/rmw/executor error;
- three consecutive active-state Agent ping failures;
- entity construction failure after the permitted retry path.

Safe teardown shall disarm motors first, invalidate all service generations,
and prevent publications or responses from the failed session. It then
finalizes constructed ROS objects in reverse order, resets USART1 and the
micro-ROS arena in task context, and retries with bounded backoff. It shall not
reset the MCU merely because the Agent or cable is absent.

The pinned Micro-XRCE-DDS Client 2.4.2 framing implementation has a measured
resynchronization limitation. A complete frame with a wrong CRC is discarded
and the next valid frame is accepted. If the final framed CRC octet is missing,
the parser consumes the immediately following valid frame in the deterministic
bulk-read and one-byte schedules. When three valid frames remain continuously
available, those exact schedules return the latter two. A separate case proves
conditional recovery after a later `0x7e` delimiter is followed by a zero-byte
custom-transport read before that frame's remaining bytes arrive. Duplicated
truncated input and the malformed payload itself remain fail-closed. These
observations characterize fixed schedules; they are not a general framing-
parser resynchronization deadline.

Reliable XRCE retransmission can restore reliable traffic after framing and
session liveness recover, but it is not a motor-safety mechanism and does not
replay best-effort motor commands. If recurring corruption or another arrival
schedule prevents parser recovery, three consecutive 10 ms Agent-ping failures
at the 500 ms cadence trigger bounded session teardown and recreation; this is
the operational recovery path, not a property attributed to the framing parser.
In all cases, each motor's independent 198--200 ms command lease expires without
a fresh accepted command; neither retransmission nor eventual session recovery
may refresh a lease with stale data.

Session deactivation shall publish `INACTIVE` and perform the emergency motor
stop in one controller critical section, in that order. A fatal USART1 or DMA
ISR shall latch its error before stopping motors. The target motor-arm and
four-channel duty hooks shall reject with `BUSY` whenever that sticky latch is
set, and those checks execute inside `MotorControlTask`'s existing controller
critical sections. Because the priority-4 RX-DMA top half can preempt those
BASEPRI sections while the priority-6 deferred stop cannot, each hook checks on
both sides of its writes. A pre-check failure emergency-stops before returning;
a post-check after the arm authority write or complete four-channel duty update
also emergency-stops before returning `BUSY`. If the latch arrives after the
post-check, there is no subsequent write and the deferred ISR stop runs when the
critical section exits. The latch may be cleared only by the normal USART1
transport-open path for a fresh physical session; no ROS message, service,
parameter, or host-side action is an unlock.

An encoder-read failure or a non-`BUSY` motor-arm/duty failure shall latch the
controller output fault, publish the current session inactive, invalidate all
four motor-owner leases, and stop the physical outputs before the detecting
`MotorControlTask` critical section ends. A `BUSY` arm/duty result caused by the
transport inhibit performs the same revocation without converting a recoverable
transport fault into a watchdog fault. It can regain authority only through a
different, freshly created session generation after normal transport open. A
controller output fault or a safety-supervisor decision to withhold IWDG is not
cleared by any session generation; reset into safe boot is required. The
supervisor publishes inactive and stops under the controller critical section
before yielding, so the watchdog reset interval is never a re-arm window.

Reliable publish and service-response operations in `ACTIVE` have a 10 ms XRCE
session timeout. Each entity-creation call is limited to 40 ms and the complete
creation phase to 2 s; each remote destroy/finalizer call is limited to 10 ms
and the complete remote-destruction phase to 500 ms. Task heartbeats advance
between lifecycle calls. If remote destruction times out because the Agent is
absent, teardown records the failure and continues local construction-state
cleanup, transport reset, arena restoration, and reconnect rather than waiting
or forcing an MCU reset.

An ACTIVE slice has one common blocking-operation permit shared by service
responses, reliable telemetry, Agent ping, and time synchronization. A strict
service → reliable-telemetry → maintenance class rotation decides which class
may use it; idle turns are not borrowed. Within maintenance, a due ping runs
before a due time-sync attempt, and the latter remains due for the next
maintenance turn. A request or publication flood therefore cannot starve
session health, while a maintenance flood cannot starve the ROS endpoints. A
second permit request in one slice is a fatal invariant violation.

On reconnect, all motion mailboxes and lease timestamps from the old session
are invalid. PWM/bus servos continue the communication-loss hold state, while
motors require a new accepted command.

All seven topic gateways capture the observed active generation before their
fast check and recheck `ACTIVE` plus that exact generation inside the controller
critical section. A callback spanning revoke/reconnect returns `BUSY` and
publishes nothing. Motor, PWM, LED, RGB, and OLED subset shadows have explicit
session ownership: their first admitted callback in a fresh session drains old
unread data and clears field-valid generations. Worker-owned already applied
PWM/LED/RGB/OLED state remains held; only fields explicitly accepted in the new
session may change it. Whole-command bus and buzzer work retains its generation
tag and is rejected when that tag is stale.

The controller treats its nonzero session generation as a wrap-aware monotonic
watermark. An active generation accepts only an exact idempotent repeat; any
different active request is ignored until an explicit inactive transition.
Deactivation always stops motors, but an equal or stale generation cannot lower
the watermark. Reactivation requires a strictly newer nonzero generation under
signed-delta ordering, including the `UINT32_MAX` to `1` wrap. Consequently a
late teardown callback cannot make the just-revoked generation eligible for
replay.

## Watchdog supervision

The legacy firmware refreshes IWDG from unrelated application and RGB paths,
which can hide a failed critical task. V2 shall have exactly one refresh site in
`SafetySupervisorTask`.

Configure IWDG prescaler 64 and reload 249, nominally 500 ms at a 32 kHz LSI.
The board's actual LSI tolerance shall be measured; the qualified maximum reset
interval shall be below one second. The supervisor runs every 20 ms and refreshes
only when all task heartbeat ages are within these limits:

| Task | Maximum heartbeat age |
| --- | ---: |
| `MotorControlTask` | 30 ms |
| `MicroRosTask` | 100 ms |
| `BusServoTask` | 100 ms |
| `SensorTask` | 150 ms |
| `PeripheralTask` | 150 ms |

Every task shall advance its heartbeat after completing one bounded iteration,
not before entering a wait or hardware operation. A disconnected but healthy
`MicroRosTask` advances while executing the Agent backoff state machine.

If a heartbeat is stale, the supervisor calls `EmergencyStopMotors()`, records
the failing task in retained reset diagnostics where possible, and stops
refreshing IWDG. No other task, driver, assertion handler, or interrupt may
refresh it. A fatal startup error shall also leave motors safe and allow IWDG to
reset rather than spin forever with interrupts disabled.

Only the first concrete stale task of a boot is retained. A 12-byte,
four-byte-aligned `.noinit` record contains magic `0x52525732`, a version-1
payload (`version[31:24]`, zero reserved bits `[23:8]`, task `[7:0]`), and the
payload's bitwise complement. The writer clears magic and completes a data
synchronization barrier, writes payload/complement, completes a memory barrier,
then publishes magic and completes a final data synchronization barrier. Early
boot captures RCC reset cause first, validates the record, and clears it for
every reset cause. It exposes the retained task only for an independent-
watchdog reset; power, pin, software, window-watchdog, brownout, low-power,
unknown, malformed, and torn cases report `TASK_NONE`. The current boot's
public `last_watchdog_task` remains the captured prior-boot value until reset;
current fault detail is available through health and last-error fields.

Qualification treats every watchdog reset as a safety failure, including in
the operator-driven MCU-reset campaign. That campaign accepts only power-on,
pin, software, or brownout as an intentional reset cause; window-watchdog,
low-power, unknown, and unrecognized causes also fail and cannot satisfy a
reset cycle. The evidence ledger records the accepted cause on the exact
session transition to which the post-reset diagnostics belong.

## Bounded peripheral failures

- UART5 reads and writes use fixed request/response buffers and per-operation
  timeouts. The worst all-field bus query must finish or report failure before
  the 200 ms MCU service deadline.
- OLED I2C, IMU software-I2C, RGB SPI DMA, and ADC waits shall have explicit
  timeouts. `HAL_MAX_DELAY`, `0xFFFF`, and unbounded acknowledge loops are
  forbidden.
- A failed sensor or display transaction increments its device counter and
  retains the previous data/display values. A sensor snapshot is marked
  invalid or stale exactly as its interface defines. The failure shall not
  block motor control or the executor.
- Repeated peripheral failures may mark that device unavailable, but shall not
  manufacture measurements or successful service fields.
- The low-battery state uses the exact debounce, hysteresis, ten-second repeat,
  and buzzer-priority rules in the ROS interface contract. It retains the
  latest host buzzer pattern and resumes it after the battery pattern. Only the
  motor command lease or an explicit higher-level safety requirement may
  change motor output.

## Static-memory integrity

The MCU shall have no general-purpose runtime heap. During `CREATE_ENTITIES`, a
bounded CCM arena services micro-ROS initialization. While the generation is
sealed in `ACTIVE`:

- allocate, reallocate, or deallocate is a fatal diagnostic and safe teardown/
  reset event;
- all strings, arrays, queues, snapshots, transport buffers, and service
  responses retain fixed storage;
- stack overflow hooks call `EmergencyStopMotors()` and withhold IWDG refresh;
- link-map and runtime high-water gates in [architecture.md](architecture.md)
  are mandatory.

After ROS execution is quiescent, `TEARDOWN` may admit only the deallocation
calls made by object finalizers, then resets the whole arena. A remote-destroy
timeout does not skip required local finalization or arena restoration. New
allocation or reallocation remains forbidden. The next `CREATE_ENTITIES` may
use the reset arena before sealing the new generation; every cycle shall return
to the same baseline.

Any allocator callback while the `ACTIVE` seal is set is a firmware-invariant
violation: stop motors, latch and count the fault, perform only best-effort
teardown, and withhold IWDG refresh. It shall reset into safe boot rather than
resume through the ordinary reconnect loop.

Memory pressure shall never be handled by losing an unchecked pointer, reusing
an in-flight service buffer, or allocating per incoming message.

## Fault response matrix

| Fault | Immediate response | Recovery |
| --- | --- | --- |
| One motor command expires | Zero/disarm that motor; clear its PID state. | Fresh valid command for that motor while session is active and only within the active build's authority. |
| All command traffic stops but Agent still answers | Per-motor leases expire independently. | Fresh valid commands subject to the normal lock or commissioning cap; no session recreation required. |
| Agent process or cable disappears | Motor lease remains primary; detected ping/transport failure disarms all and tears down. | Automatic Agent/session retry; fresh motor commands remain subject to the active build's authority. |
| Invalid command | Reject atomically and count; refresh no affected lease. | Publisher corrects the message. |
| Mailbox overwrite | Apply newest value and count overwrite. | No special recovery. |
| Button overflow | Drop oldest event and count. | Queue drains normally. |
| Service slot busy | Immediate `BUSY` response. | Client retries with backoff. |
| UART5 field timeout | Return validity/applied mask plus typed failure. | Later service request; UART5 worker remains serialized. |
| USART1 error or RX overrun | Emergency motor stop and session teardown. | Task-context UART reset and reconnect. |
| Critical task stall | Emergency motor stop, record cause, withhold IWDG. | Hardware watchdog reset into safe boot. |
| Sensor/display failure | Retain prior state, mark stale/unavailable, count error. | Bounded retry by owning task. |
| Arena or stack violation | Emergency motor stop and fatal diagnostic. | IWDG reset; release remains blocked until root cause is fixed. |

## Required observability

The diagnostic publication shall include at least:

- current session state, session generation, reconnect count, and last teardown
  reason;
- per-motor lease expiry and command-reject counts;
- motor mailbox-consumption count, strict-over-20-ms count, and maximum
  callback-acceptance-to-owner-consumption age;
- mailbox overwrite and button drop counts;
- service busy, timeout, partial, and late-completion counts;
- USART1 error classes, RX high-water/overrun, TX timeout, and maximum transport
  wait;
- UART5 and per-peripheral timeout/error counts;
- task missed-release counts, execution high-water, stack minimum, and sampled
  heartbeat age;
- allocation attempts after seal;
- reset cause and last retained watchdog offender;
- cumulative RX/TX byte counters from which combined one-second traffic is
  derived. Reliable history remains the fixed eight-by-512-byte configured
  capacity; ACK failure and occupancy behavior are verified by fault injection
  and independent XRCE capture because the pinned public middleware API exposes
  no truthful retry or stream-occupancy counters.

Counters shall saturate rather than wrap silently. Diagnostics may be lost with
the session; safety behavior shall not depend on their delivery.

## Release-blocking evidence

Release requires the full matrix in [verification.md](verification.md),
including sustained 500 Hz motor commands with all retained telemetry and the
worst bus service, malformed-input tests, queue saturation, UART fault injection,
Agent/cable restart cycles, watchdog task-stall injection, and a soak test.

At minimum, qualification shall demonstrate:

- callback-acceptance-to-`MotorControlTask` mailbox-consumption p99 no greater
  than 20 ms;
- every expired motor reaches zero no earlier than 198 ms and no later than
  200 ms after its last valid command is accepted;
- no motor control release miss, RX overrun, unexpected reset, or post-seal
  allocation during the one-hour full-load run;
- service completion within its deadline or an explicit typed failure;
- at least 25% remaining stack for every task;
- automatic recovery from at least 100 Agent kill/restart and cable
  unplug/replug cycles without duplicate ROS entities or stale command replay;
- deliberate critical-task stalls cause motor shutdown followed by watchdog
  reset within the characterized sub-one-second bound.

Any failure blocks release. Raising a queue limit, weakening a deadline, or
changing a safe state requires the change-control process in
[README.md](README.md), not a test waiver.
