# Mentor Pi Controller v2 ROS Interface Contract

The opt-in host-only polynomial tracking interfaces, topics, services, QoS,
scheduling, and safe-output behavior are normative in
[tracking-controller.md](tracking-controller.md). They do not change the fixed
MCU endpoint count or `/mentor_pi/controller` contract below.

## 1. Contract status and identity

This document is the normative public ROS 2 contract for controller v2. An
implementation is conforming only when its graph, IDL, QoS, validation,
timing, and failure behavior match this document. Product requirements are in
[requirements.md](requirements.md); task and transport ownership are in
[architecture.md](architecture.md).

| Item | Contract value |
|---|---|
| ROS distribution | ROS 2 Humble |
| Custom interface package | `mentor_pi_interfaces` |
| Firmware package | `mentor_pi_mcu` |
| Bring-up/supervisor package | `mentor_pi_bringup` |
| Node namespace | `/mentor_pi` |
| Node name | `controller` |
| Fully qualified node name | `/mentor_pi/controller` |
| Publishers / subscriptions / services | 7 / 7 / 7 |
| Default transport device | `/dev/mentor_pi_mcu` |
| Serial settings | 1,000,000 baud, 8 data bits, no parity, 1 stop bit, no flow control |
| Compatibility policy | Clean v2 only; no legacy type, name, or packet aliases |

Names in the remainder of this document are fully qualified. Implementations
shall create endpoints from relative names inside namespace `/mentor_pi` so
normal ROS remapping remains available.

The MCU node shall disable `/rosout` and shall not create parameter,
parameter-event, statistics, action, or `/clock` endpoints. Middleware-internal
discovery entities are not application endpoints; apart from those internal
entities, the 7/7/7 inventory below is the complete MCU graph surface.

## 2. Endpoint inventory, QoS, and rates

`BE` means best effort; `REL` means reliable. Every endpoint is volatile with
keep-last history. Deadline, lifespan, liveliness lease, and transient-local
durability are not requested. A host publisher offering reliable QoS is
compatible with a controller subscription requesting best effort QoS.

For command subscriptions, **nominal** is the intended application rate and
**maximum** is the highest supported ingress rate. Traffic above maximum is
unsupported and may be coalesced or dropped, but shall not create backlog,
starve control work, or consume additional memory. The 500 Hz motor maximum is
an overload/stress contract; it does not promise application of every sample.

### 2.1 Subscriptions: host to MCU

| Fully qualified topic | Type | QoS | Nominal | Maximum | Delivery behavior |
|---|---|---:|---:|---:|---|
| `/mentor_pi/motors/command` | `mentor_pi_interfaces/msg/MotorCommand` | BE, depth 1 | 50 Hz | 500 Hz | Merge selected channels into latest state |
| `/mentor_pi/pwm_servos/command` | `mentor_pi_interfaces/msg/PwmServoCommand` | BE, depth 1 | ≤20 Hz/on change | 50 Hz | Merge selected channels into latest state |
| `/mentor_pi/bus_servos/command` | `mentor_pi_interfaces/msg/BusServoCommand` | BE, depth 1 | ≤10 Hz/on change | 20 Hz | One replace-latest batch slot |
| `/mentor_pi/leds/command` | `mentor_pi_interfaces/msg/LedCommand` | REL, depth 1 | ≤5 Hz/on change | 10 Hz | Latest validated pattern per LED |
| `/mentor_pi/buzzer/command` | `mentor_pi_interfaces/msg/BuzzerCommand` | REL, depth 1 | ≤5 Hz/on change | 10 Hz | Latest validated pattern |
| `/mentor_pi/rgb/command` | `mentor_pi_interfaces/msg/RgbCommand` | REL, depth 1 | ≤30 Hz/on change | 50 Hz | Replace host-owned RGB2; RGB1 is MCU status |
| `/mentor_pi/oled/command` | `mentor_pi_interfaces/msg/OledCommand` | REL, depth 1 | ≤1 Hz/on change | 2 Hz | Merge selected lines into latest state |

Best-effort depth-one motion QoS may discard an older sample before the MCU
takes it. Subset merging applies only to samples that reach the callback and
pass validation; the controller never reconstructs a partial update it did not
receive. A motor publisher shall therefore repeat every channel whose lease it
intends to maintain, normally with the full four-bit mask. A PWM- or bus-servo
publisher that requires confirmed application shall observe PWM state or use
the applicable bus state service and repeat its bounded command when needed.

### 2.2 Publishers: MCU to host

| Fully qualified topic | Type | QoS | Publish rate |
|---|---|---:|---:|
| `/mentor_pi/motors/state` | `mentor_pi_interfaces/msg/MotorState` | BE, depth 1 | 50 Hz |
| `/mentor_pi/pwm_servos/state` | `mentor_pi_interfaces/msg/PwmServoState` | BE, depth 1 | 20 Hz |
| `/mentor_pi/imu` | `mentor_pi_interfaces/msg/ImuState` | BE, depth 1 | 50 Hz |
| `/mentor_pi/buttons/events` | `mentor_pi_interfaces/msg/ButtonEvent` | REL, depth 8 | Event-driven; maximum 20 events/s burst |
| `/mentor_pi/battery/state` | `mentor_pi_interfaces/msg/BatteryState` | REL, depth 1 | 1 Hz |
| `/mentor_pi/heartbeat` | `mentor_pi_interfaces/msg/Heartbeat` | REL, depth 1 | 2 Hz |
| `/mentor_pi/diagnostics` | `mentor_pi_interfaces/msg/ControllerDiagnostics` | REL, depth 1 | 1 Hz |

### 2.3 Services

All services use reliable, volatile, keep-last depth 1 QoS. The controller
accepts one request at a time for each non-bus service and one request total
across all three bus services. A request received while its slot is occupied
gets a `BUSY` response. Non-bus work has a 50 ms local deadline. Bus work has a
200 ms MCU deadline, and host clients shall use a 250 ms response timeout.

| Fully qualified service | Type | Purpose |
|---|---|---|
| `/mentor_pi/motors/set_model` | `mentor_pi_interfaces/srv/SetMotorModel` | Select one profile for all four motors |
| `/mentor_pi/motors/set_adrc` | `mentor_pi_interfaces/srv/SetMotorAdrc` | Apply volatile closed-loop gains while all motors are stopped |
| `/mentor_pi/pwm_servos/set_offsets` | `mentor_pi_interfaces/srv/SetPwmServoOffsets` | Set PWM-servo calibration offsets |
| `/mentor_pi/bus_servos/get_state` | `mentor_pi_interfaces/srv/GetBusServoState` | Read selected registers from one servo |
| `/mentor_pi/bus_servos/configure` | `mentor_pi_interfaces/srv/ConfigureBusServo` | Write selected registers on one servo |
| `/mentor_pi/bus_servos/stop` | `mentor_pi_interfaces/srv/StopBusServos` | Stop one through 16 servos |
| `/mentor_pi/battery/set_low_threshold` | `mentor_pi_interfaces/srv/SetBatteryThreshold` | Set the runtime low-voltage threshold |

The three bus services share one non-preemptible request slot. While that slot
is occupied, every additional bus request, including stop, receives `BUSY`.
When the slot is free and more than one request is ready, the executor polls
stop first, then alternates get/configure deterministically; any accepted bus
service is dispatched before pending move traffic. Thus an accepted stop never
waits behind a move and may interrupt a multi-servo move between frames, but it
does not preempt an already accepted query or configuration. A client that
receives `BUSY` may retry within its own bounded policy. Accepting a stop records
the current move-generation watermark. Every not-yet-transmitted frame in the
active batch and every pending move at or below that watermark is discarded;
none resumes implicitly after the stop completes. Only a `BusServoCommand`
accepted after the stop may restart motion traffic.

## 3. Common rules and result model

### 3.1 General validation

- Masks shall be nonzero, shall contain only defined bits, and apply only to
  selected elements. Values in unselected fixed-array elements are ignored.
- A `count` shall be within its stated range. Fixed-array elements at indices
  greater than or equal to `count` are ignored.
- Selected floating-point values shall be finite; NaN and either infinity are
  invalid.
- Multi-element validation is atomic. If any selected element is invalid, the
  complete topic message or service request is rejected without changing an
  output. An invalid motor command does not refresh a lease.
- Numeric ranges are inclusive. Values outside a range are rejected, not
  clamped, unless a saturation behavior is explicitly stated.
- Unknown enum values, mask bits, or button events are invalid.
- A topic rejection increments diagnostics counters and records the last
  error. Topic commands have no per-message acknowledgement.

### 3.2 `msg/Result.msg`

```text
uint8 OK=0
uint8 INVALID_ARGUMENT=1
uint8 OUT_OF_RANGE=2
uint8 BUSY=3
uint8 TIMEOUT=4
uint8 IO_ERROR=5
uint8 UNSUPPORTED=6
uint8 PARTIAL=7

uint8 code
uint16 detail
```

| Code | Meaning |
|---:|---|
| `OK` | Request was valid and completed within its deadline. For a write-only bus command this means successfully transmitted, not physically verified. |
| `INVALID_ARGUMENT` | Invalid zero/unknown mask, zero count, duplicate ID, inconsistent limits, malformed text, or unknown enum. |
| `OUT_OF_RANGE` | A recognized field is outside its inclusive numeric range or a selected float is non-finite. |
| `BUSY` | The fixed request/worker slot is occupied or the requested state transition is unsafe now. |
| `TIMEOUT` | The operation did not complete before its local deadline or a device did not reply. |
| `IO_ERROR` | HAL, UART, sensor, or peripheral I/O failed. |
| `UNSUPPORTED` | The request is defined by the interface but unavailable in the running hardware build. |
| `PARTIAL` | One or more bus fields completed before a later field failed; consult the valid/applied mask. |

`detail` is zero for success and when no additional detail exists. For a fixed
array validation error it is the one-based failing element index. For a field
mask error it is the offending bit mask. For an I/O error it is the stable
driver-specific error value documented by the implementation; raw memory
addresses and unstable HAL line numbers shall not be exposed.

## 4. Motor interface

Array index 0 through 3 maps to physical M1 through M4. Positive rotation and
encoder polarity are defined by [hardware-baseline.md](hardware-baseline.md).

### 4.1 `msg/MotorCommand.msg`

```text
uint8 MOTOR_1=1
uint8 MOTOR_2=2
uint8 MOTOR_3=4
uint8 MOTOR_4=8
uint8 ALL_MOTORS=15

uint8 update_mask
float32[4] target_rps
```

`update_mask` bits 0 through 3 select M1 through M4. Each selected value is
output-shaft revolutions per second in the raw-signed MCU motor coordinate,
not the ROS chassis coordinate. It must satisfy the absolute limit of the
active model:

| Model value | Name | Encoder ticks/output revolution | Absolute RPS limit |
|---:|---|---:|---:|
| 0 | JGB520 | 3960 | 1.5 |
| 1 | JGB37 | 1980 | 3.0 |
| 2 | JGA27 | 1040 | 6.0 |
| 3 | JGB528 | 5764 | 1.1 |

The only supported firmware artifact is `NORMAL_CLOSED_LOOP_DEFAULT` with
`control_mode=CLOSED_LOOP`. A structurally valid selected zero target is a stop
update. A selected nonzero target is accepted only when its magnitude is no
greater than both the active profile limit and the 6 RPS implementation
ceiling. A command containing any invalid selected value is rejected atomically
as `INVALID_ARGUMENT` or `OUT_OF_RANGE`; no selected target changes and no
motion lease is refreshed. Because this is a topic, the result is reported
through controller diagnostics rather than a synchronous response. No ROS
message or service selects a different firmware control mode.

Each selected channel receives an independent 200 ms lease. TIM7 releases
`MotorControlTask` at 1 kHz; including scheduling jitter, the qualified maximum
interval between completed lease evaluations is 2,000 microseconds. The fixed
expiry threshold is 198,000 microseconds: at the first evaluation whose unsigned
monotonic age is at least that threshold, a nonzero target is atomically set to
zero and its watchdog bit is set. This early threshold guarantees the zero
target is installed no later than 200 ms after callback acceptance; an
inter-evaluation interval above 2,000 microseconds is a test failure. Updating
one channel does not refresh another.
A later valid update clears that bit. A zero target is a commanded stop and
does not create a watchdog trip when it ages out.

Callback acceptance is read from the controller's exact modulo-`uint32`
microsecond monotonic hook. Consumption age and lease evaluation use that same
clock; deriving acceptance as millisecond uptime multiplied by 1,000 is
nonconforming.

### 4.2 `msg/MotorState.msg`

```text
uint8 MODEL_JGB520=0
uint8 MODEL_JGB37=1
uint8 MODEL_JGA27=2
uint8 MODEL_JGB528=3

builtin_interfaces/Time stamp
float32[4] target_rps
float32[4] measured_rps
int64[4] encoder_count
uint8 motor_model
uint8 watchdog_stop_mask
```

- `target_rps` is the current post-watchdog control target.
- `measured_rps` is the signed output-shaft estimate used by the ADRC loop.
- `encoder_count` is the signed accumulated quadrature count since MCU boot;
  no ROS API resets it.
- `watchdog_stop_mask` uses the same bit mapping as `MotorCommand`.

Encoder sampling and these state fields remain active while motor output is
zero. They are the ROS-visible evidence for the required passive direction test
performed by manually rotating each raised wheel before powered motion.
Firmware publishes raw signed encoder state for every motor model. `target_rps`,
`measured_rps`, accumulated count, LADRC state, and signed bridge duty share
this MCU coordinate with no firmware sign transform. The ROS hardware layer
owns the only map to positive wheel rotation: `{-1,+1,-1,+1}` in logical
FL,FR,RL,RR order, applied symmetrically to commands and feedback.

### 4.3 `srv/SetMotorModel.srv`

```text
uint8 MODEL_JGB520=0
uint8 MODEL_JGB37=1
uint8 MODEL_JGA27=2
uint8 MODEL_JGB528=3

uint8 model
---
mentor_pi_interfaces/Result result
uint8 active_model
uint32 ticks_per_revolution
float32 max_rps
```

One model applies to all four channels. Default after reset is JGA27. A request
for the already active model is idempotent: it returns `OK` with no ADRC reset,
including while a target is nonzero. For an actual model change, if any current
target is nonzero, return `BUSY` and change nothing. A successful actual change
applies the encoder constant, RPS limit, and fixed controller parameters to all
four motors, resets their ADRC observer/output state, and returns the effective
profile values. The returned `max_rps` is the model limit and does not override
the 6 RPS implementation ceiling. The setting is runtime-only. Before enabling its session-bound motion
authorization, the host supervisor shall require an `OK` response to echo the
requested `active_model` and the exact profile values in the project-owned
shared contract. A mismatched model, tick count, non-finite speed, or different
finite speed is treated as `IO_ERROR`; configuration is rejected for that
generation and the motion gate remains closed.

All four profiles use the same hardcoded, bounded first-order LADRC defaults:
input gain `b0=0.03 RPS/s/permille`, controller bandwidth `wc=4 rad/s`,
observer bandwidth `wo=12 rad/s`, and velocity-filter new-sample weight `0.5`.
They are not release-qualified. D3 HIL shall qualify or replace these values,
physical command/encoder polarity, filter, and the currently disabled
zero-permille minimum-drive floor for each profile and record the evidence before nonzero production motion
is released. The defective legacy PID expression and its negative gains are not
a normative output algorithm. Changing any qualified constant or sign later
invalidates the affected motor HIL evidence.

### 4.4 `srv/SetMotorAdrc.srv`

```text
uint8 MOTOR_1=1
uint8 MOTOR_2=2
uint8 MOTOR_3=4
uint8 MOTOR_4=8
uint8 ALL_MOTORS=15

uint8 update_mask
float32[4] input_gain_rps_per_second_per_permille
float32[4] controller_bandwidth_rad_s
float32[4] observer_bandwidth_rad_s
float32[4] velocity_filter_new_weight
---
mentor_pi_interfaces/Result result
uint8 applied_mask
```

`update_mask` shall be a nonzero subset of `ALL_MOTORS`. Every selected input
gain shall be finite, positive, and no greater than `1000`. Every selected
controller and observer bandwidth shall be finite and positive; controller
bandwidth shall not exceed observer bandwidth, and observer bandwidth shall not
exceed `50 rad/s`. Every selected filter new-sample weight shall be finite and
in `[0, 1]`. A non-finite selected value returns `INVALID_ARGUMENT`; a finite
value outside these bounds returns `OUT_OF_RANGE`.

The service is supported by the default closed-loop ADRC artifact. It returns
`BUSY` unless every channel is disarmed, every target is zero, and every
measured speed has magnitude below `0.01 RPS`. Validation, application, and
ADRC-state reset are atomic across all selected channels. On success
`applied_mask == update_mask`; every failure returns an applied mask of zero.
Overrides are volatile: they survive an Agent
transport reconnection but are cleared by an MCU reset or an actual motor-model
change.
The deployment supervisor validates four exactly-four-element double YAML
arrays named `input_gain_rps_per_second_per_permille`, `controller_bandwidth_rad_s`, `observer_bandwidth_rad_s`, and
`velocity_filter_new_weight`, using these same ranges. After each session it
sets the motor model, applies all four ADRC arrays with `ALL_MOTORS`, verifies
both `OK` and `applied_mask == ALL_MOTORS`, then applies PWM offsets and the
battery threshold before enabling motion.
If the bounded service deadline expires before the motor owner commits the
update, timeout cancellation wins the same owner critical section and prevents
any later gain mutation for that request. If the owner commit wins first, the
completed success response is returned instead of `TIMEOUT`.

The closed-loop image uses filtered measured RPS and evaluates first-order
linear ADRC at 100 Hz. The first sample establishes a nominal 10 ms timing
baseline; subsequent samples use the wrap-safe actual elapsed period `T`. With
`e = z1 - filtered_measured_rps` and the previously applied post-floor output:

```text
z1 += T * (z2 + b0 * applied_output - 2 * wo * e)
z2 += T * (-wo * wo * e)
output = (wc * (target_rps - z1) - z2) / b0
```

`b0` is in RPS/s/permille and `wc`/`wo` are in rad/s. A non-finite state or
`wo*T > 0.5` disarms the affected channel and records a watchdog stop. The
final output is clamped to `[-1000, 1000]` permille. The minimum-drive floor is
zero, so calculated nonzero outputs are not raised to a fixed duty. Stop, lease expiry,
disarming, session loss, a successful selected ADRC update, and an actual model
change reset observer and applied-output state.

## 5. PWM servo interface

Array index 0 through 3 maps to PWM servo connector 1 through 4.

### 5.1 `msg/PwmServoCommand.msg`

```text
uint8 SERVO_1=1
uint8 SERVO_2=2
uint8 SERVO_3=4
uint8 SERVO_4=8
uint8 ALL_SERVOS=15

uint8 update_mask
uint16 duration_ms
uint16[4] pulse_width_us
```

`duration_ms` shall be 20 through 30000. Each selected pulse width shall be
500 through 2500 microseconds. At callback acceptance, the target changes but
the current logical pulse does not. The first following common 20 ms PWM frame
boundary snapshots that current pulse and starts interpolation. The number of
frame intervals is `N = ceil(duration_ms / 20)`. Call that first boundary
`B0`. At `Bk = B0 + k * 20 ms`, for `k` from 1 through `N`, let
`q = (target - start) * k` in signed 32-bit arithmetic and install
`start + sign(q) * floor((abs(q) + N / 2) / N)`. This is nearest-integer
interpolation with exact half cases rounded away from zero. The target is
therefore installed exactly at `BN`, after `N` complete frame intervals from
`B0`. Interpolation time from `B0` is in
`[duration_ms, duration_ms + 20 ms)`. All channels selected by one message use
the same `B0` and `N`. A new target replaces the remaining trajectory from the
logical pulse currently being output at the next common boundary.

### 5.2 `msg/PwmServoState.msg`

```text
builtin_interfaces/Time stamp
uint16[4] target_pulse_width_us
uint16[4] output_pulse_width_us
int16[4] offset_us
uint8 moving_mask
```

`target_pulse_width_us` excludes calibration offset.
`output_pulse_width_us` is the actual timer pulse after interpolation and
offset. The final hardware pulse is saturated to 500 through 2500
microseconds after applying the offset. `moving_mask` bit mapping matches the
command.

After MCU reset, all four logical targets and outputs initialize to 1500
microseconds, all offsets initialize to zero, and `moving_mask` is zero. This
reset default is not an old-session replay. During ordinary Agent loss, the
active output pulses continue unchanged as required by the servo-hold rule.

### 5.3 `srv/SetPwmServoOffsets.srv`

```text
uint8 SERVO_1=1
uint8 SERVO_2=2
uint8 SERVO_3=4
uint8 SERVO_4=8
uint8 ALL_SERVOS=15

uint8 update_mask
int16[4] offset_us
---
mentor_pi_interfaces/Result result
uint8 applied_mask
```

Each selected offset shall be -100 through +100 microseconds. Validation and
application are atomic, so `applied_mask` is either `update_mask` on `OK` or
zero on failure. An actual change is committed for every selected channel at
one common 20 ms frame boundary; `OK` is sent only after that commit. Logical
targets and interpolation phase are unchanged, while the hardware pulses are
recomputed with saturation from the new offsets. Reapplying values equal to all
selected active offsets returns `OK` without changing servo state or
interpolation. Offsets are runtime-only and reset to zero.

## 6. Bus servo interface

Public addressed IDs are 1 through 253. At most 16 IDs may appear in one
batch; used IDs shall be unique. UART operations are serialized by the bus
worker and never run in a micro-ROS callback.

### 6.1 `msg/BusServoCommand.msg`

```text
uint8 count
uint8[16] servo_id
uint16[16] position
uint16 duration_ms
```

- `count`: 1 through 16.
- Used `servo_id`: 1 through 253, with no duplicates.
- Used `position`: 0 through 1000 servo units.
- `duration_ms`: 0 through 30000; zero requests the servo's fastest supported
  move.

A validated batch atomically replaces the single pending move batch. If it is
overwritten before the worker starts it, diagnostics records a mailbox
overwrite. Once a multi-servo batch starts transmitting, it completes unless
an accepted stop interrupts it between frames or a fatal UART fault occurs. On
stop acceptance, the active unsent remainder and every pending move generation
accepted before that stop are invalidated. A post-stop command receives a newer
generation and is the only way motion traffic can restart after stop completes.

### 6.2 `msg/BusServoState.msg`

```text
uint16 FIELD_ID=1
uint16 FIELD_POSITION=2
uint16 FIELD_OFFSET=4
uint16 FIELD_VOLTAGE=8
uint16 FIELD_TEMPERATURE=16
uint16 FIELD_POSITION_LIMITS=32
uint16 FIELD_VOLTAGE_LIMITS=64
uint16 FIELD_TEMPERATURE_LIMIT=128
uint16 FIELD_TORQUE=256
uint16 ALL_FIELDS=511

uint16 valid_fields
uint8 requested_id
uint8 reported_id
int16 position
int8 offset
uint16 voltage_mv
uint8 temperature_c
uint16 position_min
uint16 position_max
uint16 voltage_min_mv
uint16 voltage_max_mv
uint8 temperature_limit_c
bool torque_enabled
```

Only fields selected in `valid_fields` contain a valid result. `position` is
signed because the servo can report a transient/calibrated value outside the
normal command range. Limit values and voltage use servo units shown above.

### 6.3 `srv/GetBusServoState.srv`

```text
uint16 FIELD_ID=1
uint16 FIELD_POSITION=2
uint16 FIELD_OFFSET=4
uint16 FIELD_VOLTAGE=8
uint16 FIELD_TEMPERATURE=16
uint16 FIELD_POSITION_LIMITS=32
uint16 FIELD_VOLTAGE_LIMITS=64
uint16 FIELD_TEMPERATURE_LIMIT=128
uint16 FIELD_TORQUE=256
uint16 ALL_FIELDS=511

uint8 servo_id
uint16 fields
---
mentor_pi_interfaces/Result result
mentor_pi_interfaces/BusServoState state
```

`fields` shall be a nonzero subset of `ALL_FIELDS`. Normally `servo_id` is 1
through 253. ID 254 is permitted only when `fields == FIELD_ID` and exactly one
servo is physically connected; this is the discovery operation inherited from
the active hardware. Multiple devices replying to 254 cause `IO_ERROR`.

Reads occur in ascending field-bit order. On full success, `result` is `OK`
and `valid_fields == fields`. If no field completes, return the causal error
and zero valid fields. If a later read fails after at least one success, return
`PARTIAL` and the exact completed `valid_fields`.

### 6.4 `srv/ConfigureBusServo.srv`

```text
uint16 SET_ID=1
uint16 SET_OFFSET=2
uint16 SAVE_OFFSET=4
uint16 SET_POSITION_LIMITS=8
uint16 SET_VOLTAGE_LIMITS=16
uint16 SET_TEMPERATURE_LIMIT=32
uint16 SET_TORQUE=64
uint16 ALL_UPDATES=127

uint8 servo_id
uint16 update_mask
uint8 new_id
int8 offset
uint16 position_min
uint16 position_max
uint16 voltage_min_mv
uint16 voltage_max_mv
uint8 temperature_limit_c
bool torque_enabled
---
mentor_pi_interfaces/Result result
uint16 applied_mask
uint8 effective_id
```

Validation:

- `servo_id` and selected `new_id`: 1 through 253.
- `update_mask`: nonzero subset of `ALL_UPDATES`.
- Selected `offset`: -125 through +125 servo units.
- Selected position limits: both 0 through 1000 and
  `position_min <= position_max`.
- Selected voltage limits: both 4500 through 14000 mV and
  `voltage_min_mv <= voltage_max_mv`.
- Selected temperature limit: 0 through 100 °C.
- `SET_ID`, `SET_POSITION_LIMITS`, `SET_VOLTAGE_LIMITS`, and
  `SET_TEMPERATURE_LIMIT` use the servo's persistent write operations and are
  required to survive a servo power cycle; release HIL shall verify that
  behavior on the supported servo fixture.
- `SET_OFFSET` changes the volatile offset only. `SAVE_OFFSET` may be requested
  alone and commits the servo's current offset to its persistent storage.
- `SET_TORQUE` is volatile. The servo's power-up torque default is not relied
  upon. The MCU persists and automatically replays none of these values.

After complete validation, writes occur in this order: offset, save offset,
position limits, voltage limits, temperature limit, torque, then ID last. An
I/O failure stops further writes and returns `PARTIAL` if `applied_mask` is
nonzero, otherwise the causal error. `effective_id` is `new_id` only when
`SET_ID` is present in `applied_mask`; otherwise it is `servo_id`. These
write-only servo operations report successful transmission and do not imply
readback verification.

### 6.5 `srv/StopBusServos.srv`

```text
uint8 count
uint8[16] servo_id
---
mentor_pi_interfaces/Result result
uint8 commands_transmitted
```

`count` is 1 through 16; used IDs are unique and 1 through 253. Stop frames are
sent in array order. `commands_transmitted` counts successfully transmitted
frames. A failure after at least one transmission returns `PARTIAL`; otherwise
it returns the causal error. The dispatch and non-preemption rules in Section
2.3 apply: an accepted stop is ahead of move traffic, but a stop received while
another bus service owns the slot returns `BUSY`. Stop acceptance also applies
the move-generation invalidation rule in Sections 2.3 and 6.1.

## 7. Discrete output interfaces

### 7.1 `msg/LedCommand.msg`

```text
uint8 led_id
uint16 on_time_ms
uint16 off_time_ms
uint16 repeat
```

`led_id` is 2 or 3. LED1 is firmware-owned and an ID of 1 is rejected with
`OUT_OF_RANGE` without changing its 1 Hz system heartbeat. Timing
fields span 0 through 65535 ms. Semantics are:

- `on_time_ms == 0`: steady off; other timing fields are ignored.
- `on_time_ms > 0 && off_time_ms == 0`: steady on; `repeat` is ignored.
- Both nonzero: blink with that on/off duration; `repeat == 0` repeats until
  replaced, otherwise it is the number of complete on/off cycles.

### 7.2 `msg/BuzzerCommand.msg`

```text
uint16 frequency_hz
uint16 on_time_ms
uint16 off_time_ms
uint16 repeat
```

`frequency_hz == 0` or `on_time_ms == 0` means steady off. Otherwise frequency
shall be 10 through 20000 Hz. A nonzero on-time with zero off-time means a
steady tone. With both timing fields nonzero, `repeat` has the same finite/zero
meaning as `LedCommand`.

### 7.3 `msg/RgbCommand.msg`

```text
uint8 PIXEL_1=1
uint8 PIXEL_2=2
uint8 ALL_PIXELS=3

uint8 update_mask
uint8[2] red
uint8[2] green
uint8[2] blue
```

Bit 1 selects host-owned physical RGB pixel 2. Components are direct 0 through
255 intensities; no gamma correction, fading, or color-space conversion is
part of the public contract. The `PIXEL_1` and `ALL_PIXELS` constants remain in
the message for wire/source compatibility, but RGB pixel 1 is firmware-owned.
An `update_mask` of 1 or 3 is rejected atomically and changes neither pixel.
RGB1's heartbeat/RX/TX meanings and bounded update policy are normative in the
[verified board profile](verified-hardware-profile.md#firmware-owned-status-indicators).

### 7.4 `msg/OledCommand.msg`

```text
uint8 LINE_1=1
uint8 LINE_2=2
uint8 ALL_LINES=3

uint8 update_mask
string<=23 line_1
string<=23 line_2
```

The selected strings shall contain only printable ASCII bytes `0x20` through
`0x7e`. An empty selected string clears the line. Each string is at most 23
bytes, not 23 Unicode code points, and is copied into a fixed 24-byte backing
buffer including the null terminator. The controller-owned battery indication
is rendered on the bottom eight-pixel SSD1306 page (page 3, y=24 through 31),
preserving the legacy display placement; page 2 remains blank. The battery
indication is not writable through ROS.

## 8. Sensor and event interfaces

### 8.1 `msg/ImuState.msg`

```text
builtin_interfaces/Time stamp
float32[3] angular_velocity_rad_s
float32[3] linear_acceleration_m_s2
bool valid
```

Both arrays use x, y, z order and are expressed in the fixed coordinate frame
`imu_link` defined by [hardware-baseline.md](hardware-baseline.md); the frame
name is contractual and is not serialized as a string.
Linear acceleration is in m/s², with raw acceleration in g multiplied by
exactly `9.80665`. Angular velocity is in rad/s, with raw degrees/s multiplied
by π/180. The MCU publishes no orientation estimate or covariance.

The fixed QMI8658 configuration is accelerometer ±4 g and gyroscope
±128 degrees/s, both at 250 Hz with their mode-00 low-pass filters enabled.
CTRL5 is `0x11`, selecting 2.66% of ODR, or approximately 6.65 Hz. Thus
the nominal representable limits after conversion are ±39.2266 m/s² and
±2.234021 rad/s respectively. `SensorTask` consumes data-ready state without a
FIFO backlog and publishes the newest complete accel/gyro sample at 50 Hz. The
ROS API does not change range, ODR, or filtering at runtime. Initialization
reads CTRL5 back and rejects the device configuration if it is not `0x11`.

On a successful QMI8658 read, `valid` is true and `stamp` is the sample time.
On failure, `valid` is false, the two arrays and stamp retain the last valid
sample, and IMU/peripheral error diagnostics increment. Before the first valid
sample, the arrays and stamp are zero.

### 8.2 `msg/ButtonEvent.msg`

```text
uint8 PRESSED=1
uint8 LONG_PRESS=2
uint8 LONG_PRESS_REPEAT=4
uint8 RELEASE_FROM_LONG_PRESS=8
uint8 RELEASE_FROM_SHORT_PRESS=16
uint8 CLICK=32
uint8 DOUBLE_CLICK=64
uint8 TRIPLE_CLICK=128

builtin_interfaces/Time stamp
uint8 button_id
uint8 event
```

`button_id` is 1 or 2. Each message represents one event value, not a bitwise
combination. The internal event queue has 16 entries while the ROS writer
history has depth 8. When the internal queue is full, the oldest queued event
is dropped so the newest physical state transition is retained, and
`button_event_drops` increments.

Both active-low inputs have an absolute, phase-stable 30 ms sampling schedule.
`SensorTask` shortens its notification wait to the next deadline; an early
interrupt wake or a late dispatch never rebases the following deadline, and a
late dispatch performs only one physical sample rather than replaying missed
samples. A raw change becomes debounced after two consecutive samples at the
new level. Event timing is measured from the debounced transition:

- every debounced press emits `PRESSED`;
- a press held for 1500 ms emits `LONG_PRESS`, then emits
  `LONG_PRESS_REPEAT` every 400 ms while held;
- release before `LONG_PRESS` emits `RELEASE_FROM_SHORT_PRESS` followed by
  `CLICK`; release after `LONG_PRESS` emits only `RELEASE_FROM_LONG_PRESS`;
- a second press whose debounced transition occurs within 300 ms after the
  preceding short release emits `PRESSED` followed by `DOUBLE_CLICK`; a third
  such press emits `PRESSED` followed by `TRIPLE_CLICK`; and
- a long press or a gap greater than 300 ms clears the multi-click count.

Threshold detection has one 30 ms scan-period resolution. Timestamps identify
the scan at which the event is emitted and use the common time-sync rules.

### 8.3 `msg/BatteryState.msg`

```text
builtin_interfaces/Time stamp
uint16 voltage_mv
uint16 low_threshold_mv
bool valid
bool below_threshold
```

`voltage_mv` is the filtered board-supply ADC estimate. `valid` is false and
`voltage_mv` is zero when the internal reference is invalid or the computed
supply is below 4900 mV (battery absent) or greater than 20000 mV.
`below_threshold` is the debounced alarm
state, not a comparison of one sample:

- `SensorTask` acquires one VREFINT/PB0 pair on an absolute, phase-stable 50 ms
  schedule and converts it using the STM32 factory VREFINT calibration and the
  release-qualified battery divider gain recorded with D3 evidence; early
  notification wakes and late dispatches do not rebase the following deadline
  or replay missed conversions;
- the first valid conversion initializes the filter, and each later valid
  conversion applies `filtered += 0.05 * (raw - filtered)` before rounding to
  the nearest millivolt;
- an invalid conversion retains the internal last-valid filter and debounced
  alarm state, advances neither debounce timer, and publishes `valid == false`,
  `voltage_mv == 0`, and the retained `below_threshold` state;
- assert after valid filtered voltage remains strictly below the active
  threshold continuously for 10 seconds;
- clear after valid filtered voltage remains at or above
  `min(low_threshold_mv + 200 mV, 20000 mV)` continuously for 2 seconds.

On assertion and once every 10 seconds while asserted, the battery alarm
requests a 2100 Hz buzzer pattern with 800 ms on, 200 ms off, and five cycles.
The active battery pattern has priority over a host `BuzzerCommand`; the latest
validated host pattern is retained and resumes after the alarm pattern. The
battery alarm never changes motor output.

### 8.4 `srv/SetBatteryThreshold.srv`

```text
uint16 threshold_mv
---
mentor_pi_interfaces/Result result
uint16 active_threshold_mv
```

The requested threshold shall be 5000 through 20000 mV. Default after reset is
6300 mV. Reapplying the active threshold is idempotent: it returns `OK` without
resetting debounce timers or changing alarm state. A successful actual change
resets both debounce timers without immediately asserting or clearing the
alarm. The setting is runtime-only. On failure, `active_threshold_mv` reports
the unchanged threshold.

## 9. Time synchronization and session behavior

The MCU synchronizes epoch time with the Agent after session establishment and
before advertising heartbeat state `READY`. The creation-time attempt is
bounded to 20 ms. If it fails, the controller enters `ACTIVE` as `DEGRADED`,
retries for at most 10 ms every 5 seconds, and remains able to control hardware.
After the first success it resynchronizes at least once every 60 seconds; a
failed periodic attempt retains the last valid epoch offset, records the error,
keeps `TIME_SYNCHRONIZED` set, and retries after 5 seconds. Negative offset
corrections are slewed or deferred so stamps on each topic never regress; ROS
time never controls leases or hardware deadlines. Until the first
synchronization succeeds:

- every `builtin_interfaces/Time` stamp is `{sec: 0, nanosec: 0}`;
- heartbeat flag `TIME_SYNCHRONIZED` is clear; and
- lack of time does not stop local control or the 200 ms motor lease, which use
  monotonic MCU time.

`agent_session_id` starts at 1 on the first successful session after MCU boot
and increments after every later entity recreation during that boot, wrapping
from `UINT32_MAX` to 1 so zero remains reserved; it may therefore return to 1
after an MCU reset. The C++ configuration supervisor
treats first discovery, graph/heartbeat reappearance, a changed session ID, or
an uptime discontinuity as a new session. Uptime comparison uses modulo-`uint32`
serial-number arithmetic, so the normal wrap near 49.7 days is forward
progress, not a reset. It immediately keeps host motion disabled, waits for
heartbeat `READY` or `DEGRADED`, and idempotently calls, in order,
`motors/set_model`, `motors/set_adrc`, `pwm_servos/set_offsets`, and
`battery/set_low_threshold`. The ADRC request covers `ALL_MOTORS`, and its
`OK` response must report `applied_mask == ALL_MOTORS`. Each call has a 100 ms host timeout and at most
four attempts in that configuration generation, with 100, 200, then 400 ms
backoff. Only `BUSY`, a returned `TIMEOUT`, or client timeout is retryable.
Every future is tagged with both the host configuration generation and
`agent_session_id`; late responses and responses from an old session are
ignored. Permanent failure or retry exhaustion leaves motion disabled until
operator action or a new session. Host motion may start only after all four
calls are contract-consistent. The supervisor shall not replay bus-servo configuration or any
actuator, LED, buzzer, RGB, or OLED command. Heartbeat `READY` means
MCU/ROS/peripherals are ready; supervisor configuration readiness is host-local.

## 10. Heartbeat and diagnostics

### 10.1 `msg/Heartbeat.msg`

```text
uint8 BOOTING=0
uint8 READY=1
uint8 DEGRADED=2
uint8 FAULT=3

uint16 TIME_SYNCHRONIZED=1
uint16 MOTOR_WATCHDOG_ACTIVE=2
uint16 LOW_BATTERY=4
uint16 IMU_HEALTHY=8
uint16 BUS_SERVO_BUSY=16

builtin_interfaces/Time stamp
uint32 sequence
uint32 uptime_ms
uint32 agent_session_id
uint8 state
uint16 flags
```

`sequence` starts at zero after boot and increments for each attempted
heartbeat publication with uint32 wrap. `uptime_ms` is monotonic MCU uptime
modulo uint32. `MOTOR_WATCHDOG_ACTIVE` is set when any bit of
`watchdog_stop_mask` is set. `DEGRADED` indicates control remains available
with a nonfatal fault; `FAULT` indicates the safety state has disabled normal
output processing.

### 10.2 `msg/ControllerDiagnostics.msg`

All counters are monotonically increasing since boot and saturate at their
type's maximum; they never wrap silently. Fixed-array index constants are part
of the wire contract.

```text
# Subscription indices (arrays of length 7)
uint8 SUB_MOTORS=0
uint8 SUB_PWM_SERVOS=1
uint8 SUB_BUS_SERVOS=2
uint8 SUB_LEDS=3
uint8 SUB_BUZZER=4
uint8 SUB_RGB=5
uint8 SUB_OLED=6
uint8 SUB_COUNT=7

# Task indices (array of length 6)
uint8 TASK_SAFETY_SUPERVISOR=0
uint8 TASK_MOTOR_CONTROL=1
uint8 TASK_MICRO_ROS=2
uint8 TASK_BUS_SERVO=3
uint8 TASK_SENSOR=4
uint8 TASK_PERIPHERAL=5
uint8 TASK_COUNT=6
uint8 TASK_NONE=255

# RAM indices (arrays of length 2)
uint8 RAM_DMA_SRAM=0
uint8 RAM_CCM=1
uint8 RAM_CLASS_COUNT=2

# Peripheral indices (arrays of length 8)
uint8 PERIPH_BUS_SERVO=0
uint8 PERIPH_IMU=1
uint8 PERIPH_OLED=2
uint8 PERIPH_BATTERY_ADC=3
uint8 PERIPH_PWM_SERVO=4
uint8 PERIPH_LEDS=5
uint8 PERIPH_BUZZER=6
uint8 PERIPH_RGB=7
uint8 PERIPH_COUNT=8

# USART1 error indices (array of length 4)
uint8 USART1_FRAMING=0
uint8 USART1_NOISE=1
uint8 USART1_OVERRUN=2
uint8 USART1_PARITY=3
uint8 USART1_ERROR_COUNT=4

# Session states
uint8 SESSION_SAFE_BOOT=0
uint8 SESSION_WAIT_AGENT=1
uint8 SESSION_CREATE_ENTITIES=2
uint8 SESSION_ACTIVE=3
uint8 SESSION_TEARDOWN=4
uint8 SESSION_BACKOFF=5

# Safe-teardown reasons
uint8 TEARDOWN_NONE=0
uint8 TEARDOWN_AGENT_LOST=1
uint8 TEARDOWN_USART1_ERROR=2
uint8 TEARDOWN_RX_OVERRUN=3
uint8 TEARDOWN_TX_TIMEOUT=4
uint8 TEARDOWN_ENTITY_ERROR=5
uint8 TEARDOWN_MEMORY_VIOLATION=6
uint8 TEARDOWN_TASK_STALL=7

# Reset reasons
uint8 RESET_POWER_ON=0
uint8 RESET_PIN=1
uint8 RESET_SOFTWARE=2
uint8 RESET_INDEPENDENT_WATCHDOG=3
uint8 RESET_WINDOW_WATCHDOG=4
uint8 RESET_BROWNOUT=5
uint8 RESET_LOW_POWER=6
uint8 RESET_UNKNOWN=255

# Error-source values
uint8 SOURCE_NONE=0
uint8 SOURCE_TRANSPORT=1
uint8 SOURCE_MOTORS=2
uint8 SOURCE_PWM_SERVOS=3
uint8 SOURCE_BUS_SERVOS=4
uint8 SOURCE_LEDS=5
uint8 SOURCE_BUZZER=6
uint8 SOURCE_RGB=7
uint8 SOURCE_OLED=8
uint8 SOURCE_IMU=9
uint8 SOURCE_BATTERY=10
uint8 SOURCE_EXECUTOR=11
uint8 SOURCE_MEMORY=12

builtin_interfaces/Time stamp
uint64 transport_rx_bytes
uint64 transport_tx_bytes

uint32 uptime_ms
uint32 session_generation
uint32 agent_reconnects

uint32 command_messages
uint32 command_rejections
uint32[7] mailbox_overwrites
uint32 button_event_drops
uint32 publication_errors

uint32 service_requests
uint32 service_completions
uint32 service_busy_rejections
uint32 service_timeouts
uint32 service_partial_results
uint32 late_response_drops

uint32[4] motor_lease_expiries
uint32[4] motor_command_rejections
uint32 motor_watchdog_trips
uint32 motor_command_consumptions
uint32 motor_command_age_over_20_ms
uint32 motor_command_max_age_us
uint32 executor_overruns

uint32[8] peripheral_errors
uint32[8] peripheral_timeouts
uint32[4] usart1_errors
uint32 usart1_rx_dma_high_water_bytes
uint32 transport_rx_overruns
uint32 transport_tx_timeouts
uint32 maximum_transport_wait_us

uint32[6] task_missed_releases
uint32[6] task_max_execution_us
uint32[6] task_stack_high_water_bytes
uint32[6] task_heartbeat_age_ms
uint32[2] free_ram_bytes
uint32[2] minimum_free_ram_bytes
uint32 flash_used_bytes
uint32 flash_total_bytes
uint32 post_seal_allocation_attempts
uint32 last_error_uptime_ms

uint16 last_error_detail
uint8 session_state
uint8 last_teardown_reason
uint8 last_reset_reason
uint8 last_watchdog_task
uint8 last_error_code
uint8 last_error_source
```

Maximum serialized-size budget:

| CDR component | Count | Bytes |
|---|---:|---:|
| `builtin_interfaces/Time` | 1 | 8 |
| `uint64` fields | 2 | 16 |
| `uint32` scalars and fixed-array elements | 89 | 356 |
| `uint16` fields | 1 | 2 |
| `uint8` fields | 6 | 6 |
| **Maximum CDR payload** | | **388 bytes** |
| CDR encapsulation header | 1 | 4 |
| **Maximum serialized CDR size** | | **392 bytes** |

Constants consume no serialized bytes. The schema has no string or sequence,
so 392 bytes is a hard maximum, not a typical-size estimate. The encapsulation
header precedes the message-field payload and resets its alignment origin; it
does not introduce padding before the first `uint64`. Generated ROS 2 Humble
type support is normative for field layout and shall report a 388-byte maximum
message-field payload; an actual serialized CDR buffer including its four-byte
encapsulation shall be 392 bytes. CDR plus XRCE message/submessage headers shall
be proven no greater than the fixed
512-byte transport MTU; the 120-byte difference is not assigned to serial
framing. HDLC-style serial framing, CRC, and byte stuffing occur after the XRCE
MTU in a bounded framing buffer that may flush multiple custom-write callbacks.
Each callback shall fit the 1 KiB TX bounce buffer, and the complete escaped
wire count remains subject to the 70,000 bytes/s gate. This diagnostics sample
shall not require XRCE fragmentation.

Semantics:

- `session_generation` equals the current `Heartbeat.agent_session_id`.
  `agent_reconnects` counts successful entity generations after the first one
  during this boot.
- `command_messages` counts subscription callbacks that reached validation;
  `command_rejections` counts atomically rejected topic messages.
- `mailbox_overwrites` counts an accepted latest value replaced before its
  consumer observed it; motor/PWM per-channel merging increments once per
  message when any selected prior generation was unconsumed.
- `publication_errors` is the aggregate count of non-OK publish attempts.
  Reliable ACK behavior is verified externally because the pinned public
  micro-ROS API does not expose a truthful retry counter.
- Service counters aggregate all seven services. Every taken request increments
  `service_requests`; every response successfully handed to the current ROS
  session increments `service_completions`. Busy, timeout, and partial counters
  describe the returned result and may also be completions. A completion after
  its owner/session expired increments `late_response_drops` and is not sent
  into a new session.
- `motor_lease_expiries` is per physical motor. `motor_command_rejections`
  increments for each valid selected motor in an atomically rejected command;
  a malformed or zero mask only increments the subscription-level rejection.
- `motor_command_consumptions` increments once when `MotorControlTask` consumes
  a current-session mailbox snapshot containing at least one fresh field that
  the motor controller accepts. One sample is the greatest unsigned
  `consumed_at_us - accepted_at_us` age among those fresh accepted fields;
  rejected fields and stale/session-discarded snapshots produce no sample.
  `motor_command_age_over_20_ms` counts only samples strictly greater than
  20,000 us, so exactly 20,000 us passes. `motor_command_max_age_us` is the
  monotonic since-boot high-water value. The two counts saturate rather than
  wrap, and microsecond subtraction follows the same unsigned clock-wrap rule
  as motor leases.
- `peripheral_errors` and `peripheral_timeouts` cover active hardware owners;
  bus-servo index 0 corresponds to UART5. `usart1_errors` separately preserves
  all four transport hardware error classes.
- `transport_rx_bytes` and `transport_tx_bytes` count bytes on the USART1 wire,
  including XRCE and serial framing. Verification derives rate from counter
  deltas: the combined RX plus TX delta over every complete one-second
  measurement window shall be less than 70,000 bytes.
- `maximum_transport_wait_us` is the longest completed bounded transport wait
  since boot. The public pinned micro-ROS API exposes no truthful retry or
  reliable-stream occupancy metric, so neither is claimed in this message;
  verification instead uses the fixed configured history and external fault
  injection/capture.
- `task_missed_releases`, maximum execution time, minimum unused stack, and
  heartbeat age sampled when diagnostics is built use the fixed task-index
  mapping. Stack values are in bytes rather than FreeRTOS words.
- RAM index 0 is DMA-accessible SRAM at `0x20000000` (128 KiB); index 1 is CCM
  at `0x10000000` (64 KiB). Free values are measured against the statically
  partitioned pools/linker layout, not a general-purpose runtime heap.
- `post_seal_allocation_attempts` counts any allocate, reallocate, or deallocate
  callback made while the `ACTIVE` seal is set and shall remain zero in a
  conforming run.
- `last_watchdog_task` describes only the previous boot. It is a retained task
  index when `last_reset_reason == RESET_INDEPENDENT_WATCHDOG` and the prior
  boot committed a valid first stale-task record; otherwise it is `TASK_NONE`.
  A current-boot stall is persisted for the next boot but shall not change this
  published field before reset. The retained record is exactly three 32-bit
  words: magic `0x52525732`, payload, and bitwise-complement payload. Payload
  version 1 occupies bits 31:24, bits 23:8 are zero, and bits 7:0 contain a
  task value below `TASK_COUNT`. Firmware invalidates magic first and publishes
  it last; boot rejects torn, malformed, wrong-version, reserved-bit, and
  out-of-range records, consumes the record unconditionally, and exposes it
  only when RCC reports an independent-watchdog reset.
- `last_error_code` uses `Result.code`; source identifies the subsystem. The
  record updates only when a new non-OK condition occurs.

## 11. Overload, disconnect, and stale-work semantics

- Only `MicroRosTask` owns or calls `rcl`, `rclc`, `rmw`, or micro-ROS APIs.
  Service requests/responses are manually taken/sent there; hardware workers
  return bounded completion records.
- The motor loop runs at 100 Hz, and lease evaluation runs at 1 kHz regardless
  of ROS traffic. At 500 motor commands/s, callbacks merge the newest selected
  values rather than queueing 500 work items/s.
- On ordinary Agent loss, all motors are disarmed. PWM and bus servos hold
  their last valid target and torque state. Discrete pattern/output state also
  remains local, but is never replayed into a new ROS session.
- RX overrun, any USART1 framing/noise/overrun/parity error, or transport TX
  timeout enters safe teardown immediately. UART5, I2C, SPI, ADC, and retained
  peripheral failures remain localized to their owning worker unless they
  independently stall a supervised task. No callback or worker may continue
  using an entity generation after teardown begins.
- Every queued service/completion record carries the current session
  generation. A completion for an older generation is counted as late and
  dropped. Every latest-value command is invalidated at teardown. Consequently
  no pre-disconnect command or response can be applied or sent after reconnect.
- Detailed fault transitions and actuator safe states are normative in
  [reliability-and-safety.md](reliability-and-safety.md).

## 12. Static-memory and code-generation constraints

- Custom arrays are fixed, and the only custom strings are the two bounded
  `string<=23` OLED fields.
- Generated message storage, executor handles, XRCE streams, seven service
  request/response message stores, four pending-work slots (three non-bus and
  one shared by the three bus services), seven subscription stores, seven
  publisher messages, and worker records are allocated before the runtime
  allocation seal.
- Bounded strings are initialized with their maximum capacity before use;
  callbacks copy after validating length and never call a resize operation.
- The interface package shall be supplied to the micro-ROS firmware build so
  host and MCU code are generated from the same IDL files. Handwritten duplicate
  wire structs are forbidden.
- The MCU generator shall select only
  `rosidl_typesupport_microxrcedds_c` and shall set
  `ROSIDL_GENERATOR_C_DISABLE_TYPE_DESCRIPTION_CODEGEN=ON` globally while
  cross-building packages. This removes unused runtime type-description
  construction, not serialization support or any field from the wire schema.
  Host-side Humble generation remains unchanged.

## 13. Related documents

- [requirements.md](requirements.md) — stable product and acceptance
  requirements.
- [hardware-baseline.md](hardware-baseline.md) — physical channel mapping,
  buses, pin ownership, and electrical limits.
- [architecture.md](architecture.md) — task ownership, queues, entity life
  cycle, and host supervisor.
- [reliability-and-safety.md](reliability-and-safety.md) — fault taxonomy,
  safe states, watchdogs, and reconnect behavior.
- [legacy-audit.md](legacy-audit.md) — legacy-to-v2 disposition; it is not a
  compatibility contract.
- [development-standards.md](development-standards.md) — C++/embedded style and
  static-analysis rules.
- [verification.md](verification.md) — contract and requirement acceptance
  tests.
