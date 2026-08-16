# RRCLite controller integration

This directory is the allocation-free application layer between the portable
domain/drivers, the micro-ROS runtime, and the STM32 target. It contains no
STM32 HAL, CMSIS, or FreeRTOS headers. `ControllerRuntime` is the sole owner of
all controller state and exposes bounded iterations for native tests plus
non-returning task entries for production.

## Ownership and timing

| Owner | Work | Normal bound |
| --- | --- | --- |
| `SafetySupervisorTask` | Peer liveness, emergency motor stop, sole watchdog refresh | 20 ms period; 250 ms boot grace |
| `MotorControlTask` | Per-channel lease checks, encoder sampling, ADRC, motor output | Lease check at most every 2 ms; ADRC every tenth release |
| `MicroRosTask` | ROS executor, services, telemetry, USART1 transport | Supplied by `app/microros` |
| `BusServoTask` | UART5 frame sequencing and all bus-servo services | At most 10 ms wait between iterations |
| `SensorTask` | IMU, buttons, battery, battery-threshold service | At most 4 ms wait between iterations |
| `PeripheralTask` | PWM servos, LEDs, buzzer, RGB, OLED | At most 1 ms wait between iterations |

Callbacks validate and copy into fixed-capacity mailboxes or service slots.
Only an owner task mutates its domain controller or calls its hardware hooks.
Motor, PWM, LED, buzzer, RGB, OLED, and bus-motion work is tagged with the ROS
session generation. Teardown invalidates unread work. Active motor authority is
removed immediately; PWM pulses are frozen at the physically committed shadow;
bus and RGB operations from the old generation are canceled by their owners.

## Target startup

The production target performs these steps before starting the scheduler:

1. Initialize platform clocks, safe GPIO latches, peripherals, and DWT time.
2. Build a complete `PlatformHooks` table whose context has static lifetime.
3. Construct the explicit locked/commissioning `MotorControlConfiguration`,
   then call `ControllerInstance(motor_configuration)` and
   `Configure(hooks, imu_transform)`.
4. Call `InitializeSafeBoot()`; failure is a fail-stop condition.
5. Pass `BuildMicroRosHooks()` to `ConfigureMicroRosRuntime()`.
6. Call `BuildTaskEntries({&MentorPiMicroRosTaskMain, nullptr})`, convert the
   entries by index to STM32 `TaskHooks`, and create all six static tasks.
7. Start encoder/control timing, then start the scheduler.

The complete binding is implemented in `target/stm32/main.cc`. Task enum values
are deliberately identical between this layer, diagnostics, and the platform;
the target retains compile-time count checks when converting the arrays.

## `PlatformHooks` contract

- Every pointer checked by `PlatformHooksAreComplete()` is mandatory. Hooks are
  copied during configuration and shall not change afterward.
- Millisecond and microsecond clocks are monotonic modulo `uint32_t`; ROS epoch
  time must never feed actuator deadlines.
- Driver deadlines are absolute. Target glue converts them to the remaining
  bounded STM32 timeout and caps individual I2C calls at 10 ms.
- `wait_for_task` may return early on a coalesced notification and shall never
  wait longer than its argument.
- Critical sections must prevent task preemption and relevant frame interrupts,
  support nesting with platform register helpers, perform no allocation, and
  never block on hardware. Hooks called while critical are documented
  register-only or nonblocking admission/poll operations.
- `emergency_stop_motors` is idempotent, register-only, and safe from task or
  interrupt context. It zeros all motor compares and clears hardware authority.
- Encoder snapshots are raw `uint32_t`. TIM5/TIM2 retain 32 bits and TIM4/TIM3
  zero-extend 16 bits; the domain performs width-aware modular deltas.
- Motor duty is signed permille. Applying a multi-motor output must fail safe:
  any channel error zeros and disarms all four outputs.
- `pwm_servo_frame_sequence` increments only when the ISR begins a common frame
  and commits the complete shadow. Shadow copies are atomic with that ISR.
  `PeripheralTask` prepares B0 before that interrupt, advances interpolation
  only after observing the increment, and prepares B1 for the next interrupt.
  PWM telemetry and a changed-offset service complete from the committed frame,
  never merely from a task-side shadow write.
- Button values and LED values are semantic (`pressed` and `on`). The adapter
  contains the board polarity conversion expected by `GpioPeripheralDriver`.
- The buzzer hook returns a `Result`. Safe boot must successfully command the
  buzzer off before task creation; later failures are counted in the buzzer
  diagnostics slot and reported with `ErrorSource::kBuzzer`.
- Transient QMI8658 data-not-ready responses are normal between ODR releases.
  A software-I2C START that finds SDA held low first performs the standard,
  bounded nine-clock bus-clear sequence and retries the transaction. If IMU
  initialization remains busy across the 500 ms diagnosis interval,
  `SensorTask` reports an IMU timeout with the attempted identity address as
  its detail (normally `0x6a`) and republishes invalid IMU telemetry on each
  one-second retry so the fault remains visible through ROS diagnostics.
  If `STATUS0` remains not ready continuously for 500 ms, `SensorTask` reports
  an IMU timeout with detail 46, publishes invalid IMU telemetry, soft-resets
  the sensor, waits the documented 15 ms reset interval without blocking, and
  reapplies the production configuration. Host consumers remain fail closed
  until a valid post-reset sample arrives.
- UART5 and SPI operations admit at most one transfer. Poll is bounded; cancel
  returns the peripheral to an idle state without queuing another transfer.
- Stack high-water values are unused bytes, not FreeRTOS stack elements. Memory
  metrics report DMA SRAM and CCM in that order.

## Static-lifetime rule

`ControllerInstance()` constructs one process-lifetime runtime. The five
polymorphic driver adapters are embedded in it and can never be dynamically
allocated or deleted. Driver interfaces use protected nonvirtual destructors,
so adapter vtables contain no deleting destructor and import no global
`operator delete`.

## Qualification constraints

- The STM32 composition root applies the six-face measured QMI8658 transform:
  PCB X = sensor Y, PCB Y = -sensor X, and PCB Z = sensor Z. The driver applies
  the same signed permutation to acceleration and angular velocity. Positive
  rotation and extended timing HIL remain release-qualification checks.
- `MotorControlConfiguration{}` and
  `DefaultAdrcMotorControlConfiguration()` describe the same production ADRC
  configuration: a 6 RPS implementation ceiling and a 1000-permille output
  limit. The active model can impose a lower RPS limit. Invalid configuration
  values fail closed: nonzero selected targets return `UNSUPPORTED`, cannot
  arm, and cannot refresh a lease, while selected zero targets remain valid
  stops. There is no alternate firmware motor mode.
- Encoder direction and all ADRC/filter/minimum-drive values are
  release-provisional. All models retain the legacy raw-counter encoder factor
  `+1`, multiplied by each channel's wiring sign. JGA27 separately applies
  drive-output factor `-1`, corresponding to its legacy negative PID gains;
  the other models use drive-output factor `+1`. These are unqualified until
  guarded raised-wheel control tests record each profile.
- The controller uses the platform's pulse-shadow generator rather than the
  separate driver edge-plan abstraction; there must be only one PWM frame
  generator in the target.
- While a ROS session is active, an unexpected PWM mailbox session-tag mismatch
  is reported in peripheral slot 4 as `IO_ERROR` detail `0x5001`; a snapshot
  generation mismatch uses detail `0x5002`. Expected unread work from a prior
  session remains a silent discard. These diagnostics use the ROS telemetry
  path and never write to the Agent-owned USART1 transport.

## Native verification

From the repository root:

```sh
cmake -S firmware/mentor_pi_mcu/app/controller \
  -B build/controller-test -G Ninja \
  -DBUILD_TESTING=ON \
  -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_DRIVER_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_CONTROLLER_SANITIZERS=ON
cmake --build build/controller-test
ctest --test-dir build/controller-test --output-on-failure
```

The tests cover safe boot, complete micro-ROS hook wiring, 198 ms lease expiry,
the locked/capped motor gate, immediate session disarm, physical PWM B0/B1
commits for 20/21/39/41 ms commands, exact PWM3 500 ms interpolation with
repeated commands and session transitions, offset completion at its committing
ISR, PWM freeze, bus/RGB stale-work invalidation, button events, task
supervision, and startup grace under ASan/UBSan. They do not replace the pinned
Arm link/resource audit or HIL.
