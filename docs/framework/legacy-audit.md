# Legacy Audit and v2 Disposition

## Purpose

This document records what the rewrite preserves, replaces, or deliberately
excludes. The legacy code is evidence of hardware behavior, not a specification:
unsafe, broken, or accidental behavior shall not be reproduced.

The authoritative v2 behavior is defined by the
[requirements](requirements.md), [ROS interface contract](ros-interface-contract.md),
[architecture](architecture.md), and
[reliability and safety contract](reliability-and-safety.md). Verification IDs
below are defined in [verification.md](verification.md); physical claims are
traced in the [hardware baseline](hardware-baseline.md).

## Evidence and audit rules

The audit used:

- `docs/reference/ros_robot_controller-ros2/src/ros_robot_controller/` for the
  Python node and serial SDK;
- `docs/reference/ros_robot_controller-ros2/src/ros_robot_controller_msgs/` for
  every legacy ROS message and service;
- `docs/reference/RosRobotControllerLite_ros_250811/Hiwonder/` for packet
  handlers, device drivers, and application tasks;
- the legacy `Core/` tree and `RosRobotControllerM4.ioc` for tasks, pins,
  timers, DMA, and USART configuration; and
- the three hardware PDFs under `docs/reference/` for board evidence.

Disposition terms are normative:

- **Replace**: v2 supplies the accepted user-facing capability through a new,
  bounded contract.
- **Retain internally**: required platform behavior with no direct ROS command.
- **Exclude**: deliberately absent from v2. A residue source file or advertised
  but nonfunctional legacy endpoint does not establish support.

v2 is a clean break: it exposes none of the old node/topic/service names,
`ros_robot_controller_msgs` types, or `0xAA 0x55` protocol. The MCU ROS node is
`/mentor_pi/controller`; endpoint names below are relative to `/mentor_pi`.

## Legacy ROS-visible functions

This table covers every publisher, subscription, and service created by
`ros_robot_controller_node.py`.

| Audit ID | Legacy ROS API and packet | Legacy MCU path | v2 replacement | Defect and required disposition | Verification |
|---|---|---|---|---|---|
| ROS-LED-01 | `~/set_led` (`LedState`) → function 1, `id,on_ms,off_ms,repeat` | `packet_led_handle()` → LED pattern timer → PD9/PD10; PD11 also indicates packets | `leds/command` (`LedCommand`), LED2/LED3 only | Legacy ownership was inconsistent and payload size was unchecked; float seconds silently narrowed to 16-bit milliseconds. v2 validates first, reserves LED1 for the system heartbeat, exposes LED2/LED3, and rejects host attempts to override LED1. | `VER-HIL-LED-001`, `VER-UNIT-VAL-001` |
| ROS-BUZ-01 | `~/set_buzzer` (`BuzzerState`) → function 2, `frequency,on_ms,off_ms,repeat` | `packet_buzzer_handle()` → buzzer timer → PA6 | `buzzer/command` (`BuzzerCommand`) | No complete length/range validation; conversions can wrap; no acknowledgement/error path. v2 validates atomically and makes rejection/I/O failure observable. | `VER-HIL-BUZ-001`, `VER-UNIT-VAL-001` |
| ROS-MOT-01 | `~/set_motor` (`MotorsState`) → function 3/subcommand 1, repeated zero-based ID + float RPS | `packet_motor_handle()` → lazy `motors_init()` → TIM1/9/10/11 PWM and TIM2/3/4/5 encoders | `motors/command`, `motors/state`, `motors/set_model` | Host subtracts one without checking; MCU indexes without bounds checks; float64 narrows; handler bypasses the RPS clamp; initialization occurs in receive context; motor 4 starts the wrong encoder timer; no lease exists. The legacy PID computes but does not use its proportional error delta and does not accumulate its named integral term, so its formula shall not be copied as a reference controller. v2 uses exactly four fixed entries, verified initialization, model limits, independent 200 ms leases checked at 1 kHz, and a corrected but provisional controller behind a default motor lock. JGA27's provisional `-1` polarity derives from legacy negative-gain evidence and, like every PID profile, requires passive direction checks and guarded HIL before release. | `VER-BUILD-MOTOR-GATE-001`, `VER-HIL-MOT-001`, `VER-SAFE-LEASE-001`, `VER-LOAD-500-001` |
| ROS-PWM-01 | `~/pwm_servo/set_state` (`SetPWMServoState`) → function 4/subcommands 1 and 7 | `packet_pwm_servo_handle()` → software interpolation → PA11/PA12/PC8/PC9 | `pwm_servos/command`, `pwm_servos/set_offsets` | Optional fields are flag/value arrays; ID zero can index before the array; count/length checks are absent; concurrent writes can interleave. The handler forwards unchecked durations and relies on downstream clamping before `duration/20` arithmetic. v2 uses four fixed channels, atomic validation/merge, and one bounded offset service. | `VER-HIL-PWM-001`, `VER-UNIT-VAL-001` |
| ROS-PWM-02 | `~/pwm_servo/get_state` (`GetPWMServoState`) → function 4/subcommands 5 and 9 | Report generated from cached PWM duty/offset | `pwm_servos/state` | `GetPWMServoCmd.msg` has no `id` but Python reads `i.id`; callback signature/return are invalid; SDK uses the wrong mutex and waits without a deadline. The broken request path is removed in favor of bounded state publication. | `VER-API-001`, `VER-HIL-PWM-001` |
| ROS-BUS-01 | `~/bus_servo/set_state` (`SetBusServoState`) → function 5 write/move subcommands | `packet_serial_servo_handle()` → UART5 half-duplex bus | `bus_servos/command`, `bus_servos/configure`, `bus_servos/stop` | Unbounded optional arrays become unchecked counts/payloads; unaligned integer casts are used; slow UART work runs inline in RX context; writes have no typed result. v2 bounds a move to 16 unique IDs, uses one replace-latest move slot, validates before the first write, and serializes work in `BusServoTask`. | `VER-HIL-BUS-001`, `VER-UNIT-SVC-001`, `VER-UNIT-VAL-001` |
| ROS-BUS-02 | `~/bus_servo/get_state` (`GetBusServoState`) → function 5 read subcommands | UART5 request/reply → function 5 report | `bus_servos/get_state` | Replies have no transaction correlation; waits have no timeout; Python calls nonexistent voltage/torque methods and reports success unconditionally. v2 permits one in-flight bus service, tags generations, returns `Result`/valid masks, and enforces 200 ms MCU/250 ms host deadlines. | `VER-UNIT-SVC-001`, `VER-FAULT-BUS-001` |
| ROS-IMU-01 | `~/imu_raw` (`sensor_msgs/Imu`) ← function 7, six little-endian floats | `imu_task_entry()` → QMI8658 software I2C → 50 Hz report | `imu` (`mentor_pi_interfaces/ImuState`), 50 Hz | Data has no acquisition timestamp; host publishes an all-zero quaternion without correctly representing validity; drops are silent. The driver forwards raw sensor axis order without proving its physical PCB frame. v2 uses fixed three-element gyro/acceleration arrays, synchronized time, explicit `valid`, SI units, and a measured signed transform into the fixed `imu_link` contract without an orientation/covariance/string field. | `VER-HIL-IMU-001`, `VER-API-001` |
| ROS-BTN-01 | `~/button` (`ButtonState`) ← function 6, ID + event bits | two button timers → PE0/PE1 → report | `buttons/events` (`ButtonEvent`) | MCU can report eight distinct events, but host translates only press/click; ROS depth one silently loses bursts. v2 preserves all eight event values, uses an internal FIFO 16/drop-oldest policy, publishes with ROS history 8, and counts loss. | `VER-HIL-BTN-001`, `VER-OVERFLOW-BTN-001` |
| ROS-BAT-01 | `~/battery` (`std_msgs/UInt16`) ← function 0/subcommand 4 + millivolts | PB0/ADC1 → filter/alarm timer → 1 Hz report | `battery/state`, `battery/set_low_threshold` | Type carries no stamp/unit/validity; replaced samples are invisible; threshold control exists only in the packet protocol; alarm arbitration is implicit. v2 gives explicit millivolts/validity, bounded service validation, exact debounce, and deterministic buzzer priority. | `VER-HIL-BAT-001`, `VER-UNIT-SVC-001`, `VER-SAFE-BAT-001` |
| ROS-JOY-01 | `~/joy` (`sensor_msgs/Joy`) ← nominal function 8 | No producer in the active firmware build | **Exclude** | Python advertises the topic, but USB-host/gamepad sources are outside the active Keil project and no active code sends function 8. v2 shall not advertise Joy. | `VER-SCOPE-001` |
| ROS-SBUS-01 | `~/sbus` (`ros_robot_controller_msgs/Sbus`) ← nominal function 9 | Standalone partial `sbus_porting.c`, not integrated | **Exclude** | Required UART, RTOS objects, and task are absent. An old advertised topic is not hardware support. v2 shall not advertise SBUS. | `VER-SCOPE-001` |
| ROS-NODE-01 | Node `ros_robot_controller`, private names, `imu_frame` parameter, Python launch | `rclpy` bridge opens `/dev/ttyACM0` and runs a custom serial thread | Native Agent plus MCU `/mentor_pi/controller` and compiled C++ configuration supervisor | Device path/baud are hard-coded; startup throws; shutdown does not stop motors or reliably join RX; thread reads one byte per call. v2 defaults to `/dev/mentor_pi_mcu`, has no Python runtime bridge, and follows the bounded session/safe-state machine. | `VER-API-001`, `VER-CONFIG-001`, `VER-RECONNECT-USB-001` |

## Active MCU hardware and platform paths

These rows include active paths that the legacy host never exposed.

| Audit ID | Confirmed active legacy path | v2 disposition | Defect or required correction | Verification |
|---|---|---|---|---|
| HW-TRN-01 | Data USB-C → CH9102F → PA9/PA10 USART1 at 1,000,000 baud, 8N1; normal-mode DMA/custom frames | **Replace** with micro-ROS XRCE serial, 8 KiB circular RX DMA, 1 KiB TX bounce, MTU 512 | RX FIFO result is ignored; DMA receive is aborted/restarted in callbacks; overflow is silent; callbacks execute application work. V2 uses the standard HAL circular-DMA handler and half/full callbacks, a stable epoch plus `NDTR` cursor, bounded task reads, explicit overrun detection, and fail-closed error handling. | `VER-UNIT-RXDMA-001`, `VER-INT-TRN-001`, `VER-FUZZ-TRN-001`, `VER-RECONNECT-USB-001` |
| HW-MOT-01 | Four YX-4055AM outputs, four quadrature encoders, 100 Hz PID | **Replace** through v2 motor APIs, with normal images locked to zero/stop until HIL | In addition to ROS-MOT-01, output must be safe before driver initialization and on lease/session failure. Passive encoder direction must be proven before the acknowledged direction-check image is used. That image bypasses PID, drives fixed 250 permille, and cuts off above 0.50 measured RPS; its evidence does not qualify PID or regulated speed. | `VER-BUILD-MOTOR-GATE-001`, `VER-HIL-MOT-001`, `VER-SAFE-LEASE-001` |
| HW-PWM-01 | Four PWM-servo outputs on PA11/PA12/PC8/PC9 | **Replace** through v2 PWM APIs | Handler validation is incomplete and downstream clamping is an undocumented prerequisite. v2 validates locally and applies multi-channel updates atomically. | `VER-HIL-PWM-001`, `VER-UNIT-VAL-001` |
| HW-BUS-01 | UART5 one-wire half-duplex servo bus on PC12 | **Replace** through v2 bus APIs | Slow request/reply blocks parsing and lacks correlation. Only `BusServoTask` owns UART5 in v2. | `VER-HIL-BUS-001`, `VER-FAULT-BUS-001` |
| HW-LED-01 | LEDs 1 and 2 are active-low on PD9/PD10; LED3 is active-high on PD11 | **Replace** with firmware-owned LED1 and ROS-owned LED2/LED3 | Legacy packet indication conflicted with user ownership. v2 gives LED1 explicit time-based system-heartbeat ownership and exposes LED2/LED3 to ROS. | `VER-HIL-LED-001` |
| HW-BUZ-01 | PWM buzzer on PA6 | **Replace** through `buzzer/command` | Values can wrap and alarm/host cancellation semantics are implicit. v2 uses validated patterns and defined battery arbitration. | `VER-HIL-BUZ-001`, `VER-SAFE-BAT-001` |
| HW-RGB-01 | Two WS2812-compatible pixels through SPI1; function 11 → `packet_RGB_Ctl_handle()` | **Replace** with firmware heartbeat/RX/TX status on RGB1 plus ROS-owned RGB2 | No legacy ROS endpoint; pixel mask/payload are unchecked; Python function enum is inconsistent. v2 keeps the fixed two-pixel wire shape, accepts only mask 2, and rejects attempts to override RGB1. | `VER-HIL-RGB-001`, `VER-UNIT-VAL-001` |
| HW-OLED-01 | SSD1306-compatible 128×32 OLED on I2C1; `ENABLE_OLED=1`; function 10 → `packet_oled_handle()` and OLED task | **Replace** through `oled/command`, two printable-ASCII strings of at most 23 bytes | Legacy length can overrun 24-byte buffers; I2C can wait indefinitely; no legacy ROS endpoint. v2 bounds text and I/O time while retaining the controller-owned battery line. | `VER-HIL-OLED-001`, `VER-FAULT-I2C-001` |
| HW-BTN-01 | Two active-low buttons on PE0/PE1, 30 ms legacy scan | **Replace** through `buttons/events` | Event loss and debounce are not observable. v2 preserves types/order and exposes overflow. | `VER-HIL-BTN-001`, `VER-OVERFLOW-BTN-001` |
| HW-IMU-01 | QMI8658 at 0x6A/0x6B over software I2C PB10/PB11, interrupt PB12, 50 Hz | **Replace** through `imu` | Long blocking initialization/read paths are not bounded. `SensorTask` owns the device; failure cannot block safety/transport. | `VER-HIL-IMU-001`, `VER-FAULT-I2C-001` |
| HW-BAT-01 | PB0/ADC1 voltage divider/filter, function 0 report/`packet_battery_limit_handle()`, and local low-voltage buzzer alarm | **Replace** through battery APIs; retain local alarm behavior | Threshold is unchecked and buzzer ownership conflicts. v2 validates threshold and defines exact debounce/arbitration. | `VER-HIL-BAT-001`, `VER-SAFE-BAT-001` |
| HW-WDG-01 | IWDG refreshed by general application/RGB paths | **Retain internally**, solely refreshed by `SafetySupervisorTask`; publish reason/counters | An unrelated live task can hide a critical stall and reset cause is not visible. v2 refreshes only after required heartbeats and retains the offender. | `VER-SAFE-WDG-001`, `VER-RESET-MCU-001` |
| HW-MEM-01 | Dynamic LwMem/FreeRTOS allocations, 2 KiB RX FIFO, 64-pointer TX queue | **Replace** with static transport storage, fixed mailboxes/FIFO/service slots, and bounded micro-ROS arena/pools | Full TX queue leaks frames; RX/telemetry losses are silent; heap exhaustion can permanently stop transport. v2 seals ACTIVE allocation and returns all arena/pool baselines after recreation. | `VER-RESOURCE-001`, `VER-OVERFLOW-001`, `VER-SOAK-001` |

## Explicit exclusions

| Feature | Classification | Evidence and boundary |
|---|---|---|
| Gamepad | **Excluded** | USB HID source residue exists, but the active build has no USB-host peripheral/task or function 8 producer. `sensor_msgs/Joy` is absent from v2. |
| SBUS | **Excluded** | A partial port exists, but its UART, semaphores, event flags, and task are absent from the active project. |
| Bluetooth | **Excluded** | A battery-source comment says “Bluetooth report,” but no radio, driver, protocol, transport, or task exists. |
| USB host | **Excluded** | The communication USB-C terminates at CH9102F; STM32 USB-host is not configured. Middleware residue is not active support. |
| Native MCU USB | **Excluded** | PA11/PA12, normally USB FS D−/D+, are PWM-servo GPIO on this board. Communication uses CH9102F/USART1. |
| Power-only USB-C | **Excluded from communication** | The second USB-C is the regulated 5 V/5 A host-computer power output and has no UART/data path. |
| LCD/LVGL | **Excluded** | `ENABLE_LVGL=0` and no active LCD controller/contract exists. The separate included I2C OLED is not an LCD/LVGL feature. |
| High-level chassis | **Excluded** | Differential/mecanum helpers are residue; `chassis_init()`/motion use are disabled and the old node has no `cmd_vel`. v2 exposes four motors directly. |

An excluded feature can be added only by a requirements change and ADR. It
shall not be enabled opportunistically during the rewrite.

## Systemic defects that shall not cross into v2

1. Validate the complete frame/message, mask, sequence/count, ID, enum, range,
   duration, finite float, and cross-field invariant before indexing or changing
   state.
2. ISRs and middleware callbacks shall not execute device transactions, wait on
   locks, or perform application work; they may copy bounded data, timestamp,
   and notify the owning task.
3. Every mailbox, FIFO, history, slot, and arena shall be statically bounded
   with the documented overwrite/drop/busy policy and observable counters.
4. Stale-dangerous commands use latest-value state and explicit leases.
   Request/reply/configuration uses one owner, generation correlation, and a
   finite deadline.
5. While the ACTIVE allocation seal is set, firmware shall not allocate or
   deallocate. Entity recreation may use/finalize/reset only the fixed arena
   during `CREATE_ENTITIES`/`TEARDOWN` and shall return every pool to baseline.
6. Services return the standard `Result` model; transport receipt alone is not
   hardware success except where the contract explicitly defines transmitted
   write-only success.
7. Reset reason, transport loss, overflow, rejection, deadline, stale response,
   watchdog, and hardware I/O failure shall be observable.

## Audit closure rule

An included row closes only when every listed verification case passes. Every
exclusion must be absent under `VER-SCOPE-001`. A newly discovered active
legacy path requires a stable audit ID, explicit disposition, and verification
mapping before implementation proceeds.
