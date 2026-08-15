# PWM3 TIM13 incident report — 2026-08-15

## Outcome

PWM3 was observed stuck at 1500 microseconds on `ackermann_1` because the
STM32 TIM13 software frame generator never produced its first update
interrupt. The `mecanum_2` artifact contained the same deterministic startup
path, although a separate pre-fix Mecanum trace was not recorded. That startup
path requested a one-microsecond delay and programmed `ARR=0`. STM32F4
reference manual RM0090 specifies that the timer counter is blocked while the
auto-reload value is zero.

Commit `616dbca` preserves a one-microsecond interval without using `ARR=0`:
TIM13 is programmed with `ARR=1` and `CNT=1`, so the next tick of the 1 MHz
timer produces the update event. The normal 50 Hz PWM frame timing, PC8/PWM3
mapping, ROS interfaces, QoS, micro-ROS SDK, command semantics, and safety
behavior are unchanged.

The fix passed focused software tests and power-disconnected HIL on both
robots. This closes the firmware state-generation defect. It is not an
oscilloscope measurement of PC8 and does not qualify an energized actuator.

## Observed symptom

With subscribers discovered before publication, a PWM3 command using mask
`0x04`, target 1600 microseconds, and duration 500 ms was received without a
command rejection or mailbox overwrite. Nevertheless:

- `/ackermann_1/pwm_servos/state` continued reporting target and output 1500;
- `moving_mask` remained zero;
- the PWM peripheral-error counter did not increase; and
- repeated publications did not change the result.

An advancing ROS timestamp initially appeared to show that PWM frames were
running. That inference was incorrect: when no committed PWM telemetry was
available, the micro-ROS publisher stamped its fallback state with the current
time. The timestamp could advance while the underlying PWM state remained
stale.

## Boundary diagnosis

A temporary diagnostic firmware recorded the complete command path through
existing ROS diagnostics. It did not print on USART1, which remained owned by
the micro-ROS transport. It also ran the same PWM3 sequence autonomously after
session creation, without receiving a ROS PWM command.

The autonomous test recorded:

- ROS `command_messages`: 0;
- PWM3 controller accepts: 3;
- pending-frame prepares: 3;
- platform shadow submissions: 3;
- committed PWM frames: 0;
- TIM13 frame sequence: 0; and
- platform active and shadow PWM3 values: 1500.

The ROS-driven test recorded one gateway call, mailbox publication, mailbox
consumption, ownership acceptance, controller acceptance, pending-frame
prepare, and shadow submission. It again recorded zero frame commits and a
TIM13 frame sequence of zero.

These two controlled paths established that the failure was after the
controller shadow submission and before the first timer frame boundary. They
also ruled out the following as causes of this incident:

- Fast DDS discovery-server availability;
- duplicate ROS node names;
- `ros2_control` reference generation;
- ROS serialization or QoS delivery;
- PWM mailbox loss, overwrite, or session-ownership rejection;
- controller validation or interpolation setup; and
- a stale telemetry cache hiding otherwise advancing frame commits.

All temporary hardcoded commands, diagnostic fields, counter overrides, build
options, and platform inspection hooks were removed after this measurement.

## Root cause

The production startup sequence called `ScheduleTimer13(1U)`. The scheduler
used:

```text
ARR = bounded_delay_us - 1
CNT = 0
```

For a one-microsecond delay this produced `ARR=0`. TIM13 therefore remained
blocked, its update ISR never began the first servo frame, and every submitted
shadow remained uncommitted.

The legacy firmware under
`docs/reference/RosRobotControllerLite_ros_250811/` was used only as a wiring
and timer reference. It confirmed PC8/PWM3, a 1 MHz TIM13 base, and a nonzero
initial period. No legacy implementation was copied. Commit `e30c7ae` used the
same failing v2 TIM13 startup code and did not contain an alternative PWM fix.

Primary hardware reference:
[ST RM0090, STM32F405/407/415/417 reference manual](https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf).

## Permanent correction

`ScheduleTimer13()` now treats a one-tick interval specially:

```text
delay >= 2 us: ARR = delay - 1, CNT = 0
delay == 1 us: ARR = 1,         CNT = 1
```

Starting an up-counter at its nonzero auto-reload value makes the next 1 MHz
tick generate the update event. This preserves the requested one-microsecond
delay and also prevents a future stall when two servo falling edges are
separated by exactly one microsecond.

The 500 ms command value is interpolation duration, not a 2 Hz PWM frequency.
At the unchanged 50 Hz PWM frame rate it represents 25 frames of 20 ms each.
PWM state is published at 20 Hz. The controller's 100 ms reference timeout is
a separate host-control setting and was not changed by this repair.

## Focused verification

Only the directly relevant tests and production artifacts were built:

- `mentor_pi_mcu_controller_tests`: passed;
- `mentor_pi_mcu_stm32_peripheral_safety_contract`: passed;
- the safety-contract regression fails on the old scheduler and requires the
  nonzero one-tick `ARR`/`CNT` representation; and
- the existing controller regression covers mask `0x04`, channel index 2,
  500 ms/25-frame interpolation, repeated commands, frame commits, and session
  generation transitions.

Both normal ADRC artifacts passed provenance and memory verification:

| Robot | Namespace | ELF SHA-256 | Flash usage |
| --- | --- | --- | --- |
| Ackermann | `/ackermann_1` | `ffcf31889466a596aa7ea577ad382b54f94a5382aecdab45f571367a09989b6f` | 157,292 / 524,288 bytes |
| Mecanum | `/mecanum_2` | `78ddb3904b08bd1bc3e0c3c3dc1aed01b5c6480db0090ea0dd687e1fc05d5291` | 157,284 / 524,288 bytes |

The first Mecanum programming attempt stopped on a CubeProgrammer read error.
A subsequent read-only 16-byte access at `0x08000000` succeeded, proving that
readout protection was not blocking access. One controlled retry then reached
100%, reported `Download verified successfully`, and reset into the verified
application.

## HIL results

Actuator and motor power were disconnected throughout. Subscribers were fully
discovered before publishing `1500 -> 1600 -> 1400 -> 1500`, with 500 ms per
step.

Both robots showed:

- target acceptance in the first state sample after each command;
- progressive PWM3 output values through each transition;
- `moving_mask=0x04` during interpolation and zero at completion;
- channels 1, 2, and 4 remaining at 1500 microseconds;
- zero new command rejections;
- zero PWM mailbox overwrites;
- zero PWM peripheral errors;
- an active MCU session for the complete sequence; and
- final PWM3 target/output 1500 with moving mask zero.

The complete local captures were intentionally not committed:

| Robot | Capture | State / diagnostic samples | SHA-256 |
| --- | --- | --- | --- |
| Ackermann | `/tmp/mentor-pi-pwm-final-hil-ackermann_1-20260815.json` | 67 / 3 | `1cec7bc5bd3a2023699c4477e4490805c4d9bc72568123a5e8e31136091b6e75` |
| Mecanum | `/tmp/mentor-pi-pwm-final-hil-mecanum_2-20260815.json` | 69 / 4 | `832af46dca72aafbd28f603505d1096b7cfe433ea97e0bf09df301759cefae8e` |

After deployment, both robot repositories were clean at `616dbca`, both named
pre-deployment stashes remained intact, and both production Agent services
were active.

## Explicitly separate work

This incident did not change or resolve:

- the independent host feedback-timeout discussion (100 ms versus 500 ms);
- OLED initialization errors on absent or unverified hardware;
- the earlier micro-ROS watchdog reset investigation;
- discovery-server infrastructure availability; or
- powered steering, physical PWM waveform, endurance, or release
  qualification.
