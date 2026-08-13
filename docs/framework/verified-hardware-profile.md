# Verified RRCLite V1 Board Profile

This document is the maintained board-specific profile compiled into the MCU
firmware. It separates measurements made on the current RRCLite V1 board from
reference defaults that still require instrumented qualification. Public motor
arrays remain in connector order `M1, M2, M3, M4`; the firmware does not reorder
them into chassis order.

## Motor and encoder placement

Passive one-wheel-at-a-time captures with motor power disconnected established
this physical ownership when the robot is viewed from above with its front in
the normal driving direction:

| Connector | Chassis position | Measured forward delta before correction | Compiled channel wiring sign |
| --- | --- | ---: | ---: |
| M1 | front-left | +742 | +1 |
| M2 | rear-left | +1915 | +1 |
| M3 | front-right | -1370 | -1 |
| M4 | rear-right | -1612 | -1 |

The compiled channel signs are `{+1, +1, -1, -1}` in M1--M4 order. They are
hardware wiring corrections applied in addition to the existing per-model
encoder polarity. With the retained JGA27 model method, manually rotating any
wheel in the robot-forward direction therefore produces positive normalized
encoder velocity. These captures prove channel ownership and sign only; their
different hand-rotation distances are not ticks-per-revolution measurements.

The first very short guarded M1 test requested `+0.1 RPS` for 100 ms and ended
with only `-1` net tick, causing the fail-closed utility to report
`MOTOR_WRONG_DIRECTION`. A subsequent observable run using `+0.25 RPS` for
2,000 ms produced `+294` ticks, 0.452010 RPS peak magnitude, confirmed the
final zero state, and passed every commissioning check. The longer result
proves the current M1 sign; the one-tick result is retained as a short-window
noise/settling observation rather than used to change firmware polarity.

JGA27 remains the firmware reset default and the host supervisor still applies
the configured JGA27 model at each ROS session. The existing JGA27 reference
values for ticks per revolution, maximum speed, filter, and ADRC gains remain
provisional until a later powered, current-limited control stage measures them.
The default image now uses the shared closed-loop ADRC implementation. That
software state does not qualify the provisional gains, polarity, or regulated
speed. Encoder sign and the operator-observed physical direction must agree
before another channel is attempted, and powered characterization still needs
the guarded, current-limited HIL record.

## IMU orientation

Six stationary gravity faces established the following signed permutation for
both acceleration and angular velocity:

```text
PCB X = sensor Y
PCB Y = -sensor X
PCB Z = sensor Z
```

The firmware compiles this as `{{1,+1}, {0,-1}, {2,+1}}`. Representative
acceleration measurements in m/s² were:

| Requested PCB face | Sensor X | Sensor Y | Sensor Z |
| --- | ---: | ---: | ---: |
| +X | 0.456 | 9.606 | -0.725 |
| -X | 0.208 | -10.005 | 0.855 |
| +Y | -9.319 | -0.104 | -0.323 |
| -Y | 10.196 | -0.280 | 0.102 |
| +Z | 0.248 | 0.159 | 9.693 |
| -Z | 0.369 | -0.861 | -9.785 |

The capture proves the signed axes. Bias, scale, temperature response, angular
positive direction, and extended timing remain qualification work.

## Retained board defaults

| Function | Compiled setting | Status |
| --- | --- | --- |
| ROS transport | USART1, 1,000,000 baud, 8N1 | Polling and circular-DMA challenge passed; live ROS graph and telemetry observed. |
| Battery ADC | PB0 / ADC1 channel 8, 11:1 divider, filter weight 0.05 | Retained RRCLite reference setting; not multimeter-calibrated. |
| Battery low threshold | 6300 mV | Retained usable default; runtime service remains available. |
| Battery absent threshold | at or below 4900 mV | Compiled behavior; suppresses a false low-battery alarm without a battery. |
| OLED | Normal optional hardware path | The current fixture has no OLED; absence is not compiled as a board property. |

Generated evidence is intentionally not committed. The source capture used for
the motor and IMU conclusions is under
`build/diagnostics/characterization-20260807T114316Z/` on the development host.
Hardware claims beyond the rows above still require the verification document's
instrumented HIL gates.

## Firmware-owned status indicators

Discrete LED1 is reserved for firmware and blinks at 1 Hz as a system
heartbeat, independent of ROS connectivity. ROS LED commands accept only IDs
2 and 3; ID 1 is rejected without changing the indicator.

The first onboard RGB pixel (RGB1) is reserved for MCU status. The existing
two-element ROS message is retained for wire compatibility, but only mask `2`
(RGB2) is accepted. Masks `1` and `3` are rejected atomically; a rejected
command changes neither pixel.

RGB1 uses a low component value of 32 and is updated by `PeripheralTask` through
the existing bounded SPI1 DMA driver:

| Component | Meaning |
| --- | --- |
| Red | Toggles after each successfully published `/mentor_pi/heartbeat` sample. |
| Green | Pulses for 50 ms when the 10 Hz transport sample observes that the cumulative UART TX byte counter advanced. |
| Blue | Pulses for 50 ms when the 10 Hz transport sample observes that the cumulative UART RX byte counter advanced. |

RX and TX indications can combine with the current red heartbeat state. A closed
transport immediately clears green and blue; red retains its last heartbeat
state. The sampler performs no allocation,
does no work in the UART ISR, and starts an RGB DMA transfer only when the
desired visible color changes.
