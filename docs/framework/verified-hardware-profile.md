# Verified RRCLite V1 Board Profile

This document is the maintained board-specific profile compiled into the MCU
firmware. It separates measurements made on the current RRCLite V1 board from
reference defaults that still require instrumented qualification. Public motor
arrays remain in connector order `M1, M2, M3, M4`; the firmware does not reorder
them into chassis order.

## Motor placement and sign coordinates

Connector ownership is fixed when the robot is viewed from above with its
front in the normal driving direction:

| Connector | Chassis position | Ackermann use |
| --- | --- | --- |
| M1 | front-left | unused traction channel |
| M2 | rear-left | left drive motor |
| M3 | front-right | unused traction channel |
| M4 | rear-right | right drive motor |

Mecanum uses all four connectors. Ackermann uses M2/M4 for rear traction and
PWM3 for steering. Connector ownership does not change with motor model.

The implementation has one chassis-direction map:

```text
MCU encoder state = raw signed counter delta

MCU target = ROS wheel target * ROS_MCU_SIGN
ROS wheel state = MCU state * ROS_MCU_SIGN

bridge duty = LADRC output
```

There is no model sign, wiring sign, or output sign in firmware. `target_rps`,
raw `measured_rps`, accumulated raw `encoder_count`, LADRC state, and bridge
duty all use the same MCU coordinate.

The compiled values are:

| Boundary | Order | Value |
| --- | --- | --- |
| MCU encoder transform | M1, M2, M3, M4 | none; raw delta is retained |
| ROS↔MCU chassis sign | FL, FR, RL, RR | `{-1,+1,-1,+1}` |
| MCU output transform | M1, M2, M3, M4 | none; bridge duty is LADRC output |

The ROS map is applied to commands and feedback and is its own inverse.
Positive ROS wheel rotation rolls the chassis toward +X. Consequently, a
positive forward ROS command produces negative MCU targets on M1/M2 and
positive MCU targets on M3/M4; the bridge receives the LADRC result directly.

On 2026-08-17, with actuator and motor power disconnected, an
`ackermann_0` passive capture using raw encoder state observed
M2 at `-1054` and M4 at `+3415` after the operator rotated rear-left and
rear-right forward. No MCU reset or Agent reconnect occurred. The host rear
signs `{-1,+1}` yield positive ROS motion for both wheels.

An older 2026-08-07 development-fixture capture recorded forward deltas M1
`+742`, M2 `+1915`, M3 `-1370`, and M4 `-1612`. That conflicts with the newer
Ackermann capture, so raw electrical direction is not treated as transferable
fleet evidence. The older guarded M1 result (`+0.25 RPS` for 2,000 ms, `+294`
ticks, 0.452010 RPS peak) is retained as historical evidence for that fixture,
not qualification of the current six vehicles. Every vehicle still requires
the passive sign check before powered work, and Mecanum front-channel signs
remain unverified in this campaign.

JGA27 remains the firmware reset default and the host supervisor still applies
the configured JGA27 model at each ROS session. The existing JGA27 reference
values for ticks per revolution, maximum speed, filter, and ADRC gains remain
provisional until a later powered, current-limited control stage measures them.
The default image now uses the shared closed-loop ADRC implementation. That
software state does not qualify the provisional gains, physical polarity, or
regulated speed. Raw MCU encoder direction, ROS wheel direction, and the
operator-observed physical direction must agree before powered work, which
still needs a guarded, current-limited HIL record.

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
