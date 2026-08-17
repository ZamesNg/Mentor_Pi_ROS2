# RRCLite v2 Hardware Baseline

Status: normative hardware input to the v2 design  
Board revision: Ros Robot Controller Lite V1.0  
MCU: STM32F407VET6

## Purpose and evidence

This document records the hardware that the rewrite may use. It does not infer
features from filenames alone. Each claim below is supported by one or more of
these local references:

- the two-page [board schematic](<../reference/SCH_Ros Robot Controller Lite V1.0.pdf>);
- the two-page [hardware introduction](<../reference/第1节 RRCLite硬件介绍_V1.1.pdf>)
  for connector-edge orientation and populated-device location;
- the generated [STM32Cube configuration](../reference/RosRobotControllerLite_ros_250811/RosRobotControllerM4.ioc);
- generated HAL initialization under
  [`Core/Src`](../reference/RosRobotControllerLite_ros_250811/Core/Src/);
- board support and application code under
  [`Hiwonder`](../reference/RosRobotControllerLite_ros_250811/Hiwonder/);
- the firmware [porting log](../reference/RosRobotControllerLite_ros_250811/Doc/log.txt).

The schematic is authoritative for connectivity. The Cube configuration and
generated HAL code are authoritative for the peripheral mapping that the legacy
firmware actually selected. Application code is evidence that a populated
resource was exercised, not authority to change the wiring.

See also the [requirements](requirements.md), accepted
[transport ADR](adr/0001-mcu-ros-transport.md), [architecture](architecture.md),
and [legacy audit](legacy-audit.md).

## Locked host-to-MCU path

The communication path shall be:

```text
host USB port
  -> board USB-C connector labelled UART1
  -> CH9102F USB-to-UART bridge
  -> PA10 / USART1_RX and PA9 / USART1_TX
  -> STM32 USART1
```

The schematic connects the `UART1` USB-C D+ and D- nets to CH9102F `UD+` and
`UD-`. CH9102F `TXD` crosses to MCU PA10, and `RXD` crosses to PA9. The Cube
configuration assigns PA10 to `USART1_RX` and PA9 to `USART1_TX`; generated
`usart.c` configures 1,000,000 baud, eight data bits, one stop bit, no parity,
and no hardware flow control. The v2 transport is therefore **USB on the host
side and UART on the MCU side**.

The board has another USB-C connector in the schematic block labelled
`5V-PD-OUT`. Its CC and power circuitry does not route D+ or D- to the MCU or
CH9102F. It is a power output and shall never be presented, detected, or used
as a communication port.

The STM32F407 native full-speed USB pins do not provide an unused alternate
path on this revision. PA11 and PA12 are routed to PWM-servo outputs 1 and 2.
PB14 and PB15 are not routed as a usable host connector and the generated
firmware configures them as analog inputs. Reclaiming any of these pins or
adding a native-USB connector would require PCB modification and loss or
redesign of retained hardware; both are outside the v2 baseline.

The following alternatives are outside the supported board baseline:

- STM32 native USB CDC or USB OTG transport;
- treating the CH9102F link as a native USB peripheral in MCU software;
- communication through the `5V-PD-OUT` connector;
- PCB rework or an external adapter that bypasses CH9102F;
- changing the fixed USART1 line format or enabling flow-control pins.

## Clock baseline

The populated high-speed external crystal is Y1, marked
`16MHz_10PF_20PPM` in the board schematic and described as 16 MHz in the
hardware guide. The v2 firmware therefore uses HSE at 16 MHz and the same
validated clock tree as the legacy Cube configuration: PLLM 8, PLLN 168,
PLLP 2, PLLQ 7. This produces a 336 MHz PLL VCO, 168 MHz SYSCLK/HCLK, 42 MHz
APB1 (84 MHz APB1 timer clock), 84 MHz APB2 (168 MHz APB2 timer clock), and a
48 MHz PLLQ domain. `HSE_VALUE` shall remain 16,000,000 so HAL clock-derived
baud rates and time bases agree with the physical oscillator.

Clock Security System shall be enabled after switching to the PLL. A clock
configuration failure is a safe-boot failure: motor outputs remain zero and
the scheduler is not started. The physical bring-up checklist shall still
measure USART1 baud rate and the timer time bases, because the documents prove
the component value but not the condition of an individual board.

USART1 RX is assigned to DMA2 Stream 2, Channel 4 and USART1 TX to DMA2 Stream
7, Channel 4. The legacy Cube file selects normal mode for both. V2 deliberately
changes **RX only** to continuous circular DMA as specified in
[architecture.md](architecture.md); TX remains a bounded normal DMA transfer.
TIM7 uses priority 5. USART1 and both of its DMA streams use priority 6, at or
below the configured FreeRTOS syscall ceiling, and may use standard HAL handlers
and the approved FreeRTOS `FromISR` APIs.

## Confirmed active hardware

### Pin and peripheral ownership

The following mapping is locked for RRCLite V1.0. A logical motor row names
the two drive-PWM outputs first and its quadrature encoder second.

| Function | MCU ownership |
| --- | --- |
| Motor M1 | Positive drive TIM1 CH4/PE14, negative drive TIM1 CH3/PE13; encoder TIM5 CH1/PA0 and CH2/PA1. |
| Motor M2 | Positive drive TIM1 CH2/PE11, negative drive TIM1 CH1/PE9; encoder TIM2 CH1/PA15 and CH2/PB3. |
| Motor M3 | Positive drive TIM9 CH1/PE5, negative drive TIM9 CH2/PE6; encoder TIM4 CH1/PD12 and CH2/PD13. |
| Motor M4 | Positive drive TIM11 CH1/PB9, negative drive TIM10 CH1/PB8; encoder TIM3 CH1/PB4 and CH2/PB5. |
| Motor control release | TIM7 internal base timer, with no GPIO or DMA ownership. V2 configures a 1 kHz release; every tenth release runs the 100 Hz encoder/ADRC update. |
| PWM servos 1–4 | GPIO pulse outputs PA11, PA12, PC8, and PC9; TIM13 is the frame scheduler. |
| Bus servos | PC12, UART5 TX in single-wire half-duplex mode, 115,200 baud 8N1. |
| LEDs 1–3 | PD9, PD10, PD11. LEDs 1 and 2 are active-low; LED 3 is active-high. |
| Buzzer | PA6 GPIO waveform output; TIM12 is the pattern/frequency scheduler. PA8 `BUZZER_OLD` is not an active v2 output. |
| RGB pixels 1–2 | SPI1 MOSI/PA7 with DMA2 Stream 3, Channel 3; SPI1 SCK is PA5. RGB1 is firmware status and RGB2 is host-controlled. |
| Buttons 1–2 | PE1 (`KEY1`) and PE0 (`KEY2`), both active-low. |
| QMI8658 IMU | Software-I2C SCL/PB10 and SDA/PB11; rising-edge interrupt PB12. |
| Battery monitor | PB0/ADC1 channel 8 plus ADC1 VREFINT; ADC DMA2 Stream 0, Channel 0. |
| OLED | I2C1 SCL/PB6 and SDA/PB7. |
| Host transport | USART1 TX/PA9 and RX/PA10; RX DMA2 Stream 2 and TX DMA2 Stream 7, both Channel 4. |

The measured chassis placement, host chassis-direction map, IMU transform, and
RGB1 semantics are compiled according to the
[verified board profile](verified-hardware-profile.md).

The data USB-C connector is also the supported programming path when no debug
probe is available. RRCLite V1.0 connects CH9102F TX/RX to the STM32F407 ROM
bootloader's USART1 pins, pulls BOOT0 and BOOT1 low by default, and provides a
BOOT button that raises BOOT0 plus an active-low RST button. Holding BOOT,
tapping RST, and then releasing BOOT enters system memory for CubeProgrammer
UART download at 115200 baud, 8E1, without flow control. After programming, a
normal RST samples BOOT0 low and runs application flash. This programming mode
does not change the 1,000,000-baud 8N1 runtime transport contract and is not a
native USB/DFU implementation. The schematic also contains a CH9102F
handshake-driven download circuit. Project flashing uses separate modem ioctls:
RTS asserted with DTR deasserted asserts reset with BOOT0 high; asserting DTR
while RTS remains asserted releases reset into system memory. After verified
programming, the normal-boot sequence again asserts reset, then deasserts RTS
while DTR remains deasserted so BOOT0 is low when reset releases. The physical
BOOT/RST sequence remains a fallback only when automatic activation fails
before programming. Automatic ROM entry is source- and mock-verified but
remains a physical verification item until `make flash` succeeds
without button input on the board. Runtime uses a tracked patch to the pinned
Micro-XRCE-DDS-Agent for the same normal-boot reset on its own descriptor.
DTR/RTS are not runtime data flow, and no second process may hold the serial
device open as a modem-line guard.

The M1–M4 order above is the public array order. Firmware uses raw signed
quadrature delta directly for every motor model. `target_rps`, `measured_rps`,
accumulated count, LADRC state, and semantic controller output share that MCU
coordinate. One fixed inversion converts semantic output to physical bridge
duty for all channels and models. Connector mechanics do not define ROS
forward, so the ROS hardware layer owns the only chassis sign map:
`{-1,+1,-1,+1}` in logical FL,FR,RL,RR order. The same map is applied to
commands and feedback and is its own inverse.

An actuator-power-disconnected `ackermann_0` capture on 2026-08-17 observed
M2 decreasing and M4 increasing when the operator rotated the rear-left and
rear-right wheels forward. With direct raw MCU feedback and host rear-wheel
signs `{-1,+1}`, both become positive ROS wheel rotation. An older capture on
a different setup recorded the opposite raw signs, so no raw electrical sign
is transferable between vehicles without the required passive check.
The default ADRC firmware accepts bounded motor targets, but its physical
polarity and controller performance remain unqualified. Checkout shall first
rotate each raised wheel manually with bridge outputs disabled and record both
raw MCU `motors/state` direction and converted ROS joint direction. Powered tests
may begin only afterward with raised-wheel or equivalent guarding, a
current-limited supply, deliberately bounded commands, and continuous stop
monitoring.

### IMU frame

`imu_link` is fixed to the PCB rather than to an assumed robot mounting. View
the component side with the two USB-C connectors on the far edge, as in the
hardware introduction: +X points from the board center toward that USB-C edge,
+Y points toward the edge carrying the four PWM-servo connectors, and +Z points
out of the component side. These axes form a right-handed frame. A host robot
description shall provide the static transform from `imu_link` to its chassis
frame.

The supplied schematic and hardware guide identify the QMI8658 location and
pins but do not state the package-axis-to-PCB signed permutation. Six-face
gravity measurements on 2026-08-07 established the compile-time transform as
board X = sensor Y, board Y = -sensor X, and board Z = sensor Z. With the
provisional identity transform, the measured dominant axes were +Y, -Y, -X,
+X, +Z, and -Z for the requested PCB +X, -X, +Y, -Y, +Z, and -Z faces,
respectively. The complete readings are preserved in
`build/diagnostics/characterization-20260807T114316Z/imu-six-face.tsv`.
Positive rotations about all three board axes remain a release-qualification
check. This is a measured PCB-frame correction, not an open ROS frame decision.

| Function | Quantity and confirmed mapping | V2 disposition |
| --- | --- | --- |
| Encoder motors | Four motor-driver stages in the schematic. TIM1 CH1-4, TIM9 CH1-2, TIM10 CH1, and TIM11 CH1 generate drive PWM; TIM2, TIM3, TIM4, and TIM5 are encoder interfaces in the Cube file. | Retain all four. The default ADRC image exposes encoder state and bounded closed-loop control; passive checkout and guarded HIL precede any powered-motion or performance claim. |
| PWM servos | Four connectors. Firmware/Cube GPIOs are PA11, PA12, PC8, and PC9; TIM13 supplies the legacy frame timing. | Retain four channels. Generate pulses from a short timer ISR using task-prepared shadow values. |
| Bus servos | Half-duplex bus-servo circuit is driven by the `UART6_TX`/`SERVO_SIGNAL` schematic net. Current firmware maps the signal to UART5 on PC12 and initializes UART5 at 115,200 8N1, no flow control. | Retain on UART5 with one owning worker. Support at most 16 servo IDs in one ROS motion request. |
| Indicator LEDs | Three GPIO LEDs are present in the schematic and `LED_NUM` is three. | Retain all three; reserve LED1 for the firmware system heartbeat and expose LED2/LED3 to ROS. |
| Buzzer | One transistor-driven buzzer is present; legacy timing uses TIM12. | Retain one, with bounded frequency and pattern state. |
| RGB LEDs | Two cascaded RGB devices are present and `Pixel_S1_NUM` is two; legacy output uses SPI1 TX DMA. | Retain exactly two pixels: RGB1 for firmware status and RGB2 for ROS commands. |
| Buttons | Two button inputs are present and the firmware creates two button objects. | Retain both with debounced events. |
| IMU | The schematic identifies QMI8658 and its interrupt. The active driver uses the board software-I2C port and enables both sensors at 250 Hz with the accelerometer at ±4 g, gyroscope at ±128 degrees/s, and both internal low-pass filters in mode 00 (2.66% of ODR, approximately 6.65 Hz). Six-face hardware characterization measured board X = sensor Y, board Y = -sensor X, and board Z = sensor Z. | Retain that measurement configuration, require CTRL5 readback of `0x11`, and publish the newest bounded sample at 50 Hz using the measured signed permutation. Positive-axis rotation and extended timing HIL remain release gates. |
| Battery monitor | ADC1 samples VREFINT then PB0/channel 8 every 50 ms; the schematic contains the battery divider and regulator feedback path. The legacy estimate uses a 0.05 IIR update weight. | Retain calibrated voltage reporting, that filter response, and a configurable low threshold. |
| OLED | Active firmware enables OLED and configures an SSD1306-compatible 128 x 32 display over hardware I2C1. Cube maps I2C1 to PB6/PB7. | Retain two bounded host-controlled text lines; the controller-owned battery indication remains local. |
| Independent watchdog | IWDG is enabled in the Cube file and generated firmware. | Retain, but replace scattered refreshes with the supervised policy in [reliability-and-safety.md](reliability-and-safety.md). |

The motor and peripheral timer/channel assignment is a wiring constraint. Driver
internals may be rewritten, but an implementation shall not swap logical IDs to
different connectors without an explicit schematic/bench correction recorded in
this document and the [ROS interface contract](ros-interface-contract.md).

## Explicitly inactive legacy features

The porting log records that USB host, Bluetooth, and SBUS were removed for the
Lite board. `ENABLE_LVGL` is zero, while the smaller OLED remains enabled. Some
source files and old packet identifiers still exist for these removed features;
they shall not be compiled into, advertised by, or consume static resources in
v2.

Excluded features are:

- USB host and USB gamepad;
- Bluetooth;
- SBUS;
- LCD and LVGL;
- chassis-level kinematics;
- the legacy proprietary host packet protocol.

## MCU memory classes

The local Keil project describes 512 KiB of flash at `0x08000000` and 128 KiB
of DMA-accessible SRAM at `0x20000000`. The legacy memory port also identifies
64 KiB of core-coupled memory at `0x10000000`.

| Class | Access rule | Required use |
| --- | --- | --- |
| Flash, 512 KiB | Execute/read | Firmware, constants, micro-ROS type support. |
| SRAM, 128 KiB | CPU and DMA | USART1 RX ring and TX bounce buffer, SPI/ADC DMA buffers, RTOS stacks, queues, and shared snapshots. |
| CCM, 64 KiB | CPU only; not DMA-accessible | Resettable 48 KiB micro-ROS allocation arena, the 2 KiB CPU-only motor-task stack, and explicitly reviewed CPU-only state. No DMA buffer or DMA source may be placed here. |

The linker script shall expose separate sections for DMA buffers and CCM. The
build shall fail if a DMA object resolves into CCM or if the budgets in
[architecture.md](architecture.md) are exceeded.

## Bring-up checks before driver work

Before actuator code is enabled, bring-up shall verify:

1. the attached Linux device reports the CH9102F identity expected by the
   deployment rule;
2. the data cable is connected to `UART1`, not `5V-PD-OUT`;
3. PA9/PA10 carry a 1,000,000 baud 8N1 exchange with no RTS/CTS activity;
4. logical motor, PWM-servo, LED, button, and RGB IDs match the physical
   connectors documented above;
5. with the default ADRC image and all motor PWM outputs still disabled,
   manually rotate each raised motor output in the declared positive direction
   and record raw MCU and ROS encoder count direction;
6. UART5 remains electrically half-duplex and does not contend with the IMU or
   any excluded SBUS implementation;
7. all motor PWM outputs are zero before and throughout the checks.

Record any discrepancy as a hardware-baseline issue. Do not compensate for an
unresolved wiring discrepancy in ROS callbacks. A failed or ambiguous passive
encoder check blocks powered motion and ADRC/polarity release
qualification.
