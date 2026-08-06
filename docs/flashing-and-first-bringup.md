# Firmware Flashing and First Bring-Up

This procedure is for RRCLite V1.0 with an STM32F407VET6. A debug probe is not
required to flash the board: the connector labelled USB serial 1/download
terminates at a CH9102F whose TX/RX lines reach the STM32F407 USART1 bootloader
pins, and the PCB provides both BOOT and RST buttons. The same connector is the
1 Mbaud micro-ROS runtime link after reset; it is not a native USB DFU
connector. Use the separate 5 V/5 A USB-C connector only as its documented
power output, never for flashing or communication.

Before the board session, make a working copy of the
[board-arrival bring-up evidence record](board-arrival-bringup-checklist.md).
It follows this procedure through the initial one-motor stop/reconnect checks
and keeps the later stress and soak qualification separate.

## Required equipment

- STM32CubeProgrammer, including its `STM32_Programmer_CLI` executable;
- a current-limited board supply;
- a USB data cable for the CH9102F runtime link;
- for every first-board motor test, a fixture that keeps all wheels raised (or
  provides equivalent mechanical containment), plus a readily accessible
  motor-power stop; and
- for initial passive checks, motor power disconnected.

An ST-Link V2/V3 or J-Link and the board's `SWDIO`, `SWCLK`, `GND`, target-
voltage sense, and preferably `NRST` connection remain required for source-
level debugging and the raw timer/IMU characterization steps in this
procedure. They are not required for UART flashing, locked-image ROS tests, or
ordinary runtime use. Without a probe, stop before any gate that explicitly
requires raw SWD evidence; do not replace it with an assumption.

PA13 is SWDIO and PA14 is SWCLK in the verified legacy Cube configuration.
Confirm the physical connector labels and orientation on the actual board
before attaching a probe. Connect grounds first. Let the probe sense the
target's 3.3 V level; do not let both the probe and board drive the 3.3 V rail
unless the board documentation explicitly permits it.

## Build artifacts

From the repository root:

```sh
make doctor
make setup
make firmware
```

This command deliberately builds the normal **motor-locked** image. It accepts
selected zero-speed commands as stop commands, but atomically rejects any motor
command containing a selected nonzero target with result `UNSUPPORTED`. Encoder
telemetry and the other retained peripherals remain available. Always flash
and verify this image before considering a commissioning image.
The build fails unless the selected profile is `mode=LOCKED`; it describes the
artifact being produced and does not inspect a previously built ELF.

The firmware build produces these files under
`firmware/mentor_pi_mcu/build/stm32/`:

- `mentor_pi_mcu.elf` for debug and symbolized fault analysis;
- `mentor_pi_mcu.hex` for most programmers;
- `mentor_pi_mcu.bin` for a raw write at address `0x08000000`;
- `mentor_pi_mcu.map` for the memory audit; and
- `rrclite-build-metadata.txt` binding the motor profile, project inputs,
  pinned micro-ROS tree, and artifact SHA-256 values.

Do not flash an image if the build's 20% flash/SRAM/CCM headroom assertions
fail.

To create an immutable operator handoff after the checks pass, use:

```sh
./tools/package_board_handoff.sh
```

The default package contains only the locked artifacts under a UTC-stamped
`build/board-handoff/` directory. Its nested manifests cross-check every
ELF/HEX/BIN/map digest against `BUILD-METADATA.txt` as well as hashing the
packaged copies. A commissioning package is intentionally unavailable unless
commissioning is enabled with the exact build acknowledgement shown below
after the physical safety gate.

The handoff directory is an immutable evidence copy. The flash wrapper uses the
source-bound authoritative ELF under `firmware/mentor_pi_mcu/build/stm32/` and
creates its own hash-named temporary snapshot after verification. Before a
board session that names a specific handoff, compare its ELF and
`BUILD-METADATA.txt` byte-for-byte with the authoritative files as shown in the
board-arrival checklist. A successful handoff manifest check alone does not
select that package as the flash input.

### Guarded motor-commissioning image

Only after the normal image and passive encoder-direction checks below pass,
build the non-release commissioning image with its exact acknowledgement:

```sh
make firmware-commissioning COMMISSIONING_BUILD_ACK=MOTORS_RAISED
```

The wrapper and CMake configuration fail closed if commissioning is requested
without the exact acknowledgement. This image caps accepted targets at
0.25 RPS and motor output at 300 permille. Those limits are compile-time safety
ceilings, not tuning parameters and not evidence that the provisional PID gains
or direction factors are release-qualified. There is no ROS service, parameter,
or host configuration that unlocks the normal image.
The build must report `mode=COMMISSIONING`, the 0.25/300 limits, and
`release_qualified=0`; it fails if the acknowledgement is absent or misspelled.

The build paths are reused, so record the ELF/HEX SHA-256 digest and label the
commissioning artifacts before another build overwrites them. Rebuild and
reflash the normal locked image after commissioning unless a separately
reviewed, HIL-qualified production image has been approved.

## Flash over USB serial 1

The project-owned flash target verifies the source, profile, dependency and
artifact hashes, creates an immutable ELF snapshot, and then invokes
STM32CubeProgrammer. The ROM bootloader uses `115200` baud, 8 data bits, even
parity, one stop bit, and no flow control. This is deliberately different from
the application runtime, which uses 1,000,000 baud, 8N1.

1. Disconnect all motors, PWM servos, and the bus-servo power/device connector.
   Close the micro-ROS Agent and every other process that may own the serial
   device.
2. Connect the USB-C port labelled USB serial 1/download. Do not connect the
   5 V/5 A output port to the host as a data connection.
3. Identify the exact CH9102F device. On Ubuntu use its stable
   `/dev/serial/by-id/*` entry during first flash; do not guess a changing
   `/dev/ttyUSB*` number.
4. Hold `BOOT`, press and release `RST`, then release `BOOT`. BOOT1 is already
   pulled low on RRCLite V1.0, so this reset enters the STM32 factory system-
   memory bootloader.
5. Run the verified locked upload, replacing the example port:

   ```sh
   make flash \
     PORT=/dev/serial/by-id/REPLACE_ME \
     FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
   ```

   Replace the port with the exact stable path. If CubeProgrammer is installed
   outside the executable search path, set `STM32_CUBE_PROGRAMMER_CLI` to its
   absolute executable path. The wrapper rejects a missing or wrong
   acknowledgement, an empty or ambiguous port, the wrong motor profile,
   stale build metadata, a changed dependency/source fingerprint, a changed
   snapshot digest, or a failed CubeProgrammer read-back comparison.
6. After CubeProgrammer reports programming and verification success, release
   `BOOT` and press `RST` normally. Close CubeProgrammer before starting the
   Agent at 1,000,000 baud.

The default flash target accepts only the normal `LOCKED` artifact. After every
passive and physical commissioning gate has passed, build with its exact
commissioning acknowledgement, then flash with the separate bootloader and
physical-safety acknowledgements:

```sh
make firmware-commissioning COMMISSIONING_BUILD_ACK=MOTORS_RAISED
make flash-commissioning \
  PORT=/dev/serial/by-id/REPLACE_ME \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  COMMISSIONING_FLASH_ACK=MOTORS_RAISED_CURRENT_LIMITED
```

This UART path programs and verifies firmware but provides no breakpoint,
register, or raw snapshot access. Use SWD when a later step requires those
capabilities.

## Optional SWD source debugging

The CH9102F/USART1 bootloader is a flash transport only. Breakpoints, register
inspection, raw snapshots, and source-level debugging require an ST-Link or
J-Link connected to SWDIO, SWCLK, GND, target-voltage sense, and preferably
NRST. The project has no second firmware build graph for a debugger: build with
`make firmware`, flash the verified artifact through the supported UART target,
then attach the probe client to
`firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf` for symbols.

Debugger sessions are attach-only. Do not use an IDE firmware-download action
or a GDB `load` command, because either would bypass the artifact and motor-
profile verification performed by the flash wrapper. Use connect-under-reset
if an earlier image prevents a normal connection, and keep `NRST` connected for
that recovery path. There is no supported source-debugging workflow without an
SWD probe.

### SWD halt and the independent watchdog

The independent watchdog timeout is approximately 0.5 seconds and production
firmware intentionally does not freeze it when the core is halted. A normal
breakpoint therefore causes a watchdog reset, which is the correct production
behavior.

Only for passive inspection with motor power physically disconnected, the
debugger may temporarily freeze IWDG by setting `DBGMCU_APB1_FZ.DBG_IWDG_STOP`:

```text
0xE0042008 |= 0x00001000
```

Clear bit 12 again before any powered actuator test, reset the MCU, and verify
normal watchdog behavior. Never add this freeze to production firmware and
never use it to make a powered debug session appear safe; halting the CPU also
halts the software paths that enforce command leases and safe teardown.

### Raw IMU axis characterization over SWD

The default firmware deliberately leaves the QMI8658 axis transform
unverified. In that state `/mentor_pi/imu` remains invalid and the
firmware does not guess a robot-frame mapping. The SensorTask instead samples
untransformed sensor-frame values into this debugger-only C symbol:

```text
rrclite_imu_characterization_snapshot
```

This snapshot is statically allocated, is not a ROS endpoint, and cannot grant
motor or other actuator authority. Inspect it only with motor power physically
disconnected. Load `mentor_pi_mcu.elf` in GDB, let the target run, halt it, and
read the complete symbol:

```gdb
p/x rrclite_imu_characterization_snapshot.sequence
p rrclite_imu_characterization_snapshot
p/x rrclite_imu_characterization_snapshot.sequence
```

The two sequence reads must match and be even. An odd value means the core was
halted during the single-writer update; continue and halt again. A usable sample
has `valid = 1`, `result_code = 0`, address `0x6a` or `0x6b`, and a timestamp
that advances after continuing. Values are SI units: `acceleration_mps2[3]` and
`angular_velocity_rps[3]`. When `valid = 0`, `result_code` uses the v2 result
numbers (`3` busy, `4` timeout, `5` I/O error, `6` unsupported) and `detail`
identifies the relevant register where available. A busy snapshot is expected
if the sensor did not have a new sample at that exact attempt; continue and
retry.

With the board stationary, record the snapshot for all six physical
orientations: robot +X, -X, +Y, -Y, +Z, and -Z upward. The sensor axis aligned
with gravity should measure approximately `+9.80665` or `-9.80665 m/s^2`, and
the gyroscope should remain near zero. Then record a clearly positive
right-hand rotation about each declared robot +X, +Y, and +Z axis; use the
dominant raw gyroscope component and sign to cross-check the permutation from
the gravity measurements. Record address, revision, both sequence reads,
timestamp, all six values, board orientation or rotation, and firmware digest.
Use that evidence to derive a signed permutation, review it, and then set
`AxisTransform.verified = true` in a later candidate. Do not reinterpret this
raw snapshot as robot-frame ROS telemetry.

The approximately 0.5-second IWDG will reset the target during a long halt. For
this passive, motor-power-disconnected inspection only, GDB may save the debug
freeze register and set bit 12:

```gdb
set $saved_dbgmcu_apb1_fz = *(unsigned int *)0xE0042008
set *(unsigned int *)0xE0042008 = $saved_dbgmcu_apb1_fz | 0x00001000
```

Before reconnecting motor power, restore the saved value (or explicitly clear
bit 12), reset the MCU, and confirm normal watchdog operation:

```gdb
set *(unsigned int *)0xE0042008 = $saved_dbgmcu_apb1_fz
```

If the debugger session no longer has the saved value, reconnect with motor
power still disconnected, clear bit 12 with a read-modify-write, and reset.
Never proceed to a powered test while relying on a frozen watchdog.

### Passive encoder counters over SWD

Before any powered motor test, use the locked image with motor power physically
disconnected to prove the four encoder inputs and their signs. The firmware's
verified peripheral table maps M1 to TIM5, M2 to TIM2, M3 to TIM4, and M4 to
TIM3. TIM2/TIM5 are 32-bit counters; TIM3/TIM4 are 16-bit. The corresponding
counter registers are:

| Motor | Timer | Counter width | `CNT` address |
| --- | --- | ---: | --- |
| M1 | TIM5 | 32 | `0x40000c24` |
| M2 | TIM2 | 32 | `0x40000024` |
| M3 | TIM4 | 16 | `0x40000824` |
| M4 | TIM3 | 16 | `0x40000424` |

Attach using the authoritative locked ELF. With the core halted and motor
power still disconnected, save the debug-freeze register and freeze only IWDG
as in the preceding section. Do not set any timer-freeze bit. Read all four
raw counters:

```gdb
set $saved_dbgmcu_apb1_fz = *(unsigned int *)0xE0042008
set *(unsigned int *)0xE0042008 = $saved_dbgmcu_apb1_fz | 0x00001000
p/x *(unsigned int *)0x40000c24
p/x *(unsigned int *)0x40000024
p/x (*(unsigned int *)0x40000824) & 0xffff
p/x (*(unsigned int *)0x40000424) & 0xffff
```

Turn only M1 by hand in the declared robot-positive direction, read TIM5 again,
then turn it in the negative direction and read again. Repeat the same sequence
for M2/TIM2, M3/TIM4, and M4/TIM3. Record the before/positive/negative values,
direction of physical rotation, and firmware digest. Account for modular wrap
at 32 or 16 bits; do not treat a wrap as an enormous physical step. Exactly one
counter must change for each wheel, its raw delta must reverse sign with the
physical direction, and the matching ROS `encoder_count`/`measured_rps` signs
must agree with the reviewed normalization. Any cross-channel count or
ambiguous sign blocks commissioning.

Restore the exact saved debug value, reset, and verify that the watchdog again
operates normally before reconnecting motor power:

```gdb
set *(unsigned int *)0xE0042008 = $saved_dbgmcu_apb1_fz
monitor reset
continue
```

After the normal reset and Agent reconnection, echo
`/mentor_pi/motors/state` with best-effort/volatile QoS and repeat slow,
separate positive and negative hand turns. Correlate those normalized samples
with the archived raw-counter table; do not try to observe ROS while the core is
halted.

If the saved value is unavailable, follow the fail-safe clear/reset procedure
in the preceding watchdog section. Raw register reads are evidence only; do not
edit polarity or PID values during the board session without a reviewed source
change and a new locked handoff.

## STM32CubeProgrammer GUI over USB serial 1

STM32CubeProgrammer is both a graphical application and a command-line tool.
The normal supported path is `make flash`, which calls the CLI only after all
project safety and provenance checks. A manual GUI operation bypasses that
wrapper, so it is a recovery path for the normal locked image only. Immediately
before opening the GUI, verify the source-bound build and both manifests of the
exact locked handoff selected for the board session:

```sh
./tools/verify_firmware_artifact.sh LOCKED
(cd build/board-handoff/REPLACE_WITH_REVIEWED_DIRECTORY && \
  shasum -a 256 -c SHA256SUMS)
(cd build/board-handoff/REPLACE_WITH_REVIEWED_DIRECTORY/locked && \
  shasum -a 256 -c SHA256SUMS)
```

Enter the bootloader with the BOOT/RST sequence above. In CubeProgrammer select
`UART`, refresh and select the exact CH9102F port, then set 115200 baud, even
parity, 8 data bits, one stop bit, and flow control off. Connect, select the
reviewed handoff's `locked/mentor_pi_mcu-locked.hex`, enable programming
verification, and start programming. On success, release `BOOT`, press `RST`
normally, and close CubeProgrammer before starting the Agent.

The wrapper ultimately invokes this CubeProgrammer CLI shape against its
verified, read-only temporary snapshot (the random directory is removed when
the wrapper exits):

```sh
STM32_Programmer_CLI \
  -c port=/dev/serial/by-id/REPLACE_ME br=115200 P=EVEN db=8 sb=1 fc=OFF \
  -w /tmp/rrclite-flash.RANDOM/mentor_pi_mcu-REPLACE_WITH_SHA256.elf \
  -v
```

The exact executable path depends on the CubeProgrammer installation. On
Ubuntu it is normally in the CubeProgrammer installation directory.

Do not use the raw CLI or GUI for a commissioning image. Use
`make flash-commissioning` so the distinct motor profile and exact second
flash-time acknowledgement are enforced.

For CubeProgrammer over SWD instead, select `ST-LINK`, connect to
`STM32F407VE`, erase the required sectors, program the authoritative
`mentor_pi_mcu.hex`, enable verification, and reset/run the target. The
equivalent SWD CLI shape is:

```sh
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst \
  -w firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.hex -v -rst
```

## J-Link Commander alternative

As with CubeProgrammer, first run
`./tools/verify_firmware_artifact.sh LOCKED`. The raw Commander path is a
recovery option for the locked image only; commissioning must use the gated
UART flash target.

Use device `STM32F407VE`, interface SWD, and a conservative initial speed such
as 4 MHz. A typical interactive sequence is:

```text
device STM32F407VE
si SWD
speed 4000
r
h
loadfile firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.hex
verifybin firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.bin 0x08000000
r
g
exit
```

`loadfile` verifies programmed data by default; the explicit `verifybin` adds a
read-back comparison of the contiguous loadable image at the STM32 flash base.

## Confirmed UART bootloader topology

The RRCLite V1.0 schematic and hardware guide confirm the complete path:
data USB-C to CH9102F, CH9102F TX/RX to PA10/PA9, BOOT0 normally pulled low
with a button to 3.3 V, BOOT1 pulled low, and an active-low RST button. The
board also routes CH9102F handshake signals into an automatic-download circuit,
but the supported first-board procedure uses the deterministic physical
BOOT/RST sequence and does not depend on host-specific DTR/RTS polarity.
Runtime USB communication does not require BOOT0.

## First powered test

Do not skip from flashing directly to powered motor motion. Use this order:

1. Disconnect motor power, mechanically contain the robot with every wheel
   raised, and attach the controller to a current-limited supply.
2. Build and flash the normal motor-locked image with verification. Reset it and
   confirm that it does not enter a reset loop.
3. Connect the data USB-C cable and identify the CH9102F device on the Ubuntu
   host. Use the guarded production installer from the
   [bring-up package guide](../src/mentor_pi_bringup/README.md) to verify exactly
   one serial/`ID_PATH` identity and render the dedicated-group udev rule; do
   not copy the unrendered template. Use `/dev/mentor_pi_mcu` afterward.
4. Start the native Humble micro-ROS Agent at 1,000,000 baud, then the C++
   configuration supervisor. Inspect heartbeat and diagnostics, and test one
   LED, one button, and battery telemetry. The initial image deliberately
   reports the IMU as unverified/invalid; run the explicit
   `imu_characterization_mode` command in the
   [bring-up package guide](../src/mentor_pi_bringup/README.md) and capture the
   raw SWD snapshot below rather than expecting strict IMU preflight to pass.
5. With motor power still disconnected, prove the lock: a selected zero target
   is accepted as a stop, while a selected nonzero target is rejected as
   `UNSUPPORTED`; motor PWM must remain zero. A mixed command containing both
   zero and nonzero selected targets must be rejected atomically.
6. Perform the passive encoder-direction test before powered motion. Rotate one
   wheel at a time by hand in its declared positive direction. Record the raw
   timer-count change using the debugger and the normalized `MotorState`
   `encoder_count`/`measured_rps` sign. Repeat in the negative direction. The
   provisional JGA27 model direction factor is `-1`, derived from the negative
   legacy gains; all other retained profiles currently use `+1`. Any ambiguous
   or contradictory result blocks powered motion until the mapping is corrected
   and reviewed.
7. If all passive checks pass, configure a conservative current limit, verify
   the mechanical containment and motor-power stop, then build and flash the
   guarded commissioning image using the two exact environment variables shown
   above. Record the exact artifact digest.
8. Apply motor power and test only one motor at a time. Command no more than
   0.25 RPS and verify that applied output never exceeds 300 permille. Begin
   below the cap and stop immediately on unexpected direction, excessive
   current, oscillation, encoder disagreement, or sustained saturation.
9. Verify explicit zero/stop, the 200 ms lease stop, Agent loss, and USB loss
   before expanding the test matrix. PWM and bus servos hold their last state;
   make them mechanically safe separately.
10. Record raw and normalized encoder signs, motor direction, current, no-load
    response, provisional gain behavior, battery scale, IMU axes, diagnostics,
    resets, firmware digest, and fixture/current-limit details in the bring-up
    evidence.
11. Rebuild and flash the normal locked image when commissioning is complete.

The host topic `/mentor_pi/configuration/motion_enabled=true` means only that
the supervisor applied configuration successfully. It cannot unlock the MCU,
raise the commissioning caps, or qualify a PID/polarity profile. The bounded
commissioning utility additionally requires exactly one host-local publisher
named `/mentor_pi/configuration_supervisor` on
`/mentor_pi/configuration/motion_authorization`, locks its packed configuration
generation and Agent-session token, and aborts if either changes. Do not replace
that utility with a direct nonzero `ros2 topic pub` command.

Passing this guarded check alone does not release the motor controller. D3 HIL
must qualify every retained motor/channel combination, direction mapping,
current behavior, PID/filter/deadband constants, stop path, and full intended
operating range against the exact candidate image before nonzero motion may be
enabled in a production build.

## Recovery and rollback

An invalid application image cannot permanently remove SWD access. Power-cycle
with the probe connected, select connect-under-reset, halt the core, erase the
application flash, and program the last known-good HEX file. Preserve the ELF,
HEX, map, compiler version, dependency revisions, and SHA-256 digest for every
image tested on hardware.
