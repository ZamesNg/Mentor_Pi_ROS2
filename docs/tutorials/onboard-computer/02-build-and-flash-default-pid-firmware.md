# Onboard Computer Tutorial 02: Build and Flash the Default PID Firmware

Build, provenance-check, program, read back, and automatically reset the normal
closed-loop PID firmware. No firmware handoff is needed for a local board.

**Run on:** the connected RDK X5 onboard computer, without Docker
**Hardware state:** all motors, PWM servos, and bus servos disconnected

Previous: [Tutorial 01: Prepare the Ubuntu Development Host](01-prepare-ubuntu-development-host.md)
Next: [Tutorial 03: Build and Run the Humble Host](03-build-and-run-humble-host.md)

## 1. Build the default PID image

```sh
cd /home/zames/Mentor_Pi && make firmware
```

On Ubuntu 22.04 arm64 this command generates the source-locked Humble
micro-ROS library natively and builds with the SHA-256-verified Arm GNU 13.2.1
toolchain prepared in Tutorial 01. Stop if any command attempts Docker.

Expected result: the artifact verifier reports `motor_mode=PID`,
`artifact_mode=NORMAL`, `control_mode=CLOSED_LOOP`, `release_qualified=0`, valid
provenance, and at least 20% headroom in every memory class. Stop on stale
metadata, an invalid profile, or any build, hash, vector, or memory failure.

## 2. Configure the stable serial alias once

Connect only the USB-C connector labelled UART1/download.

```sh
cd /home/zames/Mentor_Pi && make serial-setup
```

The helper requires exactly one `1a86:55d4` CH9102F and asks for
`CONFIGURE_SERIAL_ACCESS` before using `sudo`. It installs
`/dev/mentor_pi_mcu` and the `mentor-pi-serial` group. Log out and back in after
the first successful run so every normal host tool sees the membership. The
guided one-line flash commands automatically activate an already-granted but
not-yet-active membership for their own process. Stop if the identity is
ambiguous or the alias is not readable and writable afterward.

## 3. Flash the default PID image without touching BOOT or RST

**Warning:** Motor power is disconnected. PWM and bus servos unplugged. Close
the Agent, serial terminals, and CubeProgrammer before continuing. The default
PID image remains unqualified (`release_qualified=0`) until its HIL evidence is
complete. Passive checks (Tutorial 04) and characterization (Tutorial 05) MUST
be completed before any guarded powered work.

```sh
cd /home/zames/Mentor_Pi && make flash
```

Type the requested bootloader acknowledgement exactly:

```text
ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

The helper validates the CH9102F, obtains exclusive ownership, uses DTR/RTS to
enter the STM32 ROM bootloader, programs at 115200 baud/8E1, verifies the
hash-named snapshot, and resets the verified image into normal application
boot. The application transport remains 1,000,000 baud/8N1.

Expected result:

```text
Firmware programming and read-back verification succeeded.
Verified application reset completed; no BOOT or RST press is needed.
```

Automatic BOOT control is implemented but remains hardware-unverified until
this exact command succeeds on the board. If activation fails before any
programming, the helper offers one fallback: hold BOOT, tap RST, release BOOT,
and press Enter. A programming, verification, or interruption failure never
automatically resets into a possibly incomplete image.

Stop on unexpected current, heat, actuator output, device disappearance, hash
change, activation failure after the fallback, or missing read-back success.
Do not run CubeProgrammer or `stm32flash` directly.

Next: [Tutorial 03: Build and Run the Humble Host](03-build-and-run-humble-host.md).
