# Tutorial 02: Flash the Packaged PID Firmware

Configure the stable serial alias and flash the verified PID firmware included
in the received RDK handoff. Nothing is compiled on the RDK.

**Run on:** RDK X5 connected to the controller board
**Hardware state:** all motors, PWM servos, and bus servos disconnected

Previous: [Tutorial 01: Prepare and Receive the RDK Deployment](01-prepare-ubuntu-development-host.md)
Next: [Tutorial 03: Install and Start RDK Production](03-build-and-run-humble-host.md)

## 1. Configure the stable serial alias

```sh
cd "${HOME}/Mentor_Pi" && make serial-setup
```

Connect only the USB-C connector labelled UART1/download. The helper requires
exactly one `1a86:55d4` CH9102F and installs `/dev/mentor_pi_mcu`.
Log out and back in if the new `mentor-pi-serial` membership is not active.

## 2. Flash the received firmware

**Warning:** Motor power is disconnected. PWM and bus servos unplugged. Close
the Agent, serial terminals, and CubeProgrammer. Passive checks in Tutorial 04
and characterization in Tutorial 05 must pass before guarded powered work.

```sh
cd "${HOME}/Mentor_Pi" && make flash-production
```

The command uses the bundle receipt from Tutorial 01, reports progress, checks
the packaged board release, stops an active managed production target when
needed, and flashes without rebuilding. Type the requested acknowledgement
exactly:

```text
ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

Expected result:

```text
Firmware programming and read-back verification succeeded.
Verified application reset completed; no BOOT or RST press is needed.
```

Stop on unexpected current, heat, actuator output, device disappearance,
activation failure, or missing read-back success. Keep production stopped
until this command succeeds.

Next: [Tutorial 03: Install and Start RDK Production](03-build-and-run-humble-host.md).
