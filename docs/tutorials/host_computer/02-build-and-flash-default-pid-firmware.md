# Tutorial 02: Build and Flash the Default PID Firmware

Build, verify, program, read back, and reset the normal closed-loop PID
firmware on the locally connected board.

**Run on:** normal Ubuntu development computer
**Hardware state:** all motors, PWM servos, and bus servos disconnected

Previous: [Tutorial 01: Prepare the Host Computer](01-prepare-ubuntu-development-host.md)
Next: [Tutorial 03: Build and Run the Humble Host](03-build-and-run-humble-host.md)

## 1. Build the firmware

```sh
cd "${HOME}/Mentor_Pi" && make firmware
```

Expected verification reports `motor_mode=PID`, `artifact_mode=NORMAL`,
`control_mode=CLOSED_LOOP`, valid provenance, and required memory headroom.

## 2. Configure the stable serial alias

```sh
cd "${HOME}/Mentor_Pi" && make serial-setup
```

Connect only UART1/download. The helper requires exactly one `1a86:55d4`
CH9102F and installs `/dev/mentor_pi_mcu`. Log out and back in if the
`mentor-pi-serial` group membership is not yet active.

## 3. Flash the local build

**Warning:** Motor power is disconnected. PWM and bus servos unplugged. Close
the Agent, serial terminals, and CubeProgrammer. Complete Tutorials 04 and 05
before any guarded powered work.

```sh
cd "${HOME}/Mentor_Pi" && make flash
```

Type `ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED` when prompted. The helper
programs, reads back, verifies, and resets the board. Stop on unexpected
current, heat, output, device disappearance, or missing verification.

Next: [Tutorial 03: Build and Run the Humble Host](03-build-and-run-humble-host.md).
