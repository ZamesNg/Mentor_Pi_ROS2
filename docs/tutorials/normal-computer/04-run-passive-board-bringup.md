# Normal Computer Tutorial 04: Run Passive Board Bring-Up

Prove the PID artifact, ROS graph, telemetry rates, zero motor state, passive
outputs, and basic board inputs before any powered actuator work. Detailed
firmware history is recorded in the
[firmware stabilization log](../../firmware-stabilization-log.md).

**Run on:** the normal computer's connected Docker Humble runtime
**Hardware state:** motor power disconnected; PWM and bus servos unplugged;
wheels contained

Previous: [Tutorial 03: Build and Run the Humble Host](03-build-and-run-humble-host.md)
Next: [Tutorial 05: Characterize Board Hardware](05-characterize-board-hardware.md)

## 1. Run the passive core check

Start with a fresh `make start` in terminal A after flashing. Run this check
before `make peripheral-smoke`; diagnostic counters are monotonic for the
session, so an earlier output failure would remain visible.

**Warning:** The default PID firmware accepts valid nonzero motor commands.
Physically disconnect motor power and every servo mechanism. Only
high-impedance scope or logic-analyzer probes may touch actuator outputs.

```sh
cd /home/zames/Mentor_Pi && make passive-check
```

Type `ACTUATORS_DISCONNECTED`, then answer whether the 128x32 OLED is
installed. The helper verifies the authoritative PID artifact and both nodes,
records a 60-second diagnostic archive, runs first-board IMU characterization,
publishes an all-zero motor command, and proves the reported motor targets
remain zero while actuator power is disconnected. It never publishes a
nonzero motor target.

Expected: `CHARACTERIZATION PASS`, heartbeat `2 Hz +/-5%`, IMU `50 Hz +/-5%`,
one continuous session, all 21 MCU endpoints, zero transport/reset/allocation
faults, zero motor target, and `PASSIVE CHECK PASS`. This proves live IMU
transport, not a complete physical axis or PID qualification.

With USB-only power and no battery, the fail-closed state (`0 mV`, invalid,
not below threshold) is accepted only for first-board characterization. Strict
qualification requires a valid calibrated battery input.

If the OLED is declared absent, the helper excuses only repeated SSD1306
initialization NACKs and labels the result `OLED NOT INSTALLED/NOT TESTED`.
That is not OLED evidence and does not excuse an I2C timeout or another
peripheral fault.

Stop on a missing controller or supervisor, configuration gate failure, a new
peripheral error, counter regression, Agent reconnect, USART/DMA error,
nonzero motor target, or traffic at or above 70,000 combined bytes/s in a
complete one-second window.

## 2. Run passive peripheral smoke

**Warning:** Do not attach servo mechanisms. The test changes LED1/LED2, the
buzzer, host-owned RGB1, optional OLED text, and unloaded PWM pins. LED3 and
RGB2 remain firmware status indicators.

```sh
cd /home/zames/Mentor_Pi && make peripheral-smoke
```

Type `PASSIVE_OUTPUTS_GUARDED` and answer the OLED-presence question. The
helper exercises both host LEDs, two short beeps, RGB1, both optional OLED
lines, all four unloaded PWM outputs at 1500 microseconds, both buttons, and a
read-only bus-servo query. It sends no bus-servo move and no nonzero motor
command.

Press and release button 1, then button 2, within ten seconds each. The helper
rejects a missing or wrong-button event and always turns LED1/LED2, the buzzer,
and RGB1 off on exit. It does not override LED3 or RGB2.

Expected: two visible cycles on LED1 and LED2, RGB1 blue then off, two short
1 kHz beeps, correct optional OLED text, distinct neutral PWM frames, and one
correctly identified event from each button. Full waveform, timing, recovery,
and endurance evidence belongs to Tutorial 07.

Stop on unexpected motion, persistent buzzer output, wrong channel ownership,
malformed button ordering, reset, heat, or unexplained current.

Next: [Tutorial 05: Characterize Board Hardware](05-characterize-board-hardware.md).
