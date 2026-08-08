# Tutorial 04: Run Passive Board Bring-Up

Prove the locked MCU, ROS graph, telemetry rates, motor rejection, passive
outputs, and basic recovery. Detailed firmware history is recorded in the
[firmware stabilization log](../firmware-stabilization-log.md).

**Run on:** the connected native-or-Docker Humble runtime
**Hardware state:** motor power disconnected; PWM and bus servos unplugged;
wheels contained

Previous: [Tutorial 03: Build and Run the Humble Host](03-build-and-run-humble-host.md)
Next: [Tutorial 05: Characterize Board Hardware](05-characterize-board-hardware.md)

## 1. Run the passive core check

Start with a fresh `make start` in terminal A after flashing. Run this passive
check before `make peripheral-smoke`; MCU diagnostic counters are monotonic for
the session, so an earlier smoke failure would intentionally remain visible.

**Warning:** Motor power is disconnected. PWM and bus servos unplugged. Only
high-impedance scope/logic-analyzer probes may touch actuator outputs.

```sh
cd /home/zames/Mentor_Pi && make passive-check
```

Type `ACTUATORS_DISCONNECTED`, then answer whether the 128×32 OLED is
physically installed. The helper enters the matching Humble runtime, verifies
the authoritative locked artifact and both nodes, records a new 60-second
diagnostic archive, runs first-board IMU characterization, publishes an
all-zero motor command, and proves that a selected `0.01 RPS` command is
rejected by the normal image. It prints each capture phase and a progress line
every five seconds during the observation.

Expected result: `CHARACTERIZATION PASS`, heartbeat `2 Hz ±5%`, IMU `50 Hz
±5%`, one continuous session, all 21 endpoints, zero
transport/reset/allocation faults, zero physical motor output, and `PASSIVE
CHECK PASS`. This proves live IMU transport, not the provisional axis mapping;
Tutorial 05 records six stationary faces before Tutorial 08 runs the strict
`PREFLIGHT PASS` gate.

With USB-only power and no battery connected, the exact fail-closed battery
state (`0 mV`, invalid, not below threshold) is accepted in this first-board
characterization only. Strict qualification still requires a valid calibrated
battery input.

If the OLED is declared not installed, characterization excuses only the exact
repeated SSD1306 initialization NACK and prints `PASS WITH LIMITATION: OLED NOT
INSTALLED/NOT TESTED`. This allows the remaining first-board work to continue;
it is not OLED evidence. Tutorial 07 and every release gate remain blocked
until the display is installed and passes its HIL checks. Declaring the OLED
absent never excuses an I2C timeout or any other peripheral error.

On failure, the helper prints the final `CHARACTERIZATION FAIL` line and keeps
the archive for review. Stop on a missing `/mentor_pi/controller`, supervisor
state `DISCONNECTED`, any new peripheral error, counter regression, Agent
reconnect, USART/DMA error, nonzero motor state, or traffic at or above 70,000
combined bytes/s in a complete one-second window.

## 2. Run passive peripheral smoke

**Warning:** Do not attach servo mechanisms. The test deliberately changes
LED1/LED2, the buzzer, host-controlled RGB1, OLED text, and unloaded PWM pins.
LED3 and RGB2 remain firmware status indicators.

```sh
cd /home/zames/Mentor_Pi && make peripheral-smoke
```

Type `PASSIVE_OUTPUTS_GUARDED` and answer the same OLED-presence question. The
helper exercises `/mentor_pi/leds/command` for `for led_id in 1 2`,
`/mentor_pi/buzzer/command`, RGB1 through `/mentor_pi/rgb/command`,
both OLED lines through `/mentor_pi/oled/command`, all four unloaded PWM
channels at 1500 microseconds, and both `button_id` 1 and 2 through
`/mentor_pi/buttons/events`.
It sends only the passive `/mentor_pi/bus_servos/get_state` read path; it never
publishes `BusServoCommand` motion.

When the OLED is not installed, its command and visual check are skipped and
the result is labelled `OLED NOT INSTALLED/NOT TESTED`; all other smoke steps
still run. This limitation remains blocked for Tutorial 07 and release.

When prompted, press and release button 1, then button 2, within ten seconds
each. The helper rejects a missing or wrong-button event and always turns the
LED1/LED2, buzzer, and RGB1 off when it exits, including after a failure. It
does not override LED3 or RGB2 status.

Expected result: two visible cycles on LED1 and LED2, RGB1 blue then off,
two short 1 kHz beeps, correct OLED text, distinct 20 ms PWM frames, and a
correctly identified press event from each button. The helper explicitly
cleans up LED1/LED2, the buzzer, and RGB1 and leaves PWM neutral. LED3 changes
only after a successful ROS heartbeat publish. RGB2 red/green pulse with
sampled RX/TX progress and blue remains off. Full button-event ordering and
waveform/timing qualification require the Tutorial 07 instrumented fixture.

Stop on unexpected motion, persistent buzzer output, wrong channel ownership,
malformed button ordering, reset, heat, or unexplained current.

## 3. Check Agent and USB recovery

**Warning:** Actuators remain disconnected. Do not hold BOOT or open the serial
port with another process.

```sh
cd /home/zames/Mentor_Pi && make recovery-check
```

First type `ACTUATORS_DISCONNECTED`. The helper prints node/topic/service
counts while waiting up to 30 seconds for the initial baseline. At the first
prompt, keep USB connected, restart `make start`, and press Enter immediately
after it starts. Only at the second prompt should you disconnect the CH9102F
data cable for two seconds and reconnect it. The helper prints discovery
progress and requires the complete 20-endpoint graph plus a fresh heartbeat
within five seconds after each recovery operation, as well as the stable alias
after USB re-enumeration.

Expected result: `RECOVERY CHECK PASS`, all endpoints within five seconds, a
fresh session generation, no stale command replay, and no manual RST. Stop if
LED3 remains off, the buzzer repeats, the alias fails to return, or the
controller does not reappear.

Next: [Tutorial 05: Characterize Board Hardware](05-characterize-board-hardware.md).
