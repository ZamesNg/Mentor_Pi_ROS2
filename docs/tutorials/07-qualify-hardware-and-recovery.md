# Tutorial 07: Qualify Hardware and Recovery

Create a machine-captured HIL run and execute only cases supported by a
reviewed instrumented fixture. Missing instruments are reported as blocked,
never converted into a visual pass.

**Run on:** the connected Humble runtime
**Firmware:** locked except during an explicitly guarded motor case
**Fixture:** current-limited, restrained, instrumented, and able to remove
motor power immediately

Previous: [Tutorial 06: Commission One Motor Safely](06-commission-one-motor-safely.md)
Next: [Tutorial 08: Run Stress, Soak, and Release Gates](08-run-stress-soak-and-release-gates.md)

## 1. Freeze the HIL identity

```sh
cd /home/zames/Mentor_Pi && make hil-start
```

Expected result: `HIL RUN READY` followed by a new directory containing the
source revision, CH9102F board serial, and locked firmware SHA-256. Instruments
must add fixture revision, calibration, channels, timestamps, and raw streams
to that directory.

## 2. Run guarded peripheral qualification

**Warning:** PWM and bus servos hold their last accepted state on host loss.
Restrain every powered device and confirm the physical stop before continuing.

```sh
cd /home/zames/Mentor_Pi && make hil-peripheral-check
```

The repository currently reports `BLOCKED: an instrumented peripheral fixture
command is not configured`. This is intentional. Do not replace it with a
generic publisher.

Before this target may pass, the fixture must machine-capture:

- all four motor channels/models, both directions, PID/current/saturation, the
  100 Hz loop, and independent 198–200 ms leases;
- PWM 500/1500/2500 microseconds, offsets, subset merge, 20 ms frames, and
  ±10 microsecond accuracy;
- bus-servo 1- and 16-device arrays, measured ID/baseline, safe position,
  stop/configure deadlines, persistence restoration, and corrupt/missing
  device results;
- LED1/LED2 patterns, LED3 heartbeat status, buzzer priority/resume, host RGB1
  values, rejection of RGB masks 2 and 3, RGB2 RX/TX status, OLED validation, and both buttons
  including debounce, long press, repeat, double/triple click;
- QMI8658 timing/axes/fault response and the five-point battery result; and
- watchdog, USART1/DMA, UART5, I2C, ADC, reset, and transport fault injection.

No motor-lease fixture command exists yet, so the exact physical 200 ms case
remains blocked. Do not invent a command or replace instrument evidence with a
visual observation.

## 3. Run one recovery rehearsal

**Warning:** Remove motor power before USB, Agent, or reset interruption.

```sh
cd /home/zames/Mentor_Pi && make hil-recovery-check
```

The target remains blocked until independent outage/reset timing capture is
configured. A valid implementation must rehearse USB reconnect, Agent restart,
and MCU reset once; prove motor stop within 200 ms, no stale replay, held
PWM/bus state, exact reset cause, and all 21 endpoints within five seconds.

## 4. Close safely

**Warning:** Remove all motor and servo power and restore every persistent
bus-servo value before flashing.

```sh
cd /home/zames/Mentor_Pi && make restore-locked
```

Do not label Tutorial 07 complete while either HIL target reports `BLOCKED`, an
instrument stream is absent, or the final locked read-back fails.

Next: [Tutorial 08: Run Stress, Soak, and Release Gates](08-run-stress-soak-and-release-gates.md).
