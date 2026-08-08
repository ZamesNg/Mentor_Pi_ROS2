# Tutorial 08: Run Stress, Soak, and Release Gates

Run software, performance, endurance, recovery, and rollback gates for one
exact candidate. A successful process is not automatically a qualified
release when physical metrics remain unobserved.

**Run on:** native `amd64` and native `arm64` runners for architecture gates;
the connected Humble runtime for physical campaigns
**Fixture:** guarded peripherals, current limiting, independent wire/resource
measurement, and a physical stop

Previous: [Tutorial 07: Qualify Hardware and Recovery](07-qualify-hardware-and-recovery.md)
Next: [Tutorial 09: Run the `mentor_pi_hardwares` integration](09-run-mentor-pi-hardwares.md)

## 1. Run the software and firmware gates

```sh
cd /home/zames/Mentor_Pi && make release-software-gates
```

Type `RUN_RELEASE_SOFTWARE_GATES`. The helper runs documentation, Debug
sanitizers, Release tests, TSAN, coverage, formatting, static analysis, fuzz
smoke, firmware reproducibility, target integrity, memory headroom, and locked
artifact provenance. Run the architecture gate separately on native `amd64`
and native `arm64`; do not use slow cross-architecture emulation.

Expected result: zero test/sanitizer/race/static failure, at least 90% line and
80% branch coverage, reproducible loadable bytes, complete fault vectors, and
at least 20% headroom in every memory class. Long 10-million-input fuzz/session
campaigns remain separate release evidence; smoke is not a substitute.

## 2. Run strict connected preflight

Leave `make start` running with the locked candidate.

**Warning:** Motor and servo power remain disconnected and every mechanism is
guarded.

```sh
cd /home/zames/Mentor_Pi && make qualification-preflight
```

Type `ACTUATORS_DISCONNECTED`. The helper runs a strict 60-second read-only
monitor followed by the zero-only 500 Hz motor-command preflight.

Require both passes, heartbeat 2 Hz ±5%, IMU 50 Hz ±5%, continuous session,
monotonic counters/stamps, zero reset/transport/allocation failure, 100 Hz
motor loop, 200 ms lease health, and every complete one-second escaped RX+TX
window below 70,000 bytes.

## 3. Run the canonical 60-minute load

**Warning:** The campaign changes guarded PWM, bus-servo, LED, buzzer, RGB, and
OLED outputs. A wrong bus ID or hold position can move or unload a servo.

```sh
cd /home/zames/Mentor_Pi && make campaign-load
```

Type `PERIPHERALS_DISCONNECTED_OR_GUARDED`, then enter the fixture revision,
measured bus-servo ID, reviewed hold position, tolerance, measured offset, and
current torque state. The helper auto-detects source/host revisions, firmware
SHA-256, board serial, ROS domain, and output ID.

Mode `load500` runs exactly 3600 seconds and publishes only zero motor targets.
Require 1,800,000 valid commands, p99 fresh-field age at most 20 ms, every age
below 100 ms, no reset/reconnect/resource growth, and traffic below 70,000
bytes/s. Nonzero full-range qualification requires a separately reviewed
production-authority candidate and physical fixture.

## 4. Run the canonical 24-hour soak

**Warning:** Recheck guarding, supply, bus hold state, logging capacity, and
the physical stop immediately before the uninterrupted run.

```sh
cd /home/zames/Mentor_Pi && make campaign-soak
```

Enter the same reviewed fixture values. Mode `soak` runs exactly 24 hours.
Require zero crash, reset, deadlock, endpoint loss, stale replay, post-seal
allocation, counter regression, resource growth, stack violation, unexplained
gap over twice the declared period, or excessive traffic.

## 5. Run three 100-cycle recovery campaigns

**Warning:** An independent observer/fixture must perform and timestamp every
physical disconnect, Agent interruption, or MCU reset.

```sh
cd /home/zames/Mentor_Pi && make campaign-recovery
```

Run this command three times and select `reconnect_usb`, `reconnect_agent`, and
`reset_mcu` once each. Each mode uses the validated 7200-second default timeout
and requires 100 cycles, motor stop within 200 ms where applicable, no stale
replay or unintended reset, held PWM/bus state, all endpoints within five
seconds, and a fresh command before any later motion.

## 6. Restore the release-safe image

Stop the runtime first.

**Warning:** Remove motor and servo power before the final flash.

```sh
cd /home/zames/Mentor_Pi && make restore-locked
```

Require programming/read-back success and a later locked motor-rejection pass.
Generated campaign output intentionally remains
`release_qualification: INCOMPLETE` whenever independent traffic, physical
motion, outage timing, target stack headroom, or fault measurements are absent.

The release is blocked unless every required instrument file is present and
checksummed, all HIL gates pass, and the final installed image is verified
`LOCKED`.
