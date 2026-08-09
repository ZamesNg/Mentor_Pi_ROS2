# Onboard Computer Tutorial 07: Run Native and Physical Gates

Run Docker-free arm64 software checks plus performance, endurance, and recovery
gates for one exact default PID candidate. This track does not run fuzzing or
claim complete pinned-container software qualification.

**Run on:** the RDK X5 onboard computer for native arm64 and physical campaigns
**Fixture:** guarded peripherals, current limiting, independent wire/resource
measurement, and a reachable physical stop

Previous: [Tutorial 06: ROS 2 CLI Hardware Checkout](06-ros2-cli-hardware-checkout.md)
Next: [Tutorial 08: Run the `mentor_pi_hardwares` Integration](08-run-mentor-pi-hardwares.md)

## 1. Run the onboard native gates

```sh
cd /home/zames/Mentor_Pi && make release-onboard-gates
```

Type `RUN_ONBOARD_NATIVE_GATES`. The helper runs documentation checks, the
native arm64 Humble host build/tests, Debug/Release firmware-domain tests,
sanitizers, TSan, two clean firmware builds for reproducibility, memory
headroom, and PID artifact provenance.

Expected: zero test, sanitizer, race, reproducibility, memory, or provenance
failure. The verified artifact classification is
`NORMAL_CLOSED_LOOP_DEFAULT`, with `motor_mode=PID`,
`control_mode=CLOSED_LOOP`, and `release_qualified=0` pending HIL evidence.

Clang 18 fuzzing, coverage, cross-target static analysis, formatting parity,
amd64, and pinned-container evidence belong to the normal-computer Tutorial
07. An onboard pass must never be reported as the complete software release
suite.

## 2. Run the connected zero-command preflight

Keep the direct `ros2 launch mentor_pi_bringup controller.launch.py` from
Tutorial 06 running with the same verified PID candidate. Do not start a
second runtime while it owns the serial device.

**Warning:** Motor and servo power remain disconnected. The PID firmware can
accept valid nonzero motor targets, so electrical disconnection is mandatory
for this zero-only preflight.

```sh
cd /home/zames/Mentor_Pi && make qualification-preflight
```

Type `ACTUATORS_DISCONNECTED`. The helper runs a strict 60-second read-only
monitor followed by a 60-second 500 Hz all-zero motor-command monitor.

Require heartbeat 2 Hz ±5%, IMU 50 Hz ±5%, one continuous Agent session,
monotonic stamps/counters, zero reset/transport/allocation faults, the 100 Hz
motor loop, lease safety, and complete one-second escaped RX+TX windows below
70,000 bytes.

## 3. Run the canonical 60-minute load

**Warning:** The campaign changes guarded PWM, bus-servo, LED, buzzer, RGB1,
and optional OLED outputs. A wrong bus ID or hold position can move or unload a
servo. Campaign motor targets remain hard-coded to zero.

```sh
cd /home/zames/Mentor_Pi && make campaign-load
```

Type `PERIPHERALS_DISCONNECTED_OR_GUARDED`, then enter the fixture revision,
measured bus-servo ID, reviewed hold position, tolerance, measured offset, and
current torque state. The helper records source/host revisions, PID firmware
SHA-256, board serial, ROS domain, and output identity.

Mode `load500` runs exactly 3600 seconds and requires 1,800,000 valid zero
motor commands, p99 fresh-field age at most 20 ms, every age below 100 ms, no
reset/reconnect/resource growth, and traffic below 70,000 bytes/s.

## 4. Run the canonical 24-hour soak

**Warning:** Recheck guarding, supply, bus hold state, logging capacity, and
the physical stop immediately before the uninterrupted run.

```sh
cd /home/zames/Mentor_Pi && make campaign-soak
```

Mode `soak` runs exactly 24 hours. Require zero crash, reset, deadlock,
endpoint loss, stale replay, post-seal allocation, counter regression,
resource growth, stack violation, unexplained timing gap, or excess traffic.

## 5. Run the recovery campaigns

**Warning:** An independent observer or fixture performs and timestamps every
physical USB disconnect, Agent interruption, or MCU reset. Actuators remain
disconnected or equivalently guarded throughout.

```sh
cd /home/zames/Mentor_Pi && make campaign-recovery
```

Run the command three times and select `reconnect_usb`, `reconnect_agent`, and
`reset_mcu` once each. Each mode requires 100 cycles, bounded stop where
applicable, no stale replay or unintended reset, held PWM/bus state, all
endpoints back within five seconds, and a fresh command before later motion.

## 6. Interpret the evidence boundary

Keep every generated campaign directory and independent instrument file bound
to the exact PID ELF hash. The campaign utility intentionally reports physical
release qualification as incomplete whenever an independent motion, timing,
electrical, thermal, stack, or fault metric is missing.

Do not turn a successful mock, zero-command load, process exit status, or
visual observation into a powered-motion, PID-performance, or complete HIL
claim. Record those claims only from the numbered guarded procedure and its
instrumented evidence.
