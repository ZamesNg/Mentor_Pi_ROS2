# Board-Arrival Bring-Up Evidence Record

Copy this file for each board and session; keep this source copy blank. This
record covers first-day bring-up only. It does **not** close D3 hardware parity
or D5 release qualification. Follow the detailed
[flashing and first-bring-up procedure](flashing-and-first-bringup.md) whenever
this checklist refers to it.

## Session record

| Field | Recorded value |
| --- | --- |
| Date, operator, reviewer | |
| Board revision and serial/label | |
| Host architecture, Ubuntu, ROS 2 | |
| Flash path, optional probe, USB data cable, fixture | |
| Supply voltage and current limit | |
| micro-ROS Agent version/command | |
| Supervisor configuration and revision | |
| Locked ELF/HEX path and SHA-256 | |
| Commissioning ELF/HEX path and SHA-256 | `NOT BUILT` until the commissioning gate passes |
| Evidence directory or test-run ID | |

Stop immediately on unexpected motor motion, ambiguous pin or polarity
evidence, overcurrent, smoke/heat, repeated reset, watchdog disabled during a
powered test, or loss of the physical motor-power stop. Record the observation,
remove motor power, and return to the locked image.

## 1. Prepare a passive fixture

- [ ] Verified the separate architecture-matched host handoff with
  `sha256sum --check SHA256SUMS` and recorded its release ID, source fingerprint,
  pinned builder digest, and pinned Agent source-lock digest. The preparation,
  promotion, and Agent commands came from
  [Host preparation and handoff](host-preparation-and-handoff.md).
- [ ] On the Ubuntu 24.04 VM, the host handoff was promoted from its copied
  `host/` prefix, the original build staging path was not required, and the
  pinned native Agent installation completed before any controller service was
  enabled.
- [ ] Motor power is physically disconnected.
- [ ] All four PWM-servo connectors are unplugged from servos/mechanisms and
  the bus-servo power/device connector is unplugged. The locked firmware starts
  valid 1500 microsecond PWM-servo pulses during safe boot; disconnected motor
  power does not make an attached servo mechanism passive.
- [ ] Only high-impedance scope/logic-analyzer probes are attached to actuator
  outputs until the electrical mapping checks in Section 4 are complete.
- [ ] Every wheel is raised or equivalently contained.
- [ ] The motor-power stop is reachable and has been tested.
- [ ] The board supply is current-limited; voltage and limit are recorded.
- [ ] If an SWD probe is attached, grounds were connected before SWD signals;
  target 3.3 V is not driven by both the probe and board.
- [ ] The `BOOT` and `RST` buttons, their board labels, and the BOOT0/RST
  sequence were observed and photographed before UART flashing.
- [ ] If raw timer/IMU characterization or debugging will be attempted,
  `SWDIO`, `SWCLK`, `NRST`, `GND`, target-voltage sense, and connector
  orientation were also observed and photographed. UART flashing alone does
  not satisfy those later SWD evidence gates.
- [ ] The CH9102F **data** USB-C connector was identified. The 5 V/5 A
  power-only USB-C connector is not used for communication.

Evidence/notes:

```text

```

## 2. Flash and prove the locked image first

- [ ] Built the normal image with `./tools/build_firmware.sh`, without either
  motor-commissioning environment variable.
- [ ] On the Ubuntu 24.04 host, recorded `pio --version` and ran the
  non-flashing PlatformIO preparation/verification hook successfully. This
  resolves the pinned `ststm32@17.6.0` platform before the board session and
  proves PlatformIO selects the authoritative locked ELF:

  ```sh
  pio --version
  pio run -e rrclite_uart -t checkprogsize
  ```

  Use `rrclite_stlink` for an ST-Link or `rrclite_jlink` for a J-Link.
  `checkprogsize` does not access or flash a target; a missing dependency,
  stale build, commissioning artifact, or source mismatch must be resolved
  before continuing.
- [ ] Verified the source-bound authoritative ELF and the prepared first-board
  handoff manifests, then recorded SHA-256 digests before flashing:

  ```sh
  readonly HANDOFF_DIRECTORY="${PWD}/build/board-handoff/REPLACE_WITH_REVIEWED_DIRECTORY"
  test -d "${HANDOFF_DIRECTORY}/locked"
  ./tools/verify_firmware_artifact.sh LOCKED
  (cd "${HANDOFF_DIRECTORY}" && sha256sum --check SHA256SUMS)
  (cd "${HANDOFF_DIRECTORY}/locked" && sha256sum --check SHA256SUMS)
  cmp firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf \
    "${HANDOFF_DIRECTORY}/locked/mentor_pi_mcu-locked.elf"
  cmp firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt \
    "${HANDOFF_DIRECTORY}/locked/BUILD-METADATA.txt"
  ```

  Replace the fail-closed directory component before running the block and
  record that reviewed directory in the session table.
  PlatformIO flashes the authoritative ELF, not the evidence copy under the
  handoff directory; both `cmp` commands must therefore be silent and return
  zero. Record the verified ELF, HEX, and loadable BIN digests in the session
  table; do not copy values from an earlier board session.
  If any source or artifact changed, stop and create a new reproducible handoff
  instead of updating this evidence record by hand.
- [ ] Flashed the locked image and enabled read-back verification. For the
  no-probe path, connected USB serial 1/download, disconnected all actuators,
  entered the ROM bootloader by holding `BOOT`, tapping `RST`, and releasing
  `BOOT`, then ran:

  ```sh
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
    pio run -e rrclite_uart -t upload \
      --upload-port /dev/cu.wchusbserial-REPLACE_ME
  ```

  On Ubuntu use the exact CH9102F `/dev/serial/by-id/...` path. After successful
  programming and verification, released `BOOT` and tapped `RST` normally.
  The bootloader connection was 115200/8E1; CubeProgrammer was closed before
  the Agent opened the same device at the runtime 1,000,000/8N1 settings.

  For an ST-Link instead:

  ```sh
  pio run -e rrclite_stlink -t upload
  ```

  Use `rrclite_jlink` only when the connected probe is a J-Link. The detailed
  guide gives the locked-only CubeProgrammer GUI and J-Link Commander
  alternatives.
- [ ] Reset/run is stable; reset cause and diagnostics show no reset loop.
- [ ] Before starting the Agent or sending any ROS command, every motor-bridge
  drive PWM output remains zero. The four distinct PWM-servo pins instead emit
  the intentional 1500 microsecond reset-default pulses and remain connected
  only to passive measurement equipment at this gate.

Programmer command, verification result, scope/debug evidence, and diagnostic
counters:

```text

```

**Gate:** do not build or flash a commissioning image until every item above
and the passive checks below pass.

## 3. Verify USB, Agent, and ROS before actuators

- [ ] Ubuntu enumerates the CH9102F and `/dev/mentor_pi_mcu` resolves to the
  expected device.
- [ ] The udev rule is installed; no other process owns the serial device.
- [ ] The native Jazzy micro-ROS Agent opens the port at 1,000,000 baud, 8N1,
  with no hardware flow control.
- [ ] The C++ supervisor starts and applies the recorded configuration.
- [ ] `/mentor_pi/controller`, heartbeat, and diagnostics appear with a new
  session ID and no transport/DMA fault.
- [ ] The installed read-only diagnostic collector runs the explicit first-board
  monitor for 60 seconds, preserves its complete support bundle, and reports
  `CHARACTERIZATION PASS`:

  ```sh
  readonly HANDOFF_DIRECTORY="${PWD}/build/board-handoff/REPLACE_WITH_REVIEWED_DIRECTORY"
  readonly EVIDENCE_DIRECTORY="${PWD}/evidence/first-board-$(date -u +%Y%m%dT%H%M%SZ)"
  sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/capture_board_diagnostics \
    --output "${EVIDENCE_DIRECTORY}" \
    --handoff-directory "${HANDOFF_DIRECTORY}" \
    --repository-root "${PWD}" \
    --qualification imu-characterization \
    --qualification-duration-sec 60
  ```

  The collector itself opens no serial transport, publishes no ROS command,
  calls no service, changes no service state, and flashes nothing. A failed
  prerequisite or characterization returns nonzero but still preserves the
  printed `.tar.gz` and digest; inspect `SUMMARY.txt`, `command-status.tsv`, and
  `qualification.txt` instead of discarding the failed evidence.

- [ ] The result shows the expected unverified-IMU condition only: final
  `DEGRADED`, time synchronized, `IMU_HEALTHY` clear, exactly one IMU
  `UNSUPPORTED` diagnostic, and no other fault. Because the best-effort
  volatile IMU topic publishes its initial state only once, zero IMU samples
  are valid for a late-joining monitor; if one arrives it must be invalid,
  finite, and zero-stamped, and a duplicate fails characterization.
  Firmware emits `UNSUPPORTED` only after one successful raw QMI8658
  acquisition; that marker does not validate the board-axis transform.
- [ ] Only after `CHARACTERIZATION PASS`, with motor power still disconnected,
  an all-motor zero command is accepted as a stop and every motor-bridge drive
  PWM output remains zero.
- [ ] A selected nonzero command is rejected as `UNSUPPORTED` and changes no
  output.
- [ ] A mixed selected zero/nonzero command is rejected atomically and changes
  no output. These two deliberate rejections increment persistent diagnostic
  counters, which is why they must not precede the exact characterization
  window in the same MCU boot. Use the exact locked-image commands and
  before/after counter expectations in the
  [ROS 2 CLI examples](ros2-cli-examples.md#prove-the-normal-image-motor-lock),
  never a remembered or improvised nonzero command.

Use the [bring-up package guide](../src/mentor_pi_bringup/README.md) for launch
and monitor commands and the [ROS 2 CLI examples](ros2-cli-examples.md) for
schema-correct inspection. Record device identity, commands, session ID,
endpoint count, counter snapshots, and log paths:

```text

```

`CHARACTERIZATION PASS` is not strict preflight, IMU-axis, D3 HIL, or release
evidence. After a reviewed IMU transform is installed, this mode must fail and
the default strict monitor must pass.

## 4. Record passive polarity and retained peripherals

Keep motor power disconnected. Do not infer polarity from the schematic or
legacy constants when the board can be measured.

- [ ] PWM servos and bus-servo devices/mechanisms remain disconnected before
  running the exact
  [locked-image passive peripheral smoke sequence](ros2-cli-examples.md#run-the-locked-image-passive-peripheral-smoke-sequence).
- [ ] Recorded a diagnostics snapshot before that sequence and another after
  cleanup. No command rejection or peripheral error/timeout counter changed,
  except the explicitly recorded `TIMEOUT` from an optional read-only bus query
  made with no servo attached.
- [ ] Performed raw IMU characterization using the debugger-only snapshot and
  coherent-sequence procedure in
  [Raw IMU axis characterization over SWD](flashing-and-first-bringup.md#raw-imu-axis-characterization-over-swd).
  A UART-only session must leave this unchecked and stop before powered motor
  commissioning; a successful UART flash is not substitute evidence.

| Check | Expected or observation to record | Evidence | Pass |
| --- | --- | --- | --- |
| Motor 1 encoder, hand-turned + then - | Raw count delta; normalized count/RPS sign | | [ ] |
| Motor 2 encoder, hand-turned + then - | Raw count delta; normalized count/RPS sign | | [ ] |
| Motor 3 encoder, hand-turned + then - | Raw count delta; normalized count/RPS sign | | [ ] |
| Motor 4 encoder, hand-turned + then - | Raw count delta; normalized count/RPS sign | | [ ] |
| QMI8658 identity and raw axes | Address/revision; six faces and +X/+Y/+Z rotations, or documented open characterization | | [ ] |
| Buttons 1 and 2 | Idle/pressed polarity and events | | [ ] |
| LEDs 1--3 | Correct pin, polarity, and independent response | | [ ] |
| Buzzer | Correct pin/polarity; brief safe pattern | | [ ] |
| RGB pixels 1 and 2 | Correct order and independent colors | | [ ] |
| OLED | Correct bus/address; both host lines and battery line | | [ ] |
| Battery ADC | Bench voltage, raw/VREFINT data, reported mV | | [ ] |
| PWM servo outputs 1--4 | Disconnected/analyzer-only: 1500 us reset default, four distinct commanded pulses, common-frame timing, then confirmed 1500 us cleanup | | [ ] |
| Bus-servo port | Device power disconnected for waveform check, or exactly one safely restrained supported servo for baseline read; no move/configure/persistent write | | [ ] |

List every mismatch or untested item. An ambiguous encoder/motor mapping blocks
powered motor work even if unrelated peripherals pass.

```text

```

## 5. Guarded one-motor commissioning

Commission only after Sections 1--4 pass and a second person reviews the
fixture, stop, current limit, locked-image evidence, and selected channel.

Reviewer/sign-off and selected motor: ______________________________________

- [ ] Built with both exact acknowledgements and saved the resulting artifacts
  before another build:

  ```sh
  RRCLITE_MOTOR_COMMISSIONING=1 \
  RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
    ./tools/build_firmware.sh
  ```

- [ ] Flashed and verified the recorded commissioning digest using a distinct
  commissioning environment and the exact second acknowledgement. For UART,
  re-entered the ROM bootloader with all wheels raised/current-limited and ran:

  ```sh
  RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  RRCLITE_COMMISSIONING_UPLOAD_ACK=MOTORS_RAISED_CURRENT_LIMITED \
    pio run -e rrclite_uart_commissioning -t upload \
      --upload-port /dev/cu.wchusbserial-REPLACE_ME
  ```

  For ST-Link (use `rrclite_jlink_commissioning` instead for J-Link):

  ```sh
  RRCLITE_COMMISSIONING_UPLOAD_ACK=MOTORS_RAISED_CURRENT_LIMITED \
    pio run -e rrclite_stlink_commissioning -t upload
  ```

  Use `pio debug -e rrclite_stlink --interface gdb` (or the matching J-Link or
  commissioning environment). Do not add `-t nobuild`. PlatformIO debug is
  attach-only; it never replaces
  this gated upload step.
- [ ] Cleared any temporary debugger IWDG freeze, reset, and confirmed normal
  watchdog operation before connecting motor power.
- [ ] Connected motor power with all wheels contained and the recorded current
  limit active.
- [ ] Confirmed the supervisor's transient-local
  `/mentor_pi/configuration/motion_enabled` value is `true` and
  `/mentor_pi/configuration/motion_authorization` has exactly one publisher,
  whose node identity is `/mentor_pi/configuration_supervisor` and whose packed
  generation/session token is nonzero.
- [ ] Confirmed no other process publishes `/mentor_pi/motors/command`, and then
  ran the bounded C++ utility for the reviewer-approved channel (change
  `motor_id` only as signed off). The utility itself rechecks both graph
  conditions and fails closed:

  ```sh
  ros2 run mentor_pi_bringup motor_commissioning --ros-args \
    -p acknowledgement:=MOTORS_RAISED_CURRENT_LIMITED \
    -p motor_id:=1 \
    -p target_rps:=0.05 \
    -p duration_ms:=500
  ```

- [ ] The utility's exit status was 0 and its archived outcome line records the
  expected nonzero session, selected target, encoder delta, measured response,
  `target_observed=true`, `physical_response_observed=true`,
  `zero_confirmed=true`, and `failure=NONE`. At the final observation, all four
  reported targets were zero and all four `abs(measured_rps)` values were
  strictly below 0.002 RPS.
- [ ] Verified the run used a target magnitude from 0.01 through 0.25 RPS, the
  selected response had the commanded sign and reached at least the greater of
  0.002 RPS or 10% of the target, and the selected encoder changed by at least
  two ticks in the commanded direction.
- [ ] Verified physical direction agrees with encoder direction and reported
  state.
- [ ] Verified applied output never exceeds 300 permille.
- [ ] Observed no excessive current, oscillation, encoder disagreement,
  sustained saturation, selected speed above 0.50 RPS, or motion on another
  channel above 0.02 RPS/by at least two encoder ticks.
- [ ] Verified no drive-command interval exceeded 100 ms and no motor lease,
  motor-watchdog counter, or `watchdog_stop_mask` bit changed during the run.
- [ ] Verified the final observed motor state was still all zero; a zero sample
  followed by a returning target is a failed stop, not a pass.

| Target RPS | Measured RPS | Output permille | Supply current | Direction/sign | Result/evidence |
| ---: | ---: | ---: | ---: | --- | --- |
| | | | | | |
| | | | | | |
| | | | | | |

## 6. Initial stop and reconnect checks

Run these as short, controlled trials on the same guarded one-motor fixture.
Use a logic trace or timestamped measurement; visual judgment alone is not
sufficient for the 200 ms bound. PWM and bus servos must be made mechanically
safe because they intentionally hold their last state on communication loss.

| Trial | Last valid command or fault time | Motor zero time | Interval | Recovery/session result | No stale replay | Pass |
| --- | --- | --- | ---: | --- | --- | --- |
| Explicit zero command | | | | Next control update | N/A | [ ] |
| Stop publishing; lease expiry | | | | Motor stops in 198--200 ms | Yes | [ ] |
| Kill and restart Agent | | | | Stops within 200 ms; endpoints return within 5 s | Yes; fresh command required | [ ] |
| Disconnect/reconnect data USB | | | | Stops within 200 ms; endpoints return within 5 s | Yes; fresh command required | [ ] |

- [ ] A fresh session never applies the previous session's motor command.
- [ ] PWM and bus-servo state holds as specified; no implicit bus-servo frame is
  transmitted.
- [ ] Diagnostics, reset count, transport errors, and session IDs are archived
  for every trial.
- [ ] Commissioning is complete; rebuilt, flashed, and verified the normal
  motor-locked image.

## First-day result and closeout

| Decision | Result |
| --- | --- |
| Locked-image board baseline | `PASS / FAIL / BLOCKED` |
| Guarded one-motor commissioning | `PASS / FAIL / NOT RUN` |
| Release-qualified firmware | **NO -- requires the separate qualification campaign below** |
| Open anomalies/blockers | |
| Next action, owner, reviewer | |

- [ ] Final installed locked-image digest is recorded.
- [ ] Motor power is disconnected or the board is left in a documented safe
  fixture state.
- [ ] Logs, photographs, scope/logic traces, debugger captures, configuration,
  and both artifact digests are archived under the evidence ID above.
- [ ] A final read-only diagnostic bundle was captured without qualification,
  its archive digest passes, and both files are stored under the evidence ID:

  ```sh
  readonly CLOSEOUT_EVIDENCE="${PWD}/evidence/closeout-$(date -u +%Y%m%dT%H%M%SZ)"
  sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/capture_board_diagnostics \
    --output "${CLOSEOUT_EVIDENCE}"
  (cd "$(dirname "${CLOSEOUT_EVIDENCE}")" && \
    sha256sum --check "$(basename "${CLOSEOUT_EVIDENCE}.tar.gz.sha256")")
  ```

For an anomaly, remove motor power and secure the fixture before capturing, but
do not reboot first when preserving the live session/reset evidence is safe.
Share the reviewed `.tar.gz` and `.sha256` with the completed checklist and
physical traces; `SUMMARY.txt` and `command-status.tsv` make partial capture
failures explicit.

## Separate release-qualification campaign

Do not extend this first-day checklist into an informal soak. Release evidence
is run and archived separately under the exact
[verification plan](framework/verification.md) and
[CI/hardware gates](ci-and-hardware-gates.md). It includes, among other cases:

- every retained hardware feature and every motor channel/model across the
  reviewed operating range;
- the 500 Hz, 60-minute mixed-traffic stress test, including p99 command age,
  traffic, allocation, stack, and memory gates;
- 100 USB disconnect/reconnect, 100 Agent restart, and 100 MCU reset cycles;
- the 24-hour production-rate soak; and
- malformed input, overload, UART/I2C/bus faults, watchdog stalls, and resource
  headroom measurements.

Passing Sections 1--6 authorizes only the next reviewed bring-up step. It is
not a release claim and does not qualify provisional PID, polarity, or
full-range motor behavior.
