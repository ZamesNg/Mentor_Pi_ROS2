# Firmware stabilization log

> **Historical evidence only.** This file freezes the first-board investigation
> and its retired artifact/mode names. Its `LOCKED`, `COMMISSIONING`,
> `DIRECTION_CHECK`, old tutorial numbers, and prepared-test instructions are
> not active build or operating procedures. The current repository builds only
> `NORMAL_CLOSED_LOOP_DEFAULT`; use the ordered
> [`host/`](tutorials/host/01-prerequisites-and-safety.md) or
> [`onboard/`](tutorials/onboard/01-prerequisites-and-safety.md) track.

This record separates observed symptoms, confirmed causes, temporary
workarounds, permanent source corrections, and physical verification from the
first RRCLite v2 board bring-up. It does not replace the normative contracts in
[`docs/framework/`](framework/README.md) or the numbered hardware tutorials.

Status meanings:

- **Observed**: reproduced on hardware, but the cause was not yet proven.
- **Confirmed**: source inspection, trace, or a controlled experiment proved
  the cause.
- **Temporary**: useful for diagnosis but not acceptable in the final source.
- **Corrected**: the permanent source correction passed focused software
  verification.
- **Hardware verified**: the corrected image passed the stated physical check.

No individual row and no software-only result implies release qualification.

## Hardware checkpoint

The last image tested on the board is the identity-transform locked candidate,
ELF
SHA-256
`de9e8e18611cca780cbb55903f781a906c9437e183c317f9012b1d0f43476168`.
On 2026-08-07 it produced valid six-face QMI8658 samples and established PCB
X = sensor Y, PCB Y = -sensor X, and PCB Z = sensor Z. The same procedure
mapped front-left/M1, front-right/M3, rear-left/M2, and rear-right/M4 with motor
power disconnected. Evidence is in
`build/diagnostics/characterization-20260807T114316Z/`.

The prior candidate `7051e769...` established the complete graph, supervisor
`READY`, correct heartbeat and telemetry scheduling, stable transport, and
zero post-seal allocation faults. The current result adds valid QMI startup and
the measured axis transform; it does not claim positive-axis rotation,
peripheral HIL, precision calibration, or powered motion.

## Prepared but not hardware-tested image

The current locked candidate was built from firmware source SHA-256
`a60acfd41d650587f9c7d9f2ff2d997d1fe404a8f54f136a5a82a589e6845c99`
and pinned micro-ROS archive SHA-256
`adf79c2079203740f74a87b9d5fb2fec99198c9c039897838e7aad474841f6be`.

| Milestone | Mode | ELF SHA-256 | State |
| --- | --- | --- | --- |
| `firmware/mentor_pi_mcu/build/stm32/` | `LOCKED` | `2bd7fa3e0da06d293b9d72cadcad7ad4fc2bc5735cd42fc4ad99573710d99864` | Provenance verified with the measured wheel/IMU profile and firmware-owned RGB2 status; hardware flash pending. |
| `build/firmware-milestones/05-stabilized-full-commissioning/` | `COMMISSIONING`, non-release | `36e32fd502d420b3f21ab137dba8fbef8956a383a34e8d87621ed9da84499126` | Superseded by the latest source. Rebuild only after the locked candidate passes Tutorials 04--05. |

The recorded commissioning image does not contain the latest QMI correction.
Its only purpose is historical preparation evidence; do not flash it. A new
commissioning image will be built only after the physical commissioning gate.

### Focused software verification

For the latest correction on 2026-08-07, the locked STM32 target built at
149,956 bytes text, 3,608 bytes data, and 149,072 bytes BSS and passed artifact
provenance verification. The matching Humble host compiled, passed all 1,565
executed tests with zero failures, and passed relocation verification. The
focused first-board monitor test, 152-file C++ formatting gate, tutorial helper
contract, documentation validation, shell syntax checks, and `git diff
--check` also pass.

Long fuzz, stress, recovery campaign, soak, and physical validation of this
new image remain pending.

## Failure and correction record

| ID | Status | Symptom and evidence | Confirmed cause | Permanent correction / current state | Regression protection and remaining work |
| --- | --- | --- | --- | --- | --- |
| HOST-SERIAL-001 | Corrected; hardware observed | `make flash` rejected the CH9102F stable path as unreadable/unwritable. | The login user lacked the project serial group and udev alias. | The Agent-owned `micro_ros_agent/tools/configure_serial_access.sh` creates `/dev/mentor_pi_mcu`, configures `mentor-pi-serial`, and adds the selected caller; the root tutorial façade requests interactive `newgrp mentor-pi-serial` guidance. The non-login service user uses the same policy without that interactive step. | Helper tests and `make doctor` validate identity, alias, ownership, and permissions. |
| FLASH-BOOT-001 | Automatic control implemented; hardware flash pending | CubeProgrammer opened the port but reported `GETID command not acknowledged`, `NACK`, or activation timeout. | The STM32 was not in the factory ROM bootloader; the schematic confirms that CH9102F DTR/RTS drive the board's NRST/BOOT0 transistor circuit. | `make flash-locked` uses separate modem ioctls to enter the ROM bootloader, preserves 115200/8E1 programming and read-back verification, then resets only a verified image into normal boot. Manual BOOT/RST remains a pre-programming fallback. | Mocked sequence tests prove the exact ioctl order and failure behavior. Hardware must prove automatic entry, read-back, normal reset, and the fallback before this is labelled hardware verified. |
| USB-MODEM-001 | Corrected in host source; hardware recovery test pending | First opening the CH9102F port could turn LED3 off, pulse the buzzer, or leave the MCU needing manual RST. | DTR/RTS transitions interact with the board's automatic reset/download circuit; a combined modem-control change was ineffective on this adapter. | The pinned Agent and host helpers issue separate modem-control operations that leave DTR and RTS deasserted. | The final locked image must create the controller without BOOT manipulation and pass one Agent stop/start recovery without manual reset. |
| UART-INTEGRITY-001 | Hardware verified diagnostic; temporary code removed | The isolated motor-locked diagnostic received and echoed two exact 64-byte vectors at 1,000,000 baud using polling and circular DMA. Evidence remains under ignored `build/diagnostics/uart/20260806T181104Z-69a55fde6bffcf3b-3139837/result.json`. | PA9/PA10, CH9102F, 1,000,000-baud/8N1 byte integrity, and basic DMA reception were functional. | The temporary diagnostic firmware, host CLI, trace reader, Make targets, packaging hooks, and tests were removed after the production HAL DMA transport compiled and passed its focused contract. | Preserve the ignored JSON. The final production transport still requires the passive hardware test. |
| LIBC-RAND-001 | Hardware verified correction | Firmware repeatedly reset at `RCL_OPTIONS_INIT_START`; disassembly showed pinned RMW initialization entering newlib `rand()` and attempting a 24-byte `malloc()`. | Newlib lazily allocated PRNG state while general libc allocation is intentionally unavailable. | Firmware supplies deterministic allocation-free embedded `srand()`/`rand()`; temporary client-key diagnostic encoding was removed. | The hardware baseline reached a complete graph using this correction. The source audit continues to prohibit newlib and FreeRTOS heap calls. |
| ARENA-001 | Corrected; hardware test pending | The original 48 KiB monotonic arena failed during entity construction. Increasing it and reusing only the last allocation did not reliably solve construction. | micro-ROS construction frees and reallocates non-LIFO temporary blocks, which a bump allocator cannot reclaim. | A first-party fixed-storage allocator splits, reclaims, coalesces, reallocates, and resets blocks within the 48 KiB CCM arena. | Tests cover reclamation, coalescing, in-place growth, preserved data, zero allocation, overflow, and reset/recreate. Hardware must prove graph creation and one destroy/recreate cycle. |
| ARENA-002 | Corrected; hardware test pending | A temporary 60 KiB heap created the graph but used 63,488 CCM bytes including the motor stack. | It reclaimed memory but exceeded the repository's 80% CCM gate and did not enforce the active-state allocation seal. | The heap was removed, FreeRTOS dynamic allocation was disabled, attempted allocation while active latches a diagnostic, and the 52,428-byte CCM linker gate was restored. The final builds use 51,200 CCM bytes including the 48 KiB arena and 2 KiB motor stack. | Require zero `post_seal_allocation_attempts` and allocator/pool baseline recovery after Agent restart. |
| RMW-SERVICE-001 | Corrected; hardware test pending | After graph creation the Agent deleted and recreated the client; idle manual service polling returned `RCL_RET_ERROR`. | Pinned `rmw_microxrcedds` returned `RMW_RET_ERROR` when no request buffer was present instead of OK with `taken=false`. | A reproducible patch is applied only after verifying the exact locked upstream tree. Empty service queues return OK with `taken=false`; real deserialization errors remain errors. The application request-ID workaround was removed. | The regenerated archive/tree hashes are pinned. Hardware must prove stable idle polling and successful real service replies. |
| TRANSPORT-001 | Corrected; hardware test pending | A temporary polling-RX/blocking-TX transport created stable sessions, while the earlier bespoke above-FreeRTOS-priority DMA handler did not. | The custom interrupt path was unsafe, and the polling prototype masked DMA laps and UART/DMA errors. | USART1 now uses standard priority-6 HAL UART/DMA handlers. RX uses an 8 KiB circular DMA buffer with half/full epochs and stable NDTR sampling; TX uses an SRAM bounce buffer, HAL DMA state/callbacks, a baud-derived deadline, and bounded abort-on-error behavior. Fatal errors disarm motors. | Cursor, boundary, wrap, overrun, timeout, and source-contract tests pass; locked target build and artifact verification pass. Milestone 03 SHA is `8d25a0981f3e1003d483d92dd0ba0dd11e0664d9f8837aa78fcde5223c3eef95`. Physical session and traffic checks remain. |
| AGENT-PING-001 | Corrected; hardware test pending | During diagnosis, entity creation was allowed when bytes arrived even if `rmw_uros_ping_agent()` rejected the reply. | This was only an isolation shortcut around the then-broken transport. | Strict `rmw_uros_ping_agent() == RMW_RET_OK` validation is restored; byte traffic alone cannot mark the Agent available. | Final locked hardware must create the graph and recover after one Agent restart with strict validation active. |
| BATTERY-BUZZER-001 | Corrected; calibration pending | USB-only power produced repeated automatic low-battery buzzer patterns because the estimate was below the 6,300 mV alarm threshold. | With no battery-present GPIO, an absent battery was treated as a valid low battery. | Readings below 4,900 mV are invalid/absent and do not update the filter or alarm state. Valid readings at or above 4,900 mV again use the documented debounce, hysteresis, and automatic alarm. Explicit buzzer commands remain available. | Domain boundary tests pass at 4,899/4,900 mV. Divider/VREFINT scaling and alarm timing still require Tutorial 05 hardware measurements. |
| TIMEBASE-001 | Hardware verified | Heartbeat was approximately 10.15 Hz instead of 2 Hz and IMU was also fast. | The first 16 MHz `HAL_InitTick(15)` call did not save `uwTickPrio`; clock switching later retried with invalid priority 16, leaving TIM14's prescaler configured for 16 MHz while driven at 84 MHz: an exact 5.25× tick error. | `HAL_InitTick()` validates and stores priority, stops the old timer, derives the prescaler from the current clock, resets update state, and restarts TIM14. | The fixed image measured heartbeat at 1.986--1.987 Hz for 60 seconds. The remaining IMU drift was corrected separately. |
| TELEMETRY-SCHEDULE-001 | Hardware verified | After fixing TIM14, heartbeat passed but IMU measured about 47.17 Hz, below the 47.5 Hz limit. | Assigning `last_publish_ms = now_ms` accumulated normal executor latency on every 20 ms period. | Periodic releases advance on their fixed timeline; missed periods are skipped without catch-up bursts. | The hardware baseline measured heartbeat 2.000 Hz and IMU 50.003 Hz over 60 seconds with a continuous session. |
| IMU-PUBLISH-001 | Hardware verified correction | Candidate `7051e769...` published an invalid, zero-stamped IMU message at 49.98 Hz even though the sensor worker had never supplied a sample. | `PublishImuState()` emitted the zero-initialized cache whenever the 20 ms publication release was due. | Firmware forwards only updates supplied by the sensor worker and does not manufacture samples from an untouched cache. | The subsequent six-face run received six valid physical QMI samples through ROS; the measured transform is tracked separately below. |
| IMU-STARTUP-001 | Hardware verified | An earlier run recorded no raw sample. After matching the preserved CTRL7-last initialization order, current diagnostics contain exactly one IMU `UNSUPPORTED(detail=1)` marker and no IMU timeout or repeated I/O error. | Source inspection proves that marker is emitted only after QMI8658 initialization, data-ready, and a complete raw accel/gyro read succeed. | The driver configures QMI8658 with accel/gyro disabled and writes CTRL7 enable last. | The successful raw path and subsequent six-face samples are hardware evidence. |
| IMU-ROS-CHAR-001 | Transform measured on hardware; corrected artifact verified; flash pending | The identity-map capture measured requested PCB faces +X=(0.456,9.606,-0.725), -X=(0.208,-10.005,0.855), +Y=(-9.319,-0.104,-0.323), -Y=(10.196,-0.280,0.102), +Z=(0.248,0.159,9.693), and -Z=(0.369,-0.861,-9.785) m/s². Evidence is `build/diagnostics/characterization-20260807T114316Z/imu-six-face.tsv`. | The QMI8658 sensor frame is rotated relative to the documented PCB frame: PCB X = sensor Y, PCB Y = -sensor X, and PCB Z = sensor Z. | The STM32 composition root applies `{{Y,+}, {X,-}, {Z,+}}` to acceleration and angular velocity. Locked ELF `2bd7fa3e0da06d293b9d72cadcad7ad4fc2bc5735cd42fc4ad99573710d99864` passed provenance verification. | Flash the locked image and repeat Tutorial 05. All six gravity rows must pass; positive rotations remain a release check. |
| BOARD-PROFILE-001 | Encoder mapping and M1 powered direction hardware verified | Passive one-wheel-at-a-time capture established `front-left=M1`, `front-right=M3`, `rear-left=M2`, and `rear-right=M4`. Raw forward deltas were M1 `+742`, M2 `+1915`, M3 `-1370`, and M4 `-1612`. A 100 ms M1 run ended at only `-1` tick and failed closed; the repeat at `+0.25 RPS` for 2,000 ms produced `+294` ticks, peak magnitude 0.452010 RPS, confirmed zero, and passed. | Connector ownership follows the confirmed chassis layout, while right-side encoder wiring has the opposite raw sign. The isolated one-tick short-window result was noise/settling, not reliable polarity evidence; the longer coherent response confirms M1's current sign. | The STM32 composition root retains the existing JGA27 model method and channel wiring signs `{+1,+1,-1,-1}` in M1--M4 order. No bridge inversion was applied. The maintained profile is `docs/framework/verified-hardware-profile.md`. | Keep M1 unchanged. Use an observable bounded duration for M2--M4 and require each channel's positive delta, response, and final-zero proof before recording it verified. |
| MOTOR-DIRECTION-MODE-001 | Software correction prepared; hardware flash pending | The first powered stage needs only encoder/physical direction, but the prior commissioning image ran provisional PID. Its 100 ms M1 attempt also treated one reverse tick as conclusive wrong-direction evidence. | PID behavior and a one-tick filtered velocity transient are unnecessary confounders for polarity verification. | Commissioning now uses an explicit `DIRECTION_CHECK` mode: fixed 250-permille output from command sign, no PID execution, a firmware 0.50 RPS overspeed disarm, host overspeed/lease/session guards, a two-tick direction threshold, and an operator physical-direction confirmation. | Build and provenance-verify the new commissioning image, then test each motor in both signs with raised wheels and a current-limited supply. Do not describe RPM control as verified. |
| RGB-STATUS-001 | Corrected; hardware observation pending | RGB2 previously shared the ROS-controlled two-pixel command and could not provide trustworthy MCU state. | A host-commanded status pixel can overwrite or imitate firmware state. | LED3 is firmware-owned and toggles only after a `/mentor_pi/heartbeat` publish succeeds. RGB2 is firmware-owned: red and green pulse for 50 ms when the 10 Hz sampler observes RX and TX byte progress; blue remains off. ROS LED ID 3 and RGB masks 2/3 are rejected, while LED1/LED2 and RGB1 remain host-controlled. | Unit and controller integration tests cover heartbeat gating, sampling, pulse expiry, simultaneous yellow, transport closure, and timer wrap. Observe both indicators after flashing; this is not yet hardware verified. |
| PASSIVE-BATTERY-001 | Corrected in host source; hardware rerun pending | USB-only characterization reported `voltage_mv=0`, `valid=false`, `below_threshold=false`; the monitor set the battery invalid-data bit. | Characterization reused the strict preflight battery rule even though the fixture intentionally has no battery. | Characterization accepts only that exact fail-closed absent state. Strict qualification still requires a valid, physically calibrated battery sample. | Boundary tests reject adjacent invalid states. Rerun with USB-only power. |
| PASSIVE-HISTORY-001 | Procedure corrected; OLED issue remains open | Running passive capture after peripheral smoke inherited cumulative bus-servo/OLED errors, including OLED IO errors, from the same session. | Diagnostics counters are intentionally monotonic until MCU reset; passive capture incorrectly appeared to be an isolated test. | Tutorial 04 now requires passive capture immediately after a fresh runtime reset and before peripheral smoke. | A clean run distinguishes stale history from a reproducible OLED or bus-servo fault. Any new peripheral fault still fails. |
| PASSIVE-OLED-001 | Hardware configuration recorded; OLED verification blocked | The clean `passive-20260807T102727Z` run recorded one expected IMU marker and 242 OLED initialization I/O errors because this board currently has no OLED installed. | The firmware correctly retries its required SSD1306, but the first-board helper had no way to distinguish declared absent hardware from an unexpected display failure. | `make passive-check` now asks whether the OLED is installed. The development characterization exception accepts only repeated OLED initialization NACKs with no timeout or other fault, prints progress every five seconds, and labels the result `OLED NOT INSTALLED/NOT TESTED`. | Strict preflight, peripheral HIL, and release qualification retain the OLED requirement. Install the display and pass Tutorial 07 before making an OLED or release claim. |
| PASSIVE-SETTLING-001 | Corrected in host source; hardware rerun pending | `passive-20260807T104800Z` passed every rate, graph, transport, battery, and OLED-adjusted diagnostic check but reported only failure bit `0x00200000`. The pre-qualification sample was session 1 while the monitor ran entirely in session 2. | During the first diagnostic publications after reconnect, the generic `last_error_*` tuple can still identify the preceding Agent teardown. The monitor validated that transient tuple before its existing five-second discovery period ended and permanently latched an IMU-characterization mismatch, although the captured steady-state tuple and counters were exact. | Characterization diagnostics may settle during the discovery interval. A mismatch after that interval still fails immediately, and completion always requires the latest exact diagnostic state. The terminal result is now a multiline stream table with separate traffic, motor-evidence, failure, and stream-mask sections. | A focused regression reproduces the stale teardown tuple and proves that a persistent mismatch still fails. The full Humble host suite and relocation verification passed. Rerun `make passive-check`; no firmware flash is required. |

## Prepared test order (retired historical sequence; do not execute)

When returning to the board, do not start with the commissioning image.

1. Keep every actuator disconnected and flash the current locked image:

   ```sh
   cd /home/zames/Mentor_Pi && make flash-locked
   ```

   Require ELF SHA-256
   `2bd7fa3e0da06d293b9d72cadcad7ad4fc2bc5735cd42fc4ad99573710d99864`
   and successful programming/read-back verification.

2. Start a fresh runtime, then run `make passive-check` before any peripheral
   smoke test. Require the complete graph, supervisor `READY`, heartbeat 2 Hz,
   valid IMU at 50 Hz, exact absent-battery state, zero post-seal allocations,
   and no new transport, reset, reconnect, or peripheral fault.
   Stop here if any check fails.

3. Complete passive characterization in Tutorial 05. Only with raised or
   equivalently guarded wheels, a current-limited supply, and a reachable
   physical stop, follow Tutorial 06 to rebuild/verify the commissioning
   image from the then-current source.

4. Commission one motor channel at a time with the bounded C++ utility. Remove
   motor power and restore/read back the locked image immediately afterward.

The earlier milestone directories 01--03 are diagnosis/rollback evidence, not
the recommended test sequence for the completed source.

## Deferred qualification

The 60-minute command campaign, repeated recovery campaigns, 24-hour soak,
and all unmeasured physical calibration gates remain deferred. The final
locked and commissioning images are prepared test candidates, not release-
qualified firmware.
