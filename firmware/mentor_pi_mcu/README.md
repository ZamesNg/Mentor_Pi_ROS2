# Mentor Pi MCU portable domain

This directory contains the hardware-independent C++17 control and validation
layer for RRCLite v2. Code under `include/mentor_pi_mcu/domain` and `src` has no
dependency on STM32 HAL, CMSIS, FreeRTOS, micro-ROS, or generated ROS headers.
It performs no dynamic allocation.

The STM32 application adapts ROS messages into these fixed structs, publishes
them through the bounded mailboxes, and applies returned hardware values from
the single owning task. The adapter remains responsible for immediate
register-level emergency motor shutdown; `MotorController::DisarmAll()` is the
matching software-state transition.

The motor sign contract is deliberately simple: MCU encoder state is the raw
signed counter delta for every motor model, and bridge duty is the signed LADRC
output without another transform. The host owns the only chassis-direction
conversion between this MCU coordinate and positive ROS wheel rotation. ADRC
values and unverified physical channel mappings remain release-provisional
until guarded HIL records them.

Closed-loop control uses first-order linear ADRC at 100 Hz. The extended-state
observer estimates motor speed and the combined disturbance from filtered
encoder velocity and the previously applied output. The controller then uses
the speed error and disturbance estimate to produce signed permille output.
Output is bounded to 1000 permille. The minimum-drive floor is currently zero,
so small nonzero ADRC outputs are no longer raised to a fixed duty. Every model
defaults to input gain `b0=0.03
RPS/s/permille`, controller bandwidth `wc=4 rad/s`, observer bandwidth `wo=12
rad/s`, and velocity-filter new-sample weight `0.5`. These are provisional
starting values and require guarded physical tuning.

## Motor safety configuration

The only supported firmware build is the closed-loop ADRC image
(`control_mode=CLOSED_LOOP`, 6 RPS ceiling, ±1000 permille output,
`release_qualified=0` pending HIL). It accepts all validated zero-speed commands and
atomically rejects invalid or out-of-range commands without refreshing leases.
An invalid build-time controller configuration rejects otherwise-valid nonzero
commands as `UNSUPPORTED`. Valid nonzero commands are accepted
and applied through closed-loop ADRC control, bounded by the model-specific
RPS profile, the 6 RPS implementation ceiling, and the ±1000 permille
output limit. Passive encoder telemetry remains active whenever the MCU is
powered.

Build and inspect the ADRC release profile from the repository root:

```sh
make -C firmware setup
make -C firmware test
MENTOR_PI_NAME=mentor_pi_1 make -C firmware build
make -C firmware verify
```

`MENTOR_PI_NAME` is a required relative ROS namespace. It is compiled into the
micro-ROS node so publishers, subscriptions, and services share the selected
robot namespace. Changing it requires a new verified firmware build and flash;
`make onboard-configure` performs that bounded reconfiguration workflow.

Maintainers regenerate the checked Humble SDK after changing
`ros2_ws/src/mentor_pi_interfaces` with:

```sh
make -C firmware microros-sdk
```

Run this command as the normal Ubuntu or Dev Container user, never with
`sudo`. The Dev Container image provides that user's rosdep cache. Rebuild and
reopen the Dev Container if the generator reports that the cache is
unavailable.

The verifier must report `motor_mode=ADRC`, `artifact_mode=NORMAL`,
`control_mode=CLOSED_LOOP`, `release_qualified=0`, valid provenance, and at
least 20% headroom in every
memory class. The configuration supervisor gate, model-specific RPS limits,
independent 198 ms per-motor leases, session-loss disarming, and
transport-failure shutdown apply to every accepted command.

The encoder, controller-output, and ROS wheel sign contract is recorded in
[`verified-hardware-profile.md`](../../docs/framework/verified-hardware-profile.md).
Before
any powered motor work, complete Tutorials 01--05 passively with actuator
power disconnected, confirm wheel clearance, use a current-limited supply,
keep a physical motor-power stop reachable, follow the guarded checkout in
[host Tutorial 07](../../docs/tutorials/host/07-connect-and-run.md) or
[onboard Tutorial 07](../../docs/tutorials/onboard/07-integrated-runtime-and-recovery.md),
and record the required evidence through
[onboard Tutorial 08](../../docs/tutorials/onboard/08-evidence-and-qualification.md).
D3 HIL must qualify or replace every ADRC, filter, minimum-drive floor,
motor/channel polarity, and full-range behavior before production
nonzero-motion claims.

Every successful supported build also writes
`build/stm32/rrclite-build-metadata.txt`. The direct CubeProgrammer flash
wrapper verifies that metadata, the pinned generated micro-ROS header/archive
tree, the current project source fingerprint, the selected motor profile, and
the ELF hash before creating an immutable upload snapshot. Flashing an
unverified artifact and debugger-initiated firmware loads are intentionally
unsupported. Source-level debugging requires a separate SWD probe; the
CH9102F/USART1 ROM-bootloader path provides flashing only.

On Linux, `make -C firmware flash` builds the host-side CH9102F helper through
CMake/Ninja and uses separate RTS/DTR set/clear ioctls. It asserts reset with
BOOT0 high, enters the ROM bootloader, probes it, programs and verifies the
immutable snapshot, and only then resets with BOOT0 low into the application.
An ioctl, preflight, programming, or verification failure never performs the
final application reset. `AUTOMATIC_BOOT_CONTROL=0` is reserved for the
documented physical BOOT/RST fallback.

## Traceability

| Domain area | Requirements / audit | Native evidence |
|---|---|---|
| Validation and bounded text | `ROS-004`, `SAFE-001`, systemic defect 1 | `TestValidationAndStateMerging` |
| Latest mailboxes and button FIFO | `RT-003`, `RT-005`, `HW-MEM-01` | `TestFixedContainers`, `TestCommandMailboxes` |
| Motor profiles, ADRC, and leases | `CTRL-001`–`CTRL-004`, `ROS-MOT-01`, `HW-MOT-01` | `TestMotorController`; HIL still required |
| PWM interpolation and offsets | `CTRL-005`, `CTRL-006`, `ROS-PWM-01` | `TestPwmServoController` |
| Bus framing and stop watermark | `CTRL-007`–`CTRL-009`, `ROS-BUS-01` | `TestBusServoCodecAndScheduler` |
| LED and buzzer patterns | `CTRL-010`, `HW-LED-01`, `HW-BUZ-01` | `TestPatterns` |
| Button event state machine | `CTRL-013`, `ROS-BTN-01` | `TestButtons` |
| Battery filter/alarm | `CTRL-014`, `ROS-BAT-01`, `HW-BAT-01` | `TestBatteryMonitor` |

These tests provide the portable portions of `VER-UNIT-VAL-001`,
`VER-UNIT-MBOX-001`, `VER-UNIT-MOTOR-GATE-001`, `VER-UNIT-LEASE-001`,
`VER-UNIT-BTN-001`, and `VER-UNIT-DIAG-001`. They do not replace the required
STM32 integration and HIL cases.

Build and run the native tests:

```sh
cmake -S firmware/mentor_pi_mcu -B build/mentor_pi_mcu-domain
cmake --build build/mentor_pi_mcu-domain
ctest --test-dir build/mentor_pi_mcu-domain --output-on-failure
```

## Bounded fuzz smoke

Clang/libFuzzer builds seven ASan/UBSan harnesses covering exact validation
results for every v2 command and service, topic mailbox atomicity and session
reset, controller/service rejection state, bus stop ordering, bus frame codec,
the UART5 bus-servo driver behind a fixed-memory fake HAL, and the production
USART1 circular-RX bookkeeping/copy primitive. The driver harness asserts that
invalid requests never start an exchange; the RX harness compares exact,
wrapped, full, overrun, inconsistent-sample, and position-wrap behavior against
a fixed-array reference model.

The deterministic target copies the checked-in seed corpus into the build tree
before execution, so generated mutations do not modify source files. Run the
direct commands below on native Ubuntu 22.04 with Clang 18, or inside the VS
Code Dev Container. Dev Container fuzz results remain development evidence.

The previously recorded 2026-08-06 long-run observation used the disposable
single-report layout and was later overwritten by a smoke run. Its immutable
replacement, `prehardware-20260806-fuzz-qualification-v3`, ran 1,666,667 inputs
in each of seven harnesses, 11,666,669 total, with a passing semantic preflight
and ASan/UBSan campaign. Both records predate the Humble-only dependency
conversion and expansion of the authoritative source manifest, so they are
historical evidence only and do not qualify this revision. The retained
package bound production fingerprint
`d68c056faf1dd0ce4a83c32caea0f55b57e85579fbaab53f99852889ca410499`
to test-input digest
`6c8b4ff9bdf5dd2c4f34ffb53e3c77b5231f5cac4a827fb9e110bf3395bb04c2`
and final-corpus digest
`01af9fe2f3a65852e210791c695ea1d58b6999c97522b5c722a69c89dc134e89`.
Native macOS sanitizer runtimes are not accepted as qualification evidence.

The Ubuntu/Dev Container smoke commands are:

```sh
cmake -S firmware/mentor_pi_mcu -B build/fuzz-smoke -G Ninja \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_COMPILER=clang++-18 \
  -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_ENABLE_FUZZING=ON \
  -DMENTOR_PI_MCU_FUZZ_SMOKE_RUNS=10000
cmake --build build/fuzz-smoke --target mentor_pi_mcu_fuzz_smoke
```

Before any libFuzzer harness runs, that target executes
`mentor_pi_mcu_validation_semantic_contract_tests`. The same executable is a
native CTest. It sends 282 fixed, bounded byte inputs through the unchanged
validation-oracle fuzz entry point and fails unless its explicit ledger covers
all 13 v2 command/service input types, maximum bus counts, every malformed
mask-bearing type, malformed OLED length/termination, duplicate bus IDs, all
256 motor-model bytes, selected NaN/Inf values, and signed/unsigned boundaries.
These are deterministic preflight inputs; they do not assert that the
checked-in mutation seeds individually encode every case and they do not change
the checked-in corpus. The preflight establishes the deterministic
semantic-oracle matrix only; state preservation, lease behavior, and the
no-hardware-call clauses require the corresponding unit/fuzz harness evidence.

The default/direct invocation is a 10,000-input smoke run per harness, not a
qualification campaign. The current harnesses assert rejection-state
preservation for all topic mailboxes, RGB/OLED mergers, and their listed
controller state paths. The bus UART driver also asserts that an invalid move,
stop, query, or configure request starts no fake-HAL exchange. The 10-million
pre-Humble campaign completed the historical input-count and sanitizer
portion, but the Humble revision requires a new retained campaign;
comprehensive no-invalid-hardware-call evidence is also still required for
`VER-FUZZ-VAL-001`. USART1 input remains opaque to these seven harnesses, so
they do not complete `VER-FUZZ-TRN-001`; no handwritten XRCE parser is used as
substitute evidence.

## Pinned XRCE framing conformance

A separate deterministic CTest target downloads the source archive for the
exact Micro-XRCE-DDS Client commit in `config/microros_sources.lock`, verifies
its checked-in SHA-256, and compiles its official custom-transport and
stream-framing sources with the firmware's 512-byte MTU. Run it with Clang 18:

```sh
cmake -S firmware/mentor_pi_mcu/tests/xrce_conformance \
  -B build/xrce-conformance -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_C_COMPILER=clang-18 \
  -DCMAKE_CXX_COMPILER=clang++-18 \
  -DMENTOR_PI_XRCE_CONFORMANCE_SANITIZERS=ON
cmake --build build/xrce-conformance
ctest --test-dir build/xrce-conformance --output-on-failure
```

The fixed-memory test creates known-good frames through the client's public
custom-transport send path and receives them across one-byte and bounded split
patterns, including reserved-octet stuffing. It also runs a seven-case bounded
table for missing CRC octets, bad CRCs, duplicated truncations, continuous and
one-byte delivery, and conditional recovery. Bad CRCs reject and recover on the
next frame. With pinned client 2.4.2, a missing final CRC octet rejects but
consumes the immediately following frame. For the exact tested schedules, the
next two continuously supplied frames are returned; a separate case proves
recovery when a later delimiter is followed by one zero-byte transport read
before that frame's remaining bytes arrive. Neither schedule is claimed as a
general parser recovery bound. The test asserts that no malformed payload is
returned and that all read/receive loops stay within fixed budgets.

This is deterministic framing/recovery conformance only. It is not an eighth
fuzz harness and does not complete the arbitrary-input portion of
`VER-FUZZ-TRN-001`.
