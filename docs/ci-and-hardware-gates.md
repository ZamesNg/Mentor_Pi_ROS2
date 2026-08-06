# CI and Hardware Qualification Gates

This guide separates checks that can run on hosted amd64 or arm64 runners from
evidence that requires the physical RRCLite V1.0 and a guarded fixture.
Passing hosted CI does not by itself qualify a release or enable nonzero motor
authority.

## Hosted workflows

| Workflow | Hosted evidence | Hardware | Configured secrets |
|---|---|---|---|
| `software-quality.yml` | Documentation links and traceability, Google-format check, Debug ASan/UBSan tests, optimized Release tests, TSan concurrency tests, enforced 90%/80% portable coverage, deterministic libFuzzer smoke, exact pinned XRCE framing conformance, and `clang-tidy` | None | None |
| `ros-humble-amd64.yml` | Authoritative `make host` Release build, tests, offline handoff packaging, relocation, Humble metadata/manifest verification, and a network-separated pinned native-Agent build with checksummed evidence in a native Ubuntu 22.04 matrix on amd64 and arm64 | None | None |
| `firmware-reproducibility.yml` | Pinned dependencies, reviewed micro-ROS archive, linker resource assertions, two clean motor-locked builds with identical loadable bytes, a separate motor-locked STM32 Debug build, required interrupt/fault/RTOS vector-strength audit, and `clang-tidy` over its complete first-party Arm compile database | None | None |

All external actions are pinned to immutable commit SHAs. Workflow permissions
are limited to read-only repository contents; artifact upload uses GitHub's
short-lived workflow identity. Fork pull requests receive no project secret.

The XRCE conformance, ROS, and firmware jobs require public network access. The
conformance job fetches the Micro-XRCE-DDS Client archive for the exact commit
in `microros_sources.lock` and verifies its checked-in SHA-256 before compiling
it. ROS packages come from the configured ROS 2 Humble apt repository. Firmware
dependencies are fetched at the exact commits in the build scripts, and all
three firmware Dockerfiles pin their base image by digest. These are dependency
inputs, not runtime services.

## Run the software-only gates locally

The documentation checker uses only the Python standard library and is a
build-time tool, never a runtime node or bridge:

```sh
./tools/check_framework_docs.py
```

Run native Debug tests with ASan/UBSan, optimized Release tests, and the separate
TSan concurrency job:

```sh
./tools/run_native_ci_tests.sh --build-type Debug --sanitizers on
./tools/run_native_ci_tests.sh --build-type Release --sanitizers off
./tools/run_tsan_tests.sh
```

Generate the explicit first-party portable coverage report with LLVM coverage
tools:

```sh
./tools/run_coverage_tests.sh
```

This command and the hosted portable-coverage job fail below 90% line or 80%
branch coverage. The current collision-free baseline is 91.29% line and 80.94%
branch coverage across the 63-file manifest written beside the report under
`build/coverage/reports/`.

The recommended local command runs the deterministic smoke target in the
pinned Linux/Clang 18 firmware-builder container:

```sh
./tools/run_fuzz_smoke.sh
```

This is the supported path from the clean Ubuntu 24.04 Docker development
host. Native macOS sanitizer runtimes are not qualification environments. The
runner validates a bounded per-harness count,
uses `--network=none` for the test container, and treats
`build/fuzz-smoke-linux/` as disposable working storage. Only a successful run
is atomically published to a new, read-only
`build/fuzz-evidence/<run-id>/` directory. Publication rejects an existing run
ID, symbolic-link roots or targets, missing files, checksum failures, and
inconsistent report bindings. Each package includes the campaign log, a closed
`SHA256SUMS`, and separate digests for first-party production sources, test
inputs, source corpus, retained final corpus, and the content-addressed Docker
image/toolchain record. Verify a retained package with:

```sh
./tools/verify_fuzz_evidence.sh build/fuzz-evidence/<run-id>
```

Use `--run-id NAME` when an external qualification record assigns the name;
otherwise the runner generates one. Inspect the long validation plan without
starting it with
`./tools/run_fuzz_smoke.sh --runs 1666667 --print-plan`. That count executes
at least ten million cases across the six validation/state/driver harnesses,
plus the separate USART1 ring harness; it does not by itself close the full
XRCE-session-parser part of `VER-FUZZ-TRN-001`.

The previously noted 2026-08-06 11,666,669-execution observation predated this
immutable evidence format, and its single working-directory report was later
overwritten by a smoke run. The replacement campaign
`prehardware-20260806-fuzz-qualification-v3` ran 1,666,667 inputs through each
of seven harnesses under Clang 18 ASan/UBSan: 10,000,002 inputs across the six
validation/state/driver harnesses plus 1,666,667 in the USART1 ring harness,
11,666,669 total. It passed the semantic preflight and immutable verifier. Its
production fingerprint is
`d68c056faf1dd0ce4a83c32caea0f55b57e85579fbaab53f99852889ca410499`,
test-input digest is
`6c8b4ff9bdf5dd2c4f34ffb53e3c77b5231f5cac4a827fb9e110bf3395bb04c2`,
source-corpus digest is
`369692aa66ff1f1fdeee8b7c2ea672fd63864521f046b962399f369129cfb0a7`,
and 867-file final-corpus digest is
`01af9fe2f3a65852e210791c695ea1d58b6999c97522b5c722a69c89dc134e89`.
Both campaigns predate the Humble-only dependency conversion and authoritative
source-manifest expansion. They are historical evidence only and do not close
the current-revision input-count or sanitizer clause of `VER-FUZZ-VAL-001`.

State-preservation fuzz assertions cover all seven command mailboxes, the
RGB/OLED mergers, and the listed controller state paths. The bus UART driver
also proves that its rejected move, stop, query, and configure requests make no
fake-HAL exchange. Those useful partial assertions do not prove the
no-invalid-hardware-call clause for every retained hardware path. USART1 ring
bytes also remain opaque rather than being parsed by the full XRCE session
implementation.

Inside the pinned ROS-free Ubuntu 24.04 firmware-test container with Clang 18
and its libFuzzer runtime installed, the direct command used by hosted CI is:

```sh
cmake -S firmware/mentor_pi_mcu -B build/fuzz-smoke -G Ninja \
  -DBUILD_TESTING=OFF \
  -DCMAKE_CXX_COMPILER=clang++-18 \
  -DMENTOR_PI_MCU_ENABLE_SANITIZERS=OFF \
  -DMENTOR_PI_MCU_ENABLE_FUZZING=ON \
  -DMENTOR_PI_MCU_FUZZ_SMOKE_RUNS=10000
cmake --build build/fuzz-smoke --target mentor_pi_mcu_fuzz_smoke
```

The native Debug/Release CI suites and the fuzz-smoke target run
`mentor_pi_mcu_validation_semantic_contract_tests`. This bounded C++ preflight
passes 282 deterministic byte inputs through the same
`LLVMFuzzerTestOneInput` entry point as the validation-oracle fuzzer. Its
reviewable ledger must contain all 13 v2 command/service input types, both
maximum-count bus types, every mask-bearing type with a malformed mask,
malformed OLED size/termination, duplicate bus IDs, all 256 motor-model enum
bytes, selected NaN/Inf values, and signed/unsigned boundaries. It neither
modifies the checked-in source corpus nor claims those mnemonic seeds each
contain all categories. Consequently, the semantic ledger and the mutation
corpus have distinct digests in every published evidence package.

The target runs 10,000 inputs through each of seven fixed-memory harnesses:

- exact command/service validation oracles, including biased IEEE-754 values;
- RGB/OLED merge invariants;
- all seven topic mailbox admission/reset paths;
- motor, PWM, LED, buzzer, battery, motor-model, and bus-stop state paths;
- bus-servo frame codec parsing/round trips; and
- the UART5 bus-servo driver with a bounded fake HAL covering malformed replies,
  raw I/O statuses, timeout, cancellation, partial completion, and the invariant
  that a rejected request makes no hardware call; and
- the shared production USART1 circular-RX primitive against a fixed-array
  reference model, including exact/wrapped reads, a full ring, overrun,
  inconsistent DMA samples, and `uint32_t` position wrap.

The source corpus is read-only during this target. CMake copies it into
`build/fuzz-smoke/fuzz-smoke-corpus/`, and libFuzzer writes mutations only to
that disposable build-tree copy.

Build the separate deterministic XRCE framing conformance target with Clang 18:

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

This target compiles the official custom-transport and stream-framing sources
from Micro-XRCE-DDS Client commit
`83f129a80770a09aac9e823896ecbf6a0eddf0fc`. Its fixed-memory tests generate
known-good frames through the public custom-transport send path and consume
them through the matching receive path across one-byte reads, bounded read
splits, bounded write splits, and reserved-octet stuffing. A seven-case recovery
table also covers a missing final CRC octet with bulk and one-byte reads, three
continuously available valid frames after that truncation, a bad CRC with bulk
and one-byte reads, and duplicated truncated frames. Every case has a fixed
public-receive and raw-read budget and proves that the malformed payload is
never returned.

This test records an upstream limitation rather than masking it: a bad-CRC
frame is rejected and the immediately following valid frame is returned, but a
frame missing its final CRC octet consumes the immediately following valid
frame. In the exact continuous bulk and one-byte schedules tested, the next two
of three supplied frames are then returned. Separately, a zero-byte
custom-transport read immediately after a later `0x7e` delimiter is a tested
conditional recovery point; the remainder of that well-formed frame is
returned. These fixed schedules are not a general parser resynchronization
bound. Guaranteed operational recovery instead comes from three consecutive
10 ms Agent-ping failures at the 500 ms cadence followed by bounded session
teardown and recreation; motor safety remains the independent 198--200 ms
lease. This target closes deterministic framing, split, rejection, and
conditional-recovery characterization only. It is not an eighth fuzz harness
and does not satisfy the arbitrary-input campaign in `VER-FUZZ-TRN-001`.

With clang 18 installed, run format and static analysis:

```sh
CLANG_FORMAT=clang-format-18 ./tools/check_cpp_format.sh
CLANG_TIDY=clang-tidy-18 \
RUN_CLANG_TIDY=run-clang-tidy-18 \
  ./tools/run_clang_tidy.sh
```

After the pinned firmware dependencies and micro-ROS library are present, prove
that two independently cleaned builds have identical loadable bytes:

```sh
./tools/bootstrap_firmware_dependencies.sh
./tools/build_microros_library.sh
./tools/check_firmware_reproducibility.sh
./tools/run_firmware_target_ci.sh
```

The comparison script always forces `RRCLITE_MOTOR_COMMISSIONING=0`, removes
only `firmware/mentor_pi_mcu/build/stm32/` between builds, independently derives
a binary image from each ELF, compares that image and Intel HEX, and writes a
SHA-256 report under `build/firmware-reproducibility/`. The second build remains
available for inspection and upload.

The target-CI script writes only to `build/firmware-target-debug/`; it does not
replace or reclassify the locked artifacts under
`firmware/mentor_pi_mcu/build/stm32/`. It builds with CMake `Debug`, verifies
that commissioning and `NDEBUG` are both absent, compares the actual Arm
compile database with the complete production first-party source set, and runs
Clang 18 analysis with the pinned Arm GNU 13.2.1 driver and headers. After
linking, it also requires every used peripheral interrupt plus every retained
fault and FreeRTOS exception entry to appear exactly once as a strong global
text symbol in both the verified locked ELF and the independent Debug ELF; a
weak startup default cannot satisfy this gate. Pinned STM32Cube, FreeRTOS,
generated micro-ROS, and generated ROS sources remain outside the first-party
analysis set. The resulting report, analyzed-source manifest, and two
21-symbol vector manifests are uploaded with the workflow artifact. The
narrowly scoped bare-metal analyzer exceptions and their reasons are recorded
in the
[development standards](framework/development-standards.md#builds-warnings-and-automated-tooling);
the host-native analysis retains those checks.

## Gates that still require the board

The following evidence cannot be produced by GitHub-hosted runners:

- passive encoder direction, per-channel motor polarity, PID/filter/deadband,
  current, and guarded full-range motor characterization;
- QMI8658 six-face and positive-axis rotation measurements;
- battery-divider and VREFINT calibration;
- PWM, RGB, buzzer, LED, OLED, bus-servo, and button electrical/timing checks;
- watchdog timing and retained reset-cause behavior;
- USB/CH9102F/USART1 logic-analyzer measurements, physical disconnects, UART
  fault injection, and actual Agent/MCU recovery timing;
- measured flash/RAM/stack/traffic margins under production traffic;
- the 500 Hz one-hour stress run, 100-cycle reconnect/reset campaigns, and the
  24-hour soak.

Those cases require a dedicated self-hosted runner or an operator following the
[verification plan](framework/verification.md) with the board, current-limited
supply, raised-wheel fixture, emergency motor-power stop, bus-servo fixture,
measurement equipment, and uninterrupted evidence capture. No hosted workflow
pretends to close them.

## Secrets and release publication

No secret is currently needed by any checked-in workflow. Firmware artifacts
uploaded by CI are unsigned engineering outputs; the normal artifact remains
motor-locked and a commissioning artifact is never produced by hosted CI.

If release signing or external publication is added later, use a protected
GitHub environment with reviewer approval and a dedicated least-privilege
credential. That job must run only for reviewed release refs, never for pull
requests or arbitrary branch code. Hardware-runner credentials and device
access likewise belong on the protected self-hosted runner, not in repository
variables or workflow logs.

## Known software-only qualification work

The generated Humble CDR/introspection tests and seven bounded libFuzzer
harnesses are checked in. Hosted CI runs the generated type-support suite, a
deterministic 10,000-input smoke run per fuzz harness (70,000 total executions),
and the separate pinned-source XRCE framing conformance target. Smoke proves
that the harnesses build and execute under ASan/UBSan; it is not the
qualification campaign.

The deterministic semantic validation-input preflight and retained
source-corpus review are complete. The immutable runner preserves each
successful campaign and binds it to production-source, test-input, corpus, and
toolchain digests. The retained 11,666,669-execution campaign belongs to the
pre-Humble revision. A new Humble-bound retained campaign is required to close
the current input-count and sanitizer portion of `VER-FUZZ-VAL-001`. The
semantic preflight is not evidence that every retained
mutation seed contains every category. Existing fuzz assertions check
rejection-state preservation in the mailboxes, mergers, and listed controller
paths, and the bus driver checks zero fake-HAL exchanges after its rejected
requests. Full `VER-FUZZ-VAL-001` remains open until complete
per-hardware-path rejection evidence is reviewed. The separate 10-million-byte
transport campaign—including arbitrary XRCE
byte/split/corruption input through the complete session parser and ring-wrap
cases—also remains open; the seven-harness validation smoke is not
`VER-FUZZ-TRN-001` evidence. The project-owned middleware fault proxy is also
exercised as an
executable lifecycle test: a 1 ms executor failure stops and invalidates the
session, all 24 unavailable finalizers complete the modeled teardown in 263 ms,
backoff remains sliced at 10 ms, all 47 entity-creation boundaries run again,
the generation advances from 1 to 2, and the maximum modeled task-heartbeat
gap is 11 ms. A separate C++ process test exercises the installed XML launch
path, exact Agent command line, YAML loading, configuration order, and motion
authorization. A linked real-rcl/rmw test now withholds configuration-service
replies across the 100 ms client timeout and a session change, then releases
the late replies and proves that they cannot alter motion authorization.
Wire-level XRCE reply/ACK injection, withheld XRCE ACKs, and target TX-DMA/TC
fault injection remain open. The portable coverage
baseline now satisfies the 90%/80% gate in the
[verification plan](framework/verification.md); the remaining validation
semantic-equivalence clauses and full real-middleware/transport campaigns are
release-blocking items. Some need no MCU, while TX-DMA/TC timing and recovery
require the target.

The USART1 harness treats incoming XRCE data as opaque bytes. It validates the
production ring bookkeeping and byte-preserving copy path. The conformance
target now proves that the same pinned client framing implementation builds on
the host and correctly handles known-good frame splits and stuffing, rejects
the fixed malformed-frame table, and has the bounded truncation behavior
described above. Neither target runs arbitrary input through the complete
client session parser or proves that every invalid XRCE submessage is stopped
before an application callback, so that part of `VER-FUZZ-TRN-001` remains
open. A handwritten substitute parser is not acceptable evidence.
