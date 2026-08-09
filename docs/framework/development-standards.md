# Development Standards

## Scope

These rules apply to all first-party production code, tests, build definitions,
and generated-artifact configuration for the rewrite. They complement the
[architecture](architecture.md), [ROS interface contract](ros-interface-contract.md),
[reliability contract](reliability-and-safety.md), and
[verification plan](verification.md).

The [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
is the base for first-party C++. Rules in this document pin the project's
embedded and ROS-specific choices where the upstream guide permits alternatives.

## Language and runtime boundaries

### Host

- First-party host runtime code must use exactly the C++17 language baseline
  selected for this release. A later language standard requires change control.
  Every project-owned production ROS node and every project-owned component in
  the control/data path must use `rclcpp`/C++; none may use `rclpy` or Python.
- First-party packages shall contain no Python control node, data-processing
  script or Python serial bridge. Python launch descriptions
  and compiled C++ executables are used for project-owned deployment logic.
- Pinned upstream ROS 2 tooling and dependencies may use or invoke Python. This
  includes `ros2` CLI/launch infrastructure, `colcon`, `ament`, `rosidl`,
  code generators, and their transitive implementation dependencies. Such
  upstream use is not a first-party control/data path and is not prohibited;
  it shall be recorded and version-pinned like any other dependency.
- Expected failures use explicit status/result values, not new exceptions.
  Exceptions thrown by ROS or another dependency are caught at the process
  boundary, logged once with context, and converted to a controlled nonzero exit
  or documented recovery transition. Host targets are not built with
  `-fno-exceptions` because ROS dependencies may throw.

### Firmware

- The first-party MCU framework and ownership orchestration use C++17.
  Individual application or driver modules may use C11 or C++17 behind a
  documented boundary to C-based HAL, FreeRTOS, micro-ROS, or generated code.
  Handwritten C++ follows the Google C++ rules below; mixing languages inside
  one logical module requires a documented boundary.
- STM32 HAL, CMSIS, FreeRTOS, micro-ROS/rclc, and generated ROS type support may
  remain C.
- C APIs are isolated behind narrow C++ adapters. C headers are included inside
  `extern "C"` where required. IRQ/HAL callback symbols are tiny C-linkage shims
  that timestamp/copy/notify and immediately delegate to the owning adapter.
- An ISR above `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` shall call no
  FreeRTOS API, HAL state machine, application callback, or actuator function.
  No active USART1 interrupt has such an exception: RX DMA, TX DMA, and USART1
  run at the FreeRTOS-safe transport priority and use the standard HAL handlers
  and callbacks. Any new above-ceiling ISR requires an architecture review.
- Domain code must not include STM32 HAL, FreeRTOS, CMSIS, or rclc headers.
  Dependencies point inward through explicit interfaces; C handles and macros do
  not leak into motor, servo, safety, or command-validation APIs.
- C++ firmware is built with `-fno-exceptions -fno-rtti`. Firmware must not use
  iostreams, locale, filesystem, regular expressions, or facilities that hide
  allocation.
- Firmware has no general-purpose runtime heap. A statically backed 48 KiB CCM
  arena may allocate while micro-ROS entities are built in `CREATE_ENTITIES`
  and may be finalized/reset in `TEARDOWN`. The allocation seal is set before
  `ACTIVE`; while sealed there are zero successful allocate, reallocate, or
  deallocate calls, and every attempted post-seal allocator call is counted and
  treated as fatal. Reconnect shall return arena use and every pool to the
  pre-cycle baseline before rebuilding.
- The 2 KiB `MotorControlTask` stack is also CPU-only and is placed in CCM. Its
  linker section and the arena remain separately identifiable in the map; no
  pointer to either object may be passed to a DMA peripheral.
- All RTOS objects, message backing stores, executor capacity, XRCE streams,
  DMA buffers, driver state, mailboxes, event FIFOs, and service slots outside
  that bounded entity arena are statically backed.
- C firmware uses fixed arrays/structs and explicit capacity fields; C++
  firmware uses bounded storage such as `std::array`, fixed structs, bounded
  strings, or an approved fixed-capacity container. Firmware must not use
  `std::vector`, dynamically growing `std::string`, `std::function`, or owning
  raw pointers in steady-state paths.

Third-party C remains allowed without rewrite. The host runtime/framework stays
C++17; the approved C11 allowance is limited to first-party MCU code and does
not permit a new host-side C or Python control bridge.

## Google C++ conventions

- Handwritten C++ source files use `.cc`, C sources use `.c`, and headers use
  `.h`. Each C++ header is self-contained and uses a Google-style project/path
  include guard. Do not use `#pragma once`.
- Formatting is enforced with the repository-pinned `clang-format` and a
  `.clang-format` based on `Google`, 80-column limit. No hand-formatted exception
  is accepted without an explanatory disable marker around the smallest region.
- Types and functions use `UpperCamelCase`; variables use `snake_case`; data
  members have a trailing underscore; constants use `kUpperCamelCase`;
  namespaces use `snake_case`. Avoid abbreviations that are not in the project
  glossary.
- Include order follows Google style: related header, C system, C++ standard,
  third-party, then project headers, with one blank line between groups. Include
  what the file uses; do not rely on transitive includes.
- Headers expose the smallest API possible. Prefer composition and concrete
  value types. Host ownership is represented by values or `std::unique_ptr`;
  firmware ownership uses static values/references and does not allocate an
  owning smart pointer. A raw pointer/reference is non-owning and must not
  outlive its documented owner.
- Use fixed-width integer types at wire, register, timer, and persistent
  boundaries. Use `size_t` for sizes. Narrowing and signed/unsigned conversion
  must be explicit and preceded by range validation.
- Host durations use `std::chrono`. Firmware values include units in the type or
  name, for example `timeout_ms`, `period_us`, and `voltage_mv`. Bare numeric
  timing constants are forbidden.
- Prefer scoped enums, `constexpr`, `const`, brace initialization, `nullptr`, and
  `static_assert`. C-style casts, variable-length arrays, implicit fallthrough,
  and exposed mutable namespace globals are forbidden in first-party C++.
  Private statically backed owner/state objects required by the architecture
  are allowed when they have exactly one documented owner, no nontrivial static
  initialization, and no externally writable alias.
- Macros are limited to include guards, compiler/platform selection, and
  unavoidable C-library integration. New constants and inline operations are
  typed C++ declarations.
- Comments explain invariants, ownership, units, safety reasoning, and hardware
  errata. They do not narrate obvious syntax. Public APIs and every hardware
  workaround cite the governing requirement or datasheet/reference.

## Concurrency and real-time rules

- Each peripheral and mutable state object has exactly one owning task. Cross-task
  transfer uses the bounded mailbox/FIFO/service-slot design in the architecture.
- ISR code never blocks, allocates, logs, parses ROS data, calls a device state
  machine, or invokes an application callback. Only ISR-safe RTOS APIs may be
  called from interrupt context.
- ROS callbacks validate and copy into bounded storage. They never perform UART,
  I2C, SPI, ADC, flash, servo, motor, or sleep/wait operations.
- Every wait has a finite deadline. Lock ordering, task ownership, and maximum
  critical-section length are documented beside the protected resource.
- High-rate commands use latest-wins semantics. Code must not substitute an
  unbounded FIFO. Button events alone use FIFO 16, drop-oldest; service capacity
  and deadlines are fixed by the architecture.
- Safety state and the 200 ms per-motor lease use a monotonic clock and are
  independent of ROS/system time. The 1 kHz safety release must not share a
  blocking path with the 100 Hz PID update.
- Shared counters visible across ISR/task boundaries use an atomic or a proven
  critical section. Diagnostic counters do not silently wrap; use the saturation
  behavior defined by their type/contract.

## Validation, errors, and diagnostics

- Validate the complete message before changing any state. Counts are checked
  before indexing; IDs before subtraction; lengths before copying; enum values,
  finite floats, ranges, and cross-field invariants before enqueue.
- Public services use the `mentor_pi_interfaces/Result` values exactly:
  `OK`, `INVALID_ARGUMENT`, `OUT_OF_RANGE`, `BUSY`, `TIMEOUT`, `IO_ERROR`,
  `UNSUPPORTED`, or `PARTIAL`. Do not return `OK` for transport receipt alone.
- Hardware and executor errors use the error-source values in the ROS interface
  contract. Each rejected command, replacement/drop, timeout, transport loss,
  watchdog trip, and recovery has an observable counter or state transition.
- Logs are rate-limited and never emitted from ISR context. Host logs include
  endpoint/operation and result; firmware diagnostics avoid dynamic strings.
- Assertions protect programmer invariants, not user input. A malformed ROS
  message returns an error or is rejected without resetting the controller.

## Builds, warnings, and automated tooling

First-party host targets use CMake and ROS packages use `ament_cmake`.
Host-native firmware logic tests also use CMake. The authoritative MCU build is
CMake/Ninja with the pinned Arm GNU toolchain and shall emit ELF, HEX/BIN, map,
and size reports. The root Makefile is the supported thin developer frontend;
it shall not introduce another dependency graph. Third-party IDE-generated
build graphs are not supported. Tool versions and generated-build inputs are
pinned in the repository or CI image.
The micro-ROS static-library generator shall detach every cloned repository at
the reviewed `microros_sources.lock` commit, pin temporary copied sources such
as `geometry2/tf2_msgs`, reject missing or unexpected repositories, and verify
the final archive SHA-256. Recording branch-head commits only after a build is
not a reproducible dependency lock.

Firmware configuration shall fail closed to the single supported PID release
artifact. Normal, CI release, and reproducibility builds shall report
`NORMAL_CLOSED_LOOP_DEFAULT` and `control_mode=CLOSED_LOOP`; legacy mode flags,
aliases, and alternate artifacts are rejected. The build uses a 6 RPS
implementation ceiling and a 1000-permille output clamp while retaining each
motor model's lower profile limit. Release records include the binary and
source hashes. No build flag, unit test, or legacy-derived constant may label
PID performance or polarity release-qualified without the required physical
HIL record.

Mandatory gates:

1. `clang-format` check with Google style; CI checks but never silently rewrites.
2. `cpplint`/`ament_cpplint` for Google naming and header rules.
3. `clang-tidy` with `bugprone-*`, `performance-*`, `portability-*`,
   `readability-*`, the repository-reviewed exclusions below, and an explicitly
   reviewed `modernize-*` subset. Any additional local suppression requires an
   inline reason and issue/reference.
4. First-party warnings enabled and treated as errors:
   `-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion -Wshadow
   -Wdouble-promotion -Wformat=2`. Third-party includes are marked `SYSTEM` so
   their warnings cannot force weakening first-party checks.
5. Host-native unit tests under ASan+UBSan and in a separate TSan job. Fuzz jobs
   and coverage gates follow [verification.md](verification.md).
6. Firmware static analysis, clean release/debug builds, map-budget check, stack
   report, and post-seal allocator trap.
7. `colcon test` plus `colcon test-result --verbose` for ROS packages, interface
   generation, contract inspection, and launch tests.
8. Markdown link/style check for framework documents and a traceability check
   that every requirement/audit ID maps to a verification test.

CI must contain, at minimum, host debug/sanitizer, host release, firmware debug,
firmware release/size, fuzz smoke, static-analysis/style, and documentation
jobs. Long HIL, 60-minute load, reconnect/reset, and 24-hour soak jobs may run on
dedicated hardware, but their signed results are required for release.

The checked-in hosted workflows and their local entry points are run in
[normal-computer Tutorial 07](../tutorials/normal-computer/07-run-stress-soak-and-release-gates.md). Hosted jobs
use no project secret and cover documentation/traceability, format,
`clang-tidy`, native Debug ASan/UBSan, native Release, TSan, deterministic fuzz
smoke, generated CDR/introspection checks on ROS 2 Humble amd64 and arm64, and
the pinned
default PID firmware reproducibility/size build. The documentation gate is
implemented by `tools/check_framework_docs.py`; it checks relative files and
anchors, mandatory requirement mappings, every stable audit row, referenced
verification definitions, and orphaned verification cases.

These jobs do not close a verification case whose acceptance requires physical
measurements, a connected Agent/MCU, or endurance duration. The portable
90%/80% coverage gate and C++ process-level ROS launch integration are checked
in. A deterministic executable test also drives the production project-owned
lifecycle, cursors, and fault proxy through executor failure, all 24 bounded
finalizers, sliced backoff, all 47 recreation boundaries, and a generation-2
recovery. A separate linked real-rcl/rmw test now proves that configuration
service replies withheld across the client timeout or a session change are
discarded without changing motion authorization. Successful fuzz runs are now
published under no-overwrite, checksum-closed evidence directories bound to
production-source, test-input, corpus, and toolchain digests. The retained
11,666,669-execution ASan/UBSan campaign predates the Humble-only conversion
and is historical evidence only. A new Humble-bound campaign is required for
the current input-count portion of `VER-FUZZ-VAL-001`; comprehensive
no-invalid-hardware-call review remains open. The arbitrary-input XRCE
session-parser campaign, wire-level XRCE reply/ACK injection and
withheld-XRCE-ACK campaign, and target transport TX-DMA/TC fault injection
remain mandatory before D5 even though part of that work can be completed
without the board. The checked-in firmware workflow now adds a separate
default-PID CMake `Debug` build and runs Clang 18 over every first-party
translation unit in its actual Arm GNU compile database. This is in addition
to, and does not overwrite, the `MinSizeRel` reproducibility artifact.

The `.clang-tidy` profile deliberately excludes only checks that conflict with
the fixed embedded ABI or make diagnostics less actionable in this codebase:
`bugprone-easily-swappable-parameters` for HAL/ROS callback signatures;
`readability-identifier-length` for conventional names such as `id`, `tx`, and
axis names; `readability-named-parameter` for intentionally unused virtual test
arguments; `readability-magic-numbers` for protocol/register and boundary-test
vectors; `readability-non-const-parameter` for top-level pointer constness;
`readability-redundant-member-init` because explicit zero initialization is a
reviewed embedded-state convention; and
`readability-function-cognitive-complexity` because assertion macros and
explicit state machines inflate its lexical score without measuring bounded
runtime behavior. The bare-metal Arm analysis invocation additionally disables
`bugprone-dynamic-static-initializers`, which produces false positives for
scalar `constexpr` and `extern` declarations with the Arm-none-eabi headers;
`bugprone-reserved-identifier`, because linker symbols, GNU `--wrap` entry
points, and C++ ABI hooks have externally fixed reserved names; and
`modernize-avoid-c-arrays`, because STM32 HAL DMA and C ABI boundaries require
raw statically sized buffers. It also disables
`readability-inconsistent-declaration-parameter-name` because the fixed vendor
HAL prototypes use ST parameter names while the C-linkage definitions retain
project naming. The native analysis retains all four checks.
The Arm analysis also prevents Clang-only compiler warnings from inheriting
GCC's `-Werror`; the preceding pinned GCC build still treats the complete
first-party warning set as errors, while every enabled Clang-Tidy diagnostic
remains an error. Compiler warnings, sanitizer tests, the remaining analysis
checks, and targeted state-machine tests remain mandatory. Changing this list
requires review of this document, `.clang-tidy`, and the target-CI helper
together.

## Vendor and generated code

The following are exempt from first-party formatting/naming only:

- STM32Cube-generated startup, HAL glue, linker, and peripheral initialization;
- ST HAL/CMSIS, FreeRTOS, micro-ROS/rclc, Micro XRCE-DDS, ROS-generated type
  support, and other pinned third-party sources;
- generated message/service files and generated test artifacts.

Exemption does not mean trust without a boundary:

- vendor/generated files are isolated in their own targets/directories and are
  never bulk-formatted;
- version, origin, license, local patch list, and checksum are recorded;
- local patches are minimal, reviewable, reproducible, and tested at the C++
  adapter boundary;
- warnings are suppressed only on the third-party target, never globally;
- CubeMX regeneration must be reproducible and may not overwrite first-party
  code; generated callbacks remain shims;
- files under `docs/reference/` are historical evidence and must never be linked
  into the product.

## Definition of done for a code change

A change is complete only when it:

1. names its governing requirement, audit row, and test IDs;
2. preserves task ownership, bounded memory, deadline, and safe-state invariants;
3. includes boundary/error/concurrency tests proportional to the change;
4. passes format, warnings, analysis, sanitizer, unit, and affected HIL gates;
5. updates interface/framework documentation when observable behavior changes;
6. records measured flash, RAM, stack, traffic, and timing deltas for firmware
   changes; and
7. introduces no first-party Python node or control/data-path component and no
   unreviewed vendor/generated modification.
