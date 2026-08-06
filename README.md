# Mentor Pi RRCLite v2

RRCLite v2 is a clean ROS 2 Jazzy controller stack for the Mentor Pi
RRCLite V1.0 board. It replaces the legacy Python serial bridge and proprietary
MCU packet dispatcher with a native micro-ROS client on the STM32F407VET6.

The physical runtime path remains:

```text
ROS 2 nodes <-> native micro-ROS Agent
            <-> USB-C / CH9102F / 1,000,000-baud USART1
            <-> FreeRTOS firmware
            <-> robot hardware
```

Project-owned host runtime code is C++17. MCU orchestration and drivers are
C++17 around the C STM32 HAL, FreeRTOS, and micro-ROS libraries. No Python
process is in the runtime data path.

## Status

This repository is an engineering build awaiting first-board HIL
characterization. Native tests and cross-build checks can prove software
behavior, bounds, and memory layout without a board. Motor/encoder polarity,
PID tuning, analog scaling, IMU axes, peripheral timing, watchdog timing, and
the endurance/reconnect acceptance tests require the physical RRCLite V1.0.

The normal firmware build is deliberately motor-locked while that evidence is
missing. It accepts valid zero/stop motor commands but rejects every nonzero
motor target. JGA27 currently uses a provisional encoder-polarity factor of
`-1` derived from legacy controller evidence; neither that factor nor any PID
profile is release-qualified.

Do not connect unrestricted actuator power for the first flash. Follow
[Flashing and first bring-up](docs/flashing-and-first-bringup.md).
Use the blank [board-arrival evidence record](docs/board-arrival-bringup-checklist.md)
for the session; its installed read-only diagnostic command produces the
manifested archive needed for actionable bug feedback without opening the
serial transport or publishing a command.

## Repository layout

- [`docs/framework/`](docs/framework/) is the normative design and verification
  specification.
- [`src/mentor_pi_interfaces/`](src/mentor_pi_interfaces/) contains the bounded
  v2 messages and services.
- [`src/mentor_pi_bringup/`](src/mentor_pi_bringup/) contains the native Agent
  deployment assets and C++ configuration supervisor.
- [`firmware/mentor_pi_mcu/`](firmware/mentor_pi_mcu/) contains the portable
  domain, drivers, controller workers, micro-ROS runtime, and STM32 platform.
- [`tools/`](tools/) contains pinned dependency and reproducible build helpers.
- `docs/reference/` is local legacy evidence and is intentionally ignored by
  Git. Production code must not depend on it.

The parent `.gitignore` also excludes the legacy worktree's nested `.git`
boundary without modifying that independent repository. As with every Git
ignore rule, it prevents new parent-repository tracking but does not remove a
path that an existing parent index already tracks. When importing this tree
into such a repository, review the index first and, if necessary, remove only
the cached parent entry with `git rm -r --cached -- docs/reference/`; the local
evidence files remain in place.

## Build firmware

Docker Desktop or Docker Engine is required for the pinned ARM build from a
non-Ubuntu development host.

```sh
./tools/bootstrap_firmware_dependencies.sh
./tools/build_microros_library.sh
./tools/build_firmware.sh
```

The final command emits ELF, HEX, BIN, map, and size artifacts under
`firmware/mentor_pi_mcu/build/stm32/`. Linker assertions reject images that do
not retain at least 20% flash, DMA-accessible SRAM, and CCM headroom.

The micro-ROS regeneration step is networked but deterministic: it detaches all
35 fetched ROS repositories at
`firmware/mentor_pi_mcu/config/microros_sources.lock`, pins the temporary
`geometry2/tf2_msgs` copy, and rejects a generated archive whose SHA-256 does
not match the reviewed artifact lock. Updating an interface or dependency
therefore requires a deliberate lock/hash review rather than silently taking a
new Jazzy branch head.

## Flash without a debug probe

RRCLite V1.0 can be flashed through the USB-C port labelled USB serial
1/download. It routes through CH9102F to the STM32F407 factory USART1
bootloader; the separate 5 V/5 A USB-C port is power-output only. Install
STM32CubeProgrammer, disconnect all actuators, hold `BOOT`, tap `RST`, release
`BOOT`, identify the exact CH9102F device, then run:

```sh
RRCLITE_UART_BOOTLOADER_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED \
  pio run -e rrclite_uart -t upload \
    --upload-port /dev/cu.wchusbserial-REPLACE_ME
```

Use the matching `/dev/serial/by-id/...` path on Ubuntu. The command verifies
the source-bound motor-locked artifact, snapshots the ELF, programs it at
115200/8E1, and requests CubeProgrammer read-back verification. On success,
tap `RST` normally; the application then uses the same connector at
1,000,000/8N1. See
[Flashing and first bring-up](docs/flashing-and-first-bringup.md) for the GUI,
SWD, commissioning, and safety procedures.

`./tools/build_firmware.sh` defaults to a motor-locked image. Only after the
passive encoder-direction checks in the bring-up guide, with all wheels raised
or equivalently guarded and a current-limited supply, build the separate
commissioning image with both explicit acknowledgements:

```sh
RRCLITE_MOTOR_COMMISSIONING=1 \
RRCLITE_MOTOR_COMMISSIONING_ACK=MOTORS_RAISED \
./tools/build_firmware.sh
```

That image is capped at 0.25 RPS and 300 permille output and is not a release
image. The command overwrites the artifacts in the STM32 build directory, so
record its hash as commissioning-only and rebuild the default locked image
after the test.

## Run portable tests

Run every interface, MCU domain, driver, controller, micro-ROS compile-check,
and host-supervisor native suite with:

```sh
./tools/run_native_tests.sh
```

The script uses Python only for interface source-contract tests; Python is not
part of either runtime. Individual suites can also be run directly, for
example:

```sh
cmake -S firmware/mentor_pi_mcu -B build/mentor_pi_mcu-native -G Ninja \
  -DBUILD_TESTING=ON
cmake --build build/mentor_pi_mcu-native
ctest --test-dir build/mentor_pi_mcu-native --output-on-failure

cmake -S firmware/mentor_pi_mcu/drivers \
  -B build/mentor_pi_mcu-drivers -G Ninja -DBUILD_TESTING=ON
cmake --build build/mentor_pi_mcu-drivers
ctest --test-dir build/mentor_pi_mcu-drivers --output-on-failure

python3 -m unittest discover -s src/mentor_pi_interfaces/test -v
```

ROS packages are supported on ROS 2 Jazzy with Ubuntu 24.04, amd64 or arm64.
The host deployment and configuration schema are described in
[`src/mentor_pi_bringup/README.md`](src/mentor_pi_bringup/README.md).
The clean-machine dependency, merged Release build, relocation proof, and
checksummed host-handoff workflow are in
[Host preparation and handoff](docs/host-preparation-and-handoff.md).

Minimal, schema-correct commands for inspecting telemetry, publishing bounded
commands, and calling services are in
[ROS 2 CLI examples](docs/ros2-cli-examples.md).

## Software-only quality gates

The checked-in GitHub workflows run documentation/traceability checks, C++
format and static analysis, native Debug ASan/UBSan and Release tests, a
separate TSan job, an enforced 90%/80% portable coverage gate, bounded
libFuzzer smoke, generated CDR/introspection tests on ROS 2 Jazzy amd64, and a
pinned two-build firmware reproducibility comparison. The same primary native
checks can be run locally:

```sh
./tools/check_framework_docs.py
./tools/test_gitignore_contract.sh
./tools/run_native_ci_tests.sh --build-type Debug --sanitizers on
./tools/run_native_ci_tests.sh --build-type Release --sanitizers off
./tools/run_tsan_tests.sh
./tools/run_coverage_tests.sh
```

The explicit portable first-party manifest now measures 91.29% line and 80.94%
branch coverage. The local script and hosted coverage job enforce the required
90%/80% release threshold.

The firmware comparison requires the dependencies and generated library from
the firmware build section, then runs with:

```sh
./tools/check_firmware_reproducibility.sh
```

These hosted jobs require no project secrets and no connected board. See
[CI and hardware qualification gates](docs/ci-and-hardware-gates.md) for the
exact workflow boundary, local clang commands, remaining software-only work,
and the HIL evidence that must wait for physical hardware.

## Safety contract

- Normal firmware accepts motor zero/stop commands but cannot arm nonzero
  motion; the host `motion_enabled` topic does not override this lock.
- Commissioning motion requires the two exact build variables above, raised
  wheels/equivalent guarding, a current-limited supply, and prior passive
  encoder-direction checks. It remains capped at 0.25 RPS and 300 permille.
- Every motor has an independent 200 ms command lease.
- Invalid commands are rejected atomically and do not refresh a lease.
- Transport failure or RX overrun disarms motors and tears down the session.
- PWM and bus servos hold their last accepted state across host loss.
- High-rate commands use latest-value mailboxes, never unbounded FIFOs.
- The micro-ROS task alone owns ROS entities and the USART1 transport.
- The safety supervisor is the only task allowed to refresh the watchdog.

The complete normative rules and acceptance limits are in
[`docs/framework/reliability-and-safety.md`](docs/framework/reliability-and-safety.md)
and [`docs/framework/verification.md`](docs/framework/verification.md).
