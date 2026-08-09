# RRCLite v2 status and next steps

This is the current handoff for the next coding or hardware session. Start with
the numbered sequence in the [root README](../README.md#start-here); use the
[firmware stabilization log](firmware-stabilization-log.md) for historical
board evidence.

## Current source status (2026-08-09)

The planned source migrations are implemented:

- the directly buildable Humble workspace contains only
  `mentor_pi_ros2/src/{mentor_pi_interfaces,mentor_pi_bringup,mentor_pi_hardwares}`;
- active manifest validation no longer depends on the removed local schema
  snapshot or `xml-model` declarations;
- firmware builds one `NORMAL_CLOSED_LOOP_DEFAULT` PID artifact with
  `control_mode=CLOSED_LOOP`, a 6 RPS implementation ceiling, a
  +/-1000-permille output limit, model-specific lower limits, atomic command
  validation, independent 198 ms leases, session-loss disarming, and
  transport-failure shutdown;
- the motor domain has one default PID configuration and no locked,
  direction-check, or alternate control-mode branch; malformed build-time
  configuration still fails closed;
- handoff packaging contains one `firmware-pid-release/` ELF/Hex/Bin/Map set
  plus `BUILD-METADATA.txt` and `BUILD-MODE.txt`;
- build and handoff metadata keep `release_qualified=0` until the required HIL
  evidence exists;
- all project launch files are Python;
- the tutorial sequence has two complete eight-document tracks under
  `docs/tutorials/onboard-computer` and `docs/tutorials/normal-computer`;
- the RDK X5 track supports Docker-free Ubuntu 22.04 arm64 firmware generation
  with the checked local Arm GNU 13.2.1 toolchain, conventional colcon builds,
  source-bound direct ros2 launches, and an onboard gate that excludes fuzzing
  and the normal-computer coverage toolchain; the Ubuntu 24.04 normal-computer
  track retains the complete pinned Docker suite, including coverage and Clang
  18 fuzzing; and
- onboard dependency preparation treats ROS 2 Humble as an externally supplied
  prerequisite, installs the remaining native dependencies, and verifies and
  installs the checked STM32CubeProgrammer 2.23.0 arm64 package after explicit
  license acceptance.

The new controller launch starts the compiled micro-ROS Agent and configuration
supervisor together and shuts down the launch if either process exits. The
normal adaptive entry point remains:

```sh
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED make start
```

After entering the project Humble environment, conventional direct use is:

```sh
source tools/setup_onboard_ros_environment.sh
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  ros2 launch mentor_pi_bringup controller.launch.py \
    agent_executable:="${MENTOR_PI_AGENT_EXECUTABLE}"
```

Do not set that acknowledgement until the passive checks and guarded-fixture
requirements in Tutorials 01--06 are actually satisfied.

## Verification evidence

The following checks pass against the current source:

- strict local C++17 compilation of the changed motor domain, controller
  runtime, controller tests, and fuzz target with the project warning set;
- locally linked and executed MCU domain tests: `All MCU domain checks passed`;
- locally linked and executed controller integration tests:
  `controller integration tests passed`;
- tutorial/runtime action contracts;
- diagnostic capture contracts;
- host handoff and relocation contracts;
- firmware artifact verification;
- single-artifact board-handoff packaging;
- direct and guided flash fixtures;
- framework documentation/link/sequence validation: 41 Markdown files,
  189 relative links, and 16 ordered tutorials; and
- `git diff --check`.

After the dual-host change, focused tutorial, active-build, host-handoff,
firmware-artifact, dependency-provenance, and board-handoff tests pass. The
pinned Ubuntu 24.04 quality container completed its portable/native Debug test
suite, then stopped at three existing formatting violations in unchanged C++
files: `controller_tests.cc:19`, `controller_tests.cc:20`, and
`configuration_supervisor_launch_test.cc:303`.

The normal-computer `make firmware` path regenerated the locked Humble
micro-ROS archive and built/verified the Docker-pinned PID artifact with ELF
SHA-256 `d3edebc367c6bd731d3cbf8bd0ca4cb210880308e7e57389dfccbbc503eba207`.
Its bound source SHA-256 is
`4c6882ce2296711e007017e760243e6f9efd3e1b96caa66b4fc2f1ef43c429c9`.
It reports `builder_mode=docker-pinned`, `release_qualified=0`, flash usage
156,764/524,288 bytes, SRAM 102,072/131,072 bytes, and CCM 51,200/65,536 bytes.

Before the final motor-domain cleanup, the pinned Humble host stage built all
three ROS packages and reported 1,652 tests, zero errors, and zero failures.
The quality container then found one STM32 source-contract failure: the passive
M1/M2/M3/M4 wheel mapping comment was absent. That marker was restored and the
legacy motor-mode implementation exposed by the review was removed. The
focused local domain/controller builds and executions above cover those final
edits.

The Docker-pinned firmware build is current, but the new native RDK X5 path has
not yet run on Ubuntu 22.04 arm64. No flash or HIL action was performed for the
new artifact. The complete quality-container result is failing, not passing,
until the unrelated formatting violations above are resolved and the suite is
rerun.

## Next agent: do these in order

1. Resolve the three recorded formatting failures without changing behavior,
   rerun `./tools/run_quality_tests_container.sh`, then run the remaining
   normal-computer release gates including fuzzing.
2. On the RDK X5 Ubuntu 22.04 arm64 host, follow onboard Tutorials 01--03 to
   verify the SHA-256-checked native toolchain download, native micro-ROS
   generation, `make firmware`, conventional `rosdep`/`colcon build`/`colcon
   test`, build-state recording, and direct `ros2 launch`. Confirm the native
   artifact matches the reviewed micro-ROS hashes and two clean builds are
   reproducible. This is not established by the amd64 Docker result.
3. Package the current one-artifact PID handoff and retain the actual ELF hash.
   On a supported Docker runtime host, also confirm read-only `/run/udev`
   mapping and occupied-device rejection.
4. With actuator power disconnected, flash that exact artifact and complete
   Tutorials 01--05. Verify the graph, supervisor `READY`, heartbeat,
   diagnostics, passive encoder signs, IMU samples, and Agent restart recovery.
5. Only after every passive gate passes, follow Tutorial 06 with raised or
   equivalently guarded wheels, a current-limited supply, and a reachable
   physical motor-power stop. Use Tutorial 07 for campaign evidence and
   Tutorial 08 only after hardware checkout is complete.

## Physical and release boundary

The last physically exercised candidate is the historical locked image with
ELF SHA-256
`de9e8e18611cca780cbb55903f781a906c9437e183c317f9012b1d0f43476168`
from 2026-08-07. It established graph creation, stable transport/session
behavior, heartbeat and IMU rates, the six-face IMU transform, and passive
wheel/encoder ownership with motor power disconnected. It predates the current
default-PID migration and must not be flashed or treated as evidence for the
current source.

The following remain unqualified until machine-generated HIL or instrument
evidence is recorded:

- powered motor direction, ticks per revolution, PID/filter/deadband,
  electrical current, thermal behavior, and operating range;
- positive-rotation IMU orientation and extended timing;
- battery-divider/VREFINT scaling and alarm timing;
- PWM, RGB, buzzer, LED, OLED, bus-servo, and button electrical behavior;
- watchdog, USART1, UART5, I2C, reset/fault, and Agent recovery timing;
- stack/resource and escaped-wire-traffic margins; and
- the 500 Hz/60-minute run, three 100-cycle recovery campaigns, and 24-hour
  soak.

Software tests do not establish powered-motion performance or physical release
qualification. Tutorial 07 intentionally reports incomplete qualification
when required physical metrics are absent.

## Repository transfer

No Git remote is required. Preserve unrelated changes and transfer with a
trusted clone, bundle, or archive. Do not commit build outputs, downloaded
firmware dependencies, logs, `.pio/` remnants, or diagnostic evidence. The
ignored `docs/reference/` legacy snapshot must be copied separately only if it
is still needed; active builds must not depend on it.
