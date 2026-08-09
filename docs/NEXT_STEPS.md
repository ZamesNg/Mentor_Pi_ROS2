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
- all project launch files are Python; and
- the tutorial sequence is eight documents: setup, PID build/flash, Humble host,
  passive bringup, hardware characterization, guarded ROS 2 CLI checkout,
  stress/soak gates, and `mentor_pi_hardwares`.

The new controller launch starts the compiled micro-ROS Agent and configuration
supervisor together and shuts down the launch if either process exits. The
normal adaptive entry point remains:

```sh
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED make start
```

After entering the project Humble environment, conventional direct use is:

```sh
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  ros2 launch mentor_pi_bringup controller.launch.py
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
- framework documentation/link/sequence validation: 33 Markdown files,
  158 relative links, and eight ordered tutorials; and
- `git diff --check`.

Before the final motor-domain cleanup, the pinned Humble host stage built all
three ROS packages and reported 1,652 tests, zero errors, and zero failures.
The quality container then found one STM32 source-contract failure: the passive
M1/M2/M3/M4 wheel mapping comment was absent. That marker was restored and the
legacy motor-mode implementation exposed by the review was removed. The
focused local domain/controller builds and executions above cover those final
edits.

The complete pinned quality-container rerun remains outstanding because the
environment's escalation approval service rejected
`./tools/run_quality_tests_container.sh` with a usage-limit error. This is an
environment limitation, not a passing result. No native Humble rerun, STM32
artifact build, flash, or HIL action was performed after the final edits.

## Next agent: do these in order

1. Run `make test` on a host where the pinned Ubuntu 22.04/Humble Docker path
   is permitted. If the host stage is already current, at minimum run
   `./tools/run_quality_tests_container.sh`. Address real failures, then record
   the exact totals here.
2. On native Ubuntu 22.04 with ROS 2 Humble, verify conventional workspace use
   from `mentor_pi_ros2` with `rosdep`, `colcon build`, and `colcon test`, then
   smoke-test `ros2 launch mentor_pi_bringup controller.launch.py` with a safe
   serial fixture or disconnected actuator power. On one supported Docker
   runtime host, also confirm the read-only `/run/udev` mapping resolves the
   mapped CH9102F identity and that an occupied serial device is rejected.
3. Build the default PID firmware with the pinned ARM toolchain, run artifact
   and memory verification, package the one-artifact handoff, and record the
   actual ELF SHA-256. Do not reuse a historical hash.
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
