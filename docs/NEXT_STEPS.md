# RRCLite v2 status and next steps

This file records changing project status. For operating instructions, begin
with the numbered tutorial sequence in the [root README](../README.md#start-here).
The detailed first-board failures and corrections are recorded in the
[firmware stabilization log](firmware-stabilization-log.md).

## ROS workspace migration verification

The workspace/schema refactor was implemented and reviewed on 2026-08-08:

- the repository-root source tree is absent, and the three packages exist only
  under `mentor_pi_ros2/src/`;
- the local package-schema snapshot, package `xml-model` declarations, and all
  active build, generation, fingerprint, deployment, packaging, fixture, and
  CI dependencies on it are removed;
- the current host source fingerprint is
  `66b79ca27f608732e7b5be3103ffb6bb53bf5b9ba9348ecf479136cca5b9aaad`;
- the pinned Ubuntu 22.04/ROS 2 Humble `make host` path built all three packages,
  passed 1,653 tests with zero errors or failures, and relocation-verified the
  merged install; and
- focused workspace, firmware-domain, artifact, tutorial/runtime, handoff,
  active-build-policy, documentation, shell-syntax, and whitespace checks
  passed. The firmware-domain group passed all 8 tests from a read-only source
  mount, including the moved shared-interface include path.

The final `make test` quality-container stage did not start: Docker Buildx was
unable to update its sandboxed home-state activity file. Rerun that broad gate
when Docker home-state access is available. No STM32 release image or physical
hardware result was produced by this refactor.

## Pending next-version firmware migration

The ROS workspace and manifest-schema portions of the next-version migration
are implemented in the current source. The three host packages now live only
under `mentor_pi_ros2/src/`; the root source tree and local package-schema
snapshot are gone. Host and firmware tooling use the new workspace paths, and
the package manifests rely on the normal ament/rosdep/colcon validation path.

The default-firmware migration remains pending. The next coding agent shall
make these changes agree atomically before claiming the new version:

1. Make `make firmware`, `make flash`, and `make start` use the normal
   motor-enabled `PID` artifact with `control_mode=CLOSED_LOOP`, without the
   former commissioning acknowledgements. Preserve supervisor authorization,
   startup inhibition, atomic validation, model and implementation limits,
   +/-1000-permille output limiting, independent 198 ms leases, session-loss
   disarming, and transport-failure shutdown.
2. Retain independent `firmware-locked`, `flash-locked`, and `start-locked`
   recovery paths and the guarded direction-check image. Keep the old
   commissioning-PID commands temporarily as aliases to the normal PID path,
   not as a separate artifact mode.

The remaining firmware migration must update every affected command,
classification, contract, tutorial, and safety test. Run focused groups first,
then the complete regression target once. Verify `PID`, `LOCKED`, and
direction-check artifacts independently and reject cross-mode substitution.
Update this file again with the resulting hashes, test evidence, and remaining
physical gates.

## Current hardware checkpoint

The last image tested on the physical board is the identity-transform locked
candidate,
ELF SHA-256
`de9e8e18611cca780cbb55903f781a906c9437e183c317f9012b1d0f43476168`.
On 2026-08-07 it:

- created `/mentor_pi/controller`, all seven publishers, all seven
  subscriptions, and all seven MCU services;
- advanced the configuration supervisor to `READY`;
- measured heartbeat at 1.98 Hz and emitted the IMU topic at 49.98 Hz;
- held one continuous Agent session with no transport, reset, reconnect, or
  post-seal allocation fault; and
- produced valid six-face QMI8658 samples that proved the required signed axis
  permutation; and
- mapped the physical wheels to front-left/M1, front-right/M3,
  rear-left/M2, and rear-right/M4 while motor power remained disconnected.

This physically proves graph creation, the allocator/RMW corrections, DMA
transport, timebase, scheduling, stable idle service polling, QMI startup, and
the passive encoder ownership. It does not qualify peripheral HIL, precision
calibration, positive-axis IMU rotation, or powered motion.

The measured wheel signs and IMU axes are now compiled as the
[verified board profile](framework/verified-hardware-profile.md). LED3 is
reserved for successful ROS heartbeat indication. RGB2 is reserved for
low-overhead transport status: red tracks RX progress, green tracks TX
progress, and blue remains off. This source change requires a new locked build
before flash and passive hardware observation.

## Historical prepared firmware

The current source contains the complete controller paths for four closed-loop
motors and encoders, four PWM servos, UART5 bus servos, QMI8658 IMU, battery
monitoring, buttons, LEDs, RGB pixels, buzzer, OLED, watchdog, diagnostics,
micro-ROS publishers/subscriptions, and services.

The last provenance-verified locked candidate had source fingerprint
`a60acfd41d650587f9c7d9f2ff2d997d1fe404a8f54f136a5a82a589e6845c99`:

| Test order | Image | ELF SHA-256 | Purpose |
| ---: | --- | --- | --- |
| 1 | `firmware/mentor_pi_mcu/build/stm32/` | `2bd7fa3e0da06d293b9d72cadcad7ad4fc2bc5735cd42fc4ad99573710d99864` | Superseded locked artifact retained as historical evidence. Rebuild before flashing because current source moves heartbeat status to LED3 and reserves RGB2 for RX/TX. |

Every listed binary predates current positional-PID and next-version migration
work. Treat it as historical evidence, not as a flashable current artifact or
qualification of the future default PID image. The previous commissioning
image is also superseded and must not be flashed.

That historical prepared image included:

- a reclaiming and resettable 48 KiB CCM allocator with the 80% CCM linker
  gate restored and runtime allocation sealed while active;
- allocation-free `rand()`/`srand()` for the pinned micro-ROS dependency;
- the pinned RMW correction that treats an empty service queue as
  `taken=false`, not a fatal error;
- standard priority-6 HAL circular RX DMA with NDTR/epoch accounting and
  bounded TX DMA at 1,000,000 baud/8N1;
- strict Agent ping validation;
- corrected TIM14 timebase and drift-free periodic releases; and
- absent-battery handling below 4,900 mV with the valid-low-voltage alarm
  restored;
- fixed wheel ownership `front-left=M1`, `front-right=M3`, `rear-left=M2`,
  `rear-right=M4`, with measured per-channel encoder signs while retaining the
  existing JGA27 model method; and
- firmware-owned LED3 heartbeat status plus RGB2 transport status where red
  pulses on UART RX progress, green pulses on UART TX progress, and blue stays
  off.

The six-face capture in
`build/diagnostics/characterization-20260807T114316Z/imu-six-face.tsv` proved
the signed permutation. The corrected locked artifact passed provenance
verification at that source state. It was not flashed and is now superseded.

## Next coding session

Implement the pending default-firmware migration above before performing
another physical flash. Change the default firmware classification and command
paths while retaining the explicit locked recovery and direction-check modes.
Update contracts and tutorials with the code rather than preserving stale
locked-default statements for compatibility.

Run the smallest affected host, firmware-domain, controller, artifact, flash,
runtime, packaging, and documentation checks after each phase. When they all
pass, run the complete regression target once. Do not reuse the historical ELF
hashes as evidence for the migrated source.

## Post-migration hardware resume

Keep every actuator disconnected. Build and flash the new explicit locked
recovery image, verify its mode and read-back, and start it with `start-locked`.
Only that newly built locked artifact may be used to repeat passive board,
transport, IMU, and encoder checks. Require the exact graph, supervisor
`READY`, heartbeat 2 Hz, stable diagnostics, no transport/reset/reconnect
fault, normal IMU samples, and one Agent restart recovery.

Proceed to guarded PID motion only after the new locked image passes the
passive tutorials and every wheel is raised or equivalently guarded, the motor
supply is current-limited, and a physical stop is reachable. Existing locked
evidence does not qualify default PID motion, its gains, or its output range.

## Open physical and release gates

Software completeness is not physical calibration. The following remain
unqualified until the numbered tutorials produce machine-generated hardware
or instrument results:

- powered motor direction, ticks-per-revolution, PID/filter/deadband, current,
  and full operating range (passive encoder ownership/sign is established);
- QMI8658 positive-rotation orientation and extended timing;
- battery-divider/VREFINT scaling and alarm timing;
- PWM, RGB, buzzer, LED, OLED, bus-servo, and button electrical behavior;
- watchdog, USART1, UART5, I2C, and reset/fault behavior;
- physical Agent/USB/MCU recovery timing and stale-replay prevention;
- task stack/resource and escaped-wire-traffic margins; and
- the canonical 500 Hz/60-minute run, three 100-cycle recovery campaigns, and
  24-hour soak.

Tutorial 08's campaign utility intentionally reports
`release_qualification: INCOMPLETE` whenever required physical metrics are
unobserved. A zero software-test exit status does not override that result.

## Current safety boundary before migration

The current normal image remains motor-locked even though all motor-control
code is present. Powered motion is available only in the separately marked
`COMMISSIONING` image and retains command validation, conservative output and
speed caps, independent 200 ms per-motor leases, authorization, and
transport-failure disarming.

Before powered motion, complete Tutorials 01--05, raise or equivalently guard
all wheels, use a current-limited supply, verify a reachable physical stop,
and follow Tutorial 06 exactly. After every commissioning or HIL session,
restore and read back the normal locked firmware.

## Repository transfer note

There is no required Git remote. Commit and transfer source with a trusted
clone, Git bundle, or archive. Do not commit generated firmware dependencies,
build directories, logs, or diagnostic evidence. The ignored legacy snapshot
under `docs/reference/` must be copied separately only if still needed; active
builds do not depend on it.
