# RRCLite v2 contributor instructions

Read `README.md` and `docs/NEXT_STEPS.md` before changing this repository.
Treat the framework documents as the detailed contract for public interfaces,
hardware, safety, and qualification.

## Current implementation handoff (2026-08-11)

- Preserve the one-repository, three-component layout: `firmware/`,
  `micro_ros_agent/`, and `ros2_ws/`. Do not add Git submodules or compatibility
  copies of the former workspace.
- Ubuntu 22.04 amd64/arm64 is the only native build/test and production
  platform. macOS and other Linux distributions use the VS Code Dev Container
  for all component build/test work.
- `.devcontainer/` is the only Docker path. It is development-only and cannot
  be used onboard, for systemd installation, serial runtime, flashing, HIL, or
  release evidence. Do not restore production images, Docker runtime, OCI,
  QEMU, or container handoffs.
- The Agent is a separately installed non-root systemd service and the only
  serial owner. It must not start ROS applications. Applications start manually
  and keep their supervisor/hardware/tracker launch fail-coupling.
- Tutorials are two complete ordered tracks under `docs/tutorials/host/` and
  `docs/tutorials/onboard/`. Production handoffs include only `onboard/`.

## Component boundaries

- CMake/Ninja is authoritative for firmware. Build, verify, package, and flash
  only the `NORMAL_CLOSED_LOOP_DEFAULT` ADRC artifact. Do not restore removed
  commissioning modes, direction-check branches, PlatformIO, or another build
  graph.
- Firmware consumes the checked Humble SDK under
  `firmware/mentor_pi_mcu/sdk/humble/`, never ROS workspace build output.
  `ros2_ws/src/mentor_pi_interfaces` is canonical; interface changes must
  regenerate the SDK in the same commit and update its hashes.
- The Agent owns its source lock, patch, build tree, metadata, service, and udev
  rule below `micro_ros_agent/`. Preserve pinned revisions and the CH9102F
  DTR/RTS patch.
- Colcon in `ros2_ws/` discovers only its five packages. Preserve package names
  and package-internal C++ `src/` directories. External pins belong in
  `dependencies.repos` and project patches belong in `ros2_ws/patches/`.
- Keep the root Makefile limited to onboarding, integration, passive hardware,
  characterization, evidence, and qualification actions.

## Compatibility and safety

- Preserve `mentor_pi_interfaces`, public topics,
  services, QoS, units, limits, leases, and authorization behavior unless the
  user explicitly authorizes an interface change.
- Preserve startup inhibition, the single-supervisor publisher check,
  generation/session matching, model-specific RPS limits, 6 RPS ceiling,
  ±1000-permille output limit, independent 198 ms leases, session-loss
  disarming, and transport-failure shutdown.
- Invalid motor commands are atomic and do not refresh leases. PWM and bus
  servos hold their last accepted state on host loss.
- Powered motor work requires passive encoder-direction checks, raised or
  equivalently guarded wheels, a current-limited supply, and the documented
  acknowledgements. Complete Tutorials 01–05 passively before guarded work.
- Never claim ADRC performance, powered motion, endurance, or release
  qualification without recorded and reviewed HIL/instrument evidence.

## Manifest and repository hygiene

- Continue package validation with `ament_xmllint`, `ament_package`, `rosdep`,
  and colcon. Do not restore the removed non-package schema snapshot or a
  build-time schema download.
- Generated micro-ROS files, downloaded firmware/Agent dependencies, build
  trees, logs, and `.pio/` remnants remain ignored and uncommitted.
- `firmware/mentor_pi_mcu/third_party/` and Agent upstream sources are
  disposable pinned checkouts, not root submodules.
- Except for its policy README, `docs/reference/` is ignored legacy evidence.
  Never run broad `git clean -fdX`; active builds must not depend on it.
- Preserve unrelated user changes. Do not create a remote or push unless asked.

## Verification discipline

- Run the smallest focused tests covering each change, then expand in
  proportion to risk. Report what was and was not tested.
- Do not start emulation, long soak/stress, or architecture-matrix runs unless
  explicitly requested. Prefer the native architecture.
- Before a release checkpoint verify formatting, documentation, artifact/SDK
  provenance, firmware memory headroom, and relevant native component builds.
- Hardware behavior requires recorded HIL evidence; never infer it from mocks,
  native unit tests, or the Dev Container.
