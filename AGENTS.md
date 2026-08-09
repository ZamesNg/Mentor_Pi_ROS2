# RRCLite v2 contributor instructions

Read `README.md` and `docs/NEXT_STEPS.md` before changing this repository.
Treat the framework documents as the detailed contract when a task needs exact
interface, hardware, or safety requirements.

## Current implementation handoff (2026-08-09)

- The ROS workspace/schema migration, single-default-PID firmware migration,
  two complete eight-tutorial host tracks, and Python-only launch migration are
  implemented in the current worktree. Preserve them as the current contract.
- `mentor_pi_bringup controller.launch.py` starts the compiled micro-ROS Agent
  and configuration supervisor as one fail-coupled launch. Direct native use
  performs the acknowledgement, serial identity/ownership, executable, and
  development-artifact preflight; `make start` remains the adaptive
  native-or-Docker entry point.
- The firmware domain has one PID control configuration. Do not restore the
  removed locked/direction-check enum, factories, constants, or control-step
  branches. Invalid configuration values still fail closed.
- Focused documentation, tutorial/runtime, artifact, flash, handoff, domain,
  and controller tests pass. The pinned quality-container rerun now reaches
  three pre-existing formatting failures in unchanged C++ sources; the new
  native RDK X5 build/runtime path still requires arm64 Ubuntu 22.04 evidence.
  See `docs/NEXT_STEPS.md` for exact evidence and remaining native/HIL gates.
- The RDK X5 onboard-computer path is Ubuntu 22.04 arm64, Docker-free, and uses
  native Humble, conventional colcon, direct ros2 commands, native micro-ROS
  generation, and the pinned local Arm GNU 13.2.1 toolchain. The normal-computer
  path is Ubuntu 24.04 and uses the pinned Humble containers. Fuzzing remains a
  normal-computer release gate and is intentionally not an onboard gate.

## Next-version migration mandate

The user-authorized migration is implemented. The requirements below describe
the current required layout and behavior and must remain enforced.

### Default PID firmware

- `make firmware`, `make flash`, and `make start` build, verify, flash, and run
  the normal motor-enabled `PID` artifact with `control_mode=CLOSED_LOOP`. No
  commissioning classification, mode aliases, or multi-mode branching is
  required or supported.
- Preserve the configuration supervisor and its current authorization gate,
  startup inhibition, atomic command validation, model-specific RPS limits,
  the 6 RPS implementation ceiling, the +/-1000-permille output limit,
  independent 198 ms leases, session-loss disarming, and transport-failure
  shutdown.
- Build, verify, package, and run only one firmware artifact classification:
  `NORMAL_CLOSED_LOOP_DEFAULT` (PID release). Handoff packaging produces a
  single `firmware-pid-release/` layout with one ELF/Hex/Bin/Map set plus
  BUILD-METADATA and BUILD-MODE.
- Update framework requirements, safety text, tutorials, metadata, artifact
  verification, packaging, and runtime checks together. Powered motion and
  release-qualification claims require recorded HIL evidence from the
  numbered tutorial sequence.

### Standard ROS 2 workspace

- Keep the three ROS packages in the directly buildable Humble workspace:

  ```text
  mentor_pi_ros2/
    src/
      mentor_pi_interfaces/
      mentor_pi_bringup/
      mentor_pi_hardwares/
  ```

- Do not add a root compatibility symlink, duplicate package tree, or stale
  root-source fallback. Package-internal `src/` directories containing C++
  implementation files remain unchanged.
- On Ubuntu 22.04 with ROS 2 Humble, support conventional direct use from
  `mentor_pi_ros2` with `rosdep`, `colcon build`, and `colcon test`. Keep the
  root Makefile as the verified convenience interface and keep the pinned
  Ubuntu 22.04/Humble Docker path for every other supported Ubuntu release.
- Update all project-owned CMake paths, firmware interface includes, Docker
  inputs, source fingerprints, installers, packaging, CI filters, tests,
  launch/runtime tools, and documentation atomically to use
  `mentor_pi_ros2/src`.
- Keep the two complete ordered tutorial trees under
  `docs/tutorials/{onboard-computer,normal-computer}`. The onboard tree uses
  native commands and the normal-computer tree uses Docker; preserve identical
  safety ordering and hardware gates in both.

### Manifest validation

- Do not restore the former non-package schema snapshot, `xml-model`
  declarations, schema installation/copying, or schema dependencies in
  fingerprints, deployment, CI, fixtures, and tests.
- Continue validating manifests with `ament_xmllint`, `ament_package`,
  `rosdep`, and `colcon`. Do not replace the removed snapshot with a build-time
  network download.

### Migration acceptance

- Run focused tests while migrating, then run the complete regression target
  once after every focused group passes.
- Verify direct native Humble colcon use and the root adaptive Make/Docker
  path. Verify the single PID release artifact and its handoff layout.
- Keep path-contract tests rejecting a root package tree or a dependency on
  the removed schema snapshot. Distinguish package-internal, ignored
  legacy-evidence, and upstream third-party `src/` paths.
- Update `README.md`, `docs/NEXT_STEPS.md`, framework contracts, tutorials,
  and repository maps with every remaining default-firmware migration. Record
  actual artifact hashes and test evidence. Do not make PID performance,
  powered motion, or release-qualification claims without recorded HIL
  evidence.

## Supported stack

- Support only Ubuntu 22.04 and ROS 2 Humble for production. Development may
  use any Ubuntu release on amd64 or arm64: Ubuntu 22.04 uses native Humble;
  every other release uses the pinned Ubuntu 22.04/Humble Docker runtime and
  must not receive a native ROS installation.
- Use the root `Makefile` as the developer interface. CMake/Ninja is
  authoritative for firmware and `colcon` is authoritative for host packages.
- Do not restore PlatformIO or add a second IDE build graph. Do not introduce
  ROS 2 Jazzy into an active build, test, flash, or runtime path.
- Project-owned runtime code is C++17 and follows Google C++ Style. Python may
  be used by upstream ROS/build tools, but never in the project runtime path.

## Compatibility and safety

- Preserve `mentor_pi_interfaces`, `/mentor_pi/controller`, topic and service
  names, QoS, units, limits, and safety behavior unless the user explicitly
  authorizes a public-interface change.
- Invalid motor commands must remain atomic and must not refresh leases. PWM
  and bus servos hold their last accepted state on host loss.
- Powered motor motion requires passive encoder-direction checks, raised or
  equivalently guarded wheels, a current-limited supply, and the documented
  build and flash acknowledgements. Complete Tutorials 01--05 passively
  before any guarded powered work.

## Repository hygiene

- Build outputs, generated micro-ROS files, downloaded firmware dependencies,
  logs, and `.pio/` remnants are ignored and must not be committed.
- `firmware/mentor_pi_mcu/third_party/` contains disposable standalone Git
  checkouts at pinned commits. Keep them ignored and managed by `make setup`;
  do not convert them to root-repository submodules.
- Except for its tracked policy README, `docs/reference/` is a separately
  preserved, ignored legacy-evidence snapshot. Active builds must not depend
  on it. Never run a broad
  `git clean -fdX`, because that can erase this non-reproducible local copy.
- Preserve unrelated user changes. Do not create a remote or push unless the
  user asks.

## Verification discipline

- Run the smallest focused tests that cover each change, then expand testing
  in proportion to risk. Report what was and was not tested.
- Do not start long amd64 emulation, soak, stress, or full architecture-matrix
  runs unless the user explicitly asks. Prefer the native architecture.
- Before a release checkpoint, verify formatting, documentation, artifact
  provenance, firmware memory headroom, and the relevant host/firmware build.
- Hardware claims require recorded HIL evidence; do not infer them from mocks
  or native tests.
