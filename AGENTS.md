# RRCLite v2 contributor instructions

Read `README.md` and `docs/NEXT_STEPS.md` before changing this repository.
Treat the framework documents as the detailed contract when a task needs exact
interface, hardware, or safety requirements.

## Current implementation handoff (2026-08-10)

- The ROS workspace/schema migration, single-default-PID firmware migration,
  single eight-tutorial Docker host track, and Python-only launch migration are
  implemented in the current worktree. Preserve them as the current contract.
- `mentor_pi_bringup controller.launch.py` starts the compiled micro-ROS Agent
  and configuration supervisor as one fail-coupled launch inside the hardened
  runtime container; `make start` is the Docker-only entry point.
- The firmware domain has one PID control configuration. Do not restore the
  removed locked/direction-check enum, factories, constants, or control-step
  branches. Invalid configuration values still fail closed.
- Focused documentation, tutorial/runtime, artifact, cache, handoff, and
  deployment tests pass. The native amd64 Docker mock builds micro-ROS,
  firmware, host, and Agent, smokes the enhanced-zsh runtime, and proves
  micro-ROS cache reuse. The RDK X5 path still requires native arm64 and
  hardware evidence. See `docs/NEXT_STEPS.md` for exact evidence and remaining
  Docker/HIL gates.
- Both computer types use architecture-native pinned Docker images for
  firmware, micro-ROS, Agent, host, shell, development, and production. Only
  udev configuration and STM32CubeProgrammer flashing run on the host. Fuzzing
  remains a normal-computer release gate and is intentionally not an RDK gate.
- User-facing ROS shells use the runtime image's pinned Oh My Zsh, completion,
  autosuggestions, and syntax highlighting. Keep Make recipes, scripts, CI,
  builders, and the non-interactive runtime launch on Bash.

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
- Use the root Makefile as the Docker-only developer interface. Do not install
  or source host ROS; the pinned Ubuntu 22.04/Humble images provide it.
- Update all project-owned CMake paths, firmware interface includes, Docker
  inputs, source fingerprints, installers, packaging, CI filters, tests,
  launch/runtime tools, and documentation atomically to use
  `mentor_pi_ros2/src`.
- Keep one complete ordered 01--08 sequence directly under `docs/tutorials/`.
  Tutorial 01 may branch for the arm64 CubeProgrammer package and Tutorial 07
  for the lightweight RDK versus complete normal-computer software gate.

### Manifest validation

- Do not restore the former non-package schema snapshot, `xml-model`
  declarations, schema installation/copying, or schema dependencies in
  fingerprints, deployment, CI, fixtures, and tests.
- Continue validating manifests with `ament_xmllint`, `ament_package`,
  `rosdep`, and `colcon`. Do not replace the removed snapshot with a build-time
  network download.

### Migration acceptance

- Run the smallest focused tests covering changed code. Do not run unrelated
  regression suites merely because they exist.
- Verify the Docker-only Make path and single PID release artifact/handoff.
- Keep path-contract tests rejecting a root package tree or a dependency on
  the removed schema snapshot. Distinguish package-internal, ignored
  legacy-evidence, and upstream third-party `src/` paths.
- Update `README.md`, `docs/NEXT_STEPS.md`, framework contracts, tutorials,
  and repository maps with every remaining default-firmware migration. Record
  actual artifact hashes and test evidence. Do not make PID performance,
  powered motion, or release-qualification claims without recorded HIL
  evidence.

## Supported stack

- Support Ubuntu 22.04 and ROS 2 Humble inside the production image.
  Development hosts may use supported Ubuntu releases on amd64 or arm64, but
  all ROS/build/runtime paths use architecture-native pinned Docker images.
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
