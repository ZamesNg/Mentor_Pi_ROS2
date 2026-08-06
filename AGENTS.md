# RRCLite v2 contributor instructions

Read `README.md` and `docs/NEXT_STEPS.md` before changing this repository.
Treat the framework documents as the detailed contract when a task needs exact
interface, hardware, or safety requirements.

## Supported stack

- Support only Ubuntu 22.04 and ROS 2 Humble for production. Ubuntu 24.04 is a
  Docker development host and must not receive a native ROS installation.
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
- The normal firmware must remain motor-locked. Never weaken artifact checks,
  flash acknowledgements, command validation, the independent 200 ms motor
  leases, or transport-failure motor disarming.
- Invalid motor commands must remain atomic and must not refresh leases. PWM
  and bus servos hold their last accepted state on host loss.
- Commissioning motion requires passive encoder-direction checks, raised or
  equivalently guarded wheels, a current-limited supply, and the documented
  build and flash acknowledgements.

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
