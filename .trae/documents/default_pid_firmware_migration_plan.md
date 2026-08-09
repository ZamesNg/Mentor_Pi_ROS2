# Default PID Firmware Migration Plan

## 1. Summary of the Job

This migration changes the default firmware classification from **motor-locked** to **normal motor-enabled closed-loop PID**. The current state uses `LOCKED` (zero/stop only) as the default; after migration, `PID` with `control_mode=CLOSED_LOOP` becomes the default, while a separate explicit `LOCKED` recovery path is preserved.

### Mode Classification Before/After

| Aspect | Current (Before) | Target (After) |
|---|---|---|
| Default `make firmware` | `LOCKED` / `control_mode=LOCKED` / nonzero motion rejected | `PID` / `control_mode=CLOSED_LOOP` / 6 RPS ceiling, +/-1000 permille |
| Locked recovery path | Implicitly the default (`firmware`, `flash-locked`, `start`) | Explicit new targets: `firmware-locked`, `flash-locked` (kept), `start-locked` |
| Direction-check image | `COMMISSIONING` / DIRECTION_CHECK (0.25 RPS) | Retained separately as `DIRECTION_CHECK` or guarded commissioning path |
| Former commissioning-PID commands | Build separate COMMISSIONING_PID artifact | Alias to normal PID path (no fourth mode) |
| Supervisor/safety/leases | Present | **Preserved unchanged** |

### Migration Mandate (from AGENTS.md §Default PID firmware)

1. `make firmware`, `make flash`, `make start` → build/verify/flash/run the normal motor-enabled **PID** artifact with `control_mode=CLOSED_LOOP`, without commissioning acknowledgements.
2. Preserve supervisor authorization gate, startup inhibition, atomic command validation, model-specific RPS limits, 6 RPS ceiling, +/-1000-permille output limit, independent 198 ms leases, session-loss disarming, transport-failure shutdown.
3. Retain independently classified recovery targets: `firmware-locked`, `flash-locked`, `start-locked`. Retain the guarded direction-check image separately. Cross-mode artifact substitution must fail closed.
4. Retain former commissioning-PID Make commands temporarily as **aliases** to the normal PID path. They must not build or identify a fourth firmware mode.
5. Update framework requirements, safety text, tutorials, metadata, artifact verification, packaging, runtime checks atomically with the code. Existing locked hardware evidence does not qualify the new PID default.

### What Already Passed (from NEXT_STEPS.md)

- ROS workspace/schema refactor is in place: three packages live only under `mentor_pi_ros2/src/`.
- Host build passes with 1,653 tests; source fingerprint is `66b79ca27f608732e7b5be3103ffb6bb53bf5b9ba9348ecf479136cca5b9aaad`.
- Final Docker quality-container stage was blocked by Docker home-state sandbox; rerun when available.
- No STM32 release image or physical hardware result is produced by this refactor.

---

## 2. Repo Research Conclusions

### Current Firmware Modes (from build_firmware.sh, main.cc, CMakeLists.txt)

Modes are selected by two compile-time booleans passed to the STM32 target CMake:

| Build env var | CMake option | main.cc `BuildMotorConfiguration()` | motor_mode | control_mode | nonzero? |
|---|---|---|---|---|---|
| (default) | `COMMISSIONING=OFF` | `LockedMotorControlConfiguration()` | LOCKED | LOCKED | No |
| `RRCLITE_MOTOR_COMMISSIONING=1` | `COMMISSIONING=ON`, `CLOSED_LOOP=OFF` | `CommissioningMotorControlConfiguration() + kDirectionCheck` | COMMISSIONING | DIRECTION_CHECK | Yes, 0.25 RPS |
| + `RRCLITE_MOTOR_COMMISSIONING_CLOSED_LOOP=1` | `COMMISSIONING=ON`, `CLOSED_LOOP=ON` | `CommissioningMotorControlConfiguration() + kClosedLoop + 6 RPS` | COMMISSIONING_PID | CLOSED_LOOP | Yes, 6 RPS |

The new default **PID** should reuse the last configuration but WITHOUT the `MOTORS_RAISED` commissioning ack gate, and with a new mode name, metadata classification, and verification path.

### Current Make Targets

- Default motion-locked: `firmware`, `flash`, `start`, `flash-locked`, `restore-locked`, `passive-check`, `characterize-board`.
- Commissioning (acked): `firmware-commissioning`, `build-commissioning`, `flash-commissioning`, `start-commissioning`.
- Commissioning-PID (acked): `firmware-commissioning-pid`, `build-commissioning-pid`, `flash-commissioning-pid`, `start-commissioning-pid`.

### Files Referencing Modes (20 files touch LOCKED/COMMISSIONING/CONTROL_MODE)

Core build & flash (6): Makefile, tools/build_firmware.sh, tools/flash_firmware.sh, tools/guided_flash.sh, tools/verify_firmware_artifact.sh, tools/check_firmware_memory.sh, firmware target CMake, main.cc.

Runtime (4): tools/run_runtime.sh, tools/tutorial_action.sh, tools/run_runtime_action.sh, tools/characterize_board.sh.

Tests (6): tools/test_firmware_artifact_verification.sh, tools/test_tutorial_actions.sh, tools/test_flash_firmware.sh, tools/test_guided_flash.sh, tools/test_package_board_handoff.sh, firmware controller CMake gate check.

CI/packaging (4): tools/run_firmware_target_ci.sh, tools/run_firmware_target_ci_container.sh, tools/check_firmware_reproducibility.sh, tools/package_board_handoff.sh.

Docs: README.md, docs/NEXT_STEPS.md, docs/framework/{requirements,reliability-and-safety,architecture,development-standards,ros-interface-contract,mentor_pi_hardwares,implementation-roadmap,verified-hardware-profile}.md, docs/tutorials/{02–06,08}.md.

---

## 3. Files and Modules to Be Edited, Grouped by Change Area

### A. Firmware Build System & Mode Compile-Time Logic

Edit to introduce a proper PID default mode and keep LOCKED as a separate explicit build path:

1. `firmware/mentor_pi_mcu/target/stm32/CMakeLists.txt`
   - Replace the two-commissioning-booleans scheme with an explicit three/four-mode selection OR add a new "normal PID" path that sets CLOSED_LOOP without requiring the commissioning ack.
   - Add a new define `MENTOR_PI_MOTOR_DEFAULT_PID` or equivalent.
   - Keep the exact heap audit post-link script unchanged.
   - Keep the `RRCLITE_MOTOR_COMMISSIONING_ACK` gate only for the commissioning/direction-check path.

2. `firmware/mentor_pi_mcu/target/stm32/main.cc`
   - `BuildMotorConfiguration()`: add the `PID` default case that uses `CommissioningMotorControlConfiguration()` with `kClosedLoop`, 6 RPS, 1000 permille output clamp, but **without** treating it as a "commissioning" classification.
   - Keep LOCKED case producing `LockedMotorControlConfiguration()`.
   - Keep direction-check (former COMMISSIONING) case separately.

3. `firmware/mentor_pi_mcu/app/controller/tests/check_stm32_motor_transport_gate.cmake`
   - Update the gate-check expectations so a default PID build is a valid supported configuration.
   - Keep rejecting cross-mode mismatches.

### B. Firmware Build Script and Metadata

4. `tools/build_firmware.sh`
   - Add a new default mode path `PID` (control_mode=CLOSED_LOOP, 6 RPS, 1000 permille) **without** requiring commissioning acks.
   - Rename internal mode variables so the three active classifications are `PID`, `LOCKED`, `DIRECTION_CHECK` (aliased from old `COMMISSIONING`) and the fourth `COMMISSIONING_PID` name maps to PID without building differently.
   - `--print-motor-profile` must emit the new `mode=PID / control_mode=CLOSED_LOOP / nonzero_motion_enabled=1 / maximum_accepted_rps=6.0 / output_limit_permille=1000` for the default.
   - Metadata `rrclite-build-metadata.txt` must classify `motor_mode=PID`, `control_mode=CLOSED_LOOP`, `commissioning_ack=` (empty), for the default; `LOCKED` remains exactly as today; direction-check retains `motor_mode=DIRECTION_CHECK` (or keeps COMMISSIONING name only if every verifier accepts it).
   - The four CMake invocations (local cmake + docker) must pass the new mode selection, not just the two booleans. Or keep the two booleans and add a default-PID boolean/tristate so `CONTROL_MODE=CLOSED_LOOP` can be set without `COMMISSIONING_ACK`.

### C. Artifact Verification and Memory Gates

5. `tools/verify_firmware_artifact.sh`
   - Accept new mode enum: `LOCKED | PID | DIRECTION_CHECK` (continue also accepting `COMMISSIONING` → treat as `DIRECTION_CHECK`; accept `COMMISSIONING_PID` → treat as `PID`, to support aliases).
   - Add a branch for `PID` metadata checks: `motor_mode=PID`, `control_mode=CLOSED_LOOP`, empty `commissioning_ack`, CMake cache for the new default-PID flag, no `RRCLITE_MOTOR_COMMISSIONING:BOOL=ON`.
   - Keep existing `LOCKED` and commissioning/direction branches.
   - Keep the source/interface/micro-ROS/ELF hash contract unchanged across all modes.
   - Cross-mode verification must fail closed (e.g. requesting LOCKED against a PID artifact must fail with the exact mode mismatch text).

6. `tools/check_firmware_memory.sh`
   - Accept `PID` and `DIRECTION_CHECK` alongside `LOCKED`.
   - Call the verifier with the matching mode.
   - Budget gates (80% max flash/SRAM/CCM) apply identically to every mode.

### D. Root Makefile Interface

7. `Makefile`
   - `firmware` → build PID (default).
   - NEW target `firmware-locked` → explicitly builds LOCKED.
   - NEW aliases: `firmware-commissioning-pid` → aliases to `firmware` (PID), `build-commissioning-pid` → aliases to tutorial `firmware` action path so it does not create a fourth artifact. They may still print a deprecation/alias notice but must NOT build a fourth mode.
   - Keep `firmware-commissioning` / `build-commissioning` → direction-check path (still requires MOTORS_RAISED ack).
   - `flash` → low-level flash target uses PID mode classification now instead of LOCKED. Currently it passes literal `LOCKED` to `flash_firmware.sh`; change default to `PID`.
   - `flash-locked` → keep; it flashes LOCKED via `guided_flash.sh LOCKED`.
   - NEW alias `flash-commissioning-pid` → flash PID mode, not a fourth artifact (may reuse `guided_flash.sh PID` semantics).
   - Keep `flash-commissioning` → flashes direction-check/commissioning path.
   - `start` → tutorial_action `start` now expects PID firmware and its ack.
   - NEW target `start-locked` → explicitly for LOCKED firmware.
   - `start-commissioning-pid` → alias to normal PID start path (no separate classification).
   - Keep `start-commissioning` → direction-check start.
   - `restore-locked` currently depends on `firmware` and flashes `LOCKED`. Change to depend on `firmware-locked` instead so it still restores a true locked image.
   - Update `help` descriptions for every changed target.
   - Add new ack defaults/variables if PID needs a different default runtime acknowledgement.

### E. Flash Scripts

8. `tools/guided_flash.sh`
   - Mode usage: accept `LOCKED | PID | DIRECTION_CHECK` (plus backward-compat alias `COMMISSIONING→DIRECTION_CHECK`, `COMMISSIONING_PID→PID`).
   - Only commissioning/direction flash demands the second ack `MOTORS_RAISED_CURRENT_LIMITED`. PID flash (the default) uses only the ROM-bootloader disconnect ack, the same as today for locked flash.
   - Mode dispatch to `flash_firmware.sh` uses the normalized canonical mode name.

9. `tools/flash_firmware.sh`
   - Same mode enum as guided_flash.
   - Second ack only required for direction-check/commissioning mode, not PID.
   - Artifact verifier call uses the canonical mode.
   - Snapshot metadata re-checking remains strict against the same canonical mode.

### F. Runtime and Tutorial Actions

10. `tools/run_runtime.sh`
   - `--firmware-mode` accepts `LOCKED | PID | COMMISSIONING | COMMISSIONING_PID` (normalizes `COMMISSIONING_PID→PID` internally if desired; the exact wire interface remains unchanged).
   - The default `LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED` ack currently maps to firmware-mode=LOCKED. Introduce a PID-appropriate ack. Options:
     - (a) reuse the same ack string if PID safety text is unchanged (actuators disconnected at normal start); or
     - (b) add a new `PID_FIRMWARE_SAFETY_ACK` per AGENTS rule. Follow the existing convention: PID runtime still demands actuators disconnected or guarded, so (a) is the minimal fail-closed baseline; use (a) unless the tutorial text differentiates.
   - Launch: `start-hardware`, `start-mecanum`, `start-ackermann` currently forward `--firmware-mode LOCKED`. They forward `--firmware-mode PID` after migration (still with actuator-disconnect ack until HIL qualifies motion).

11. `tools/tutorial_action.sh`
   - `start` case: change required ack to map to PID firmware-mode (default ack still disconnects actuators) and invoke run_runtime with `--firmware-mode PID`.
   - NEW case `start-locked`: existing locked behavior: `--firmware-mode LOCKED`, ack LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED.
   - `start-commissioning-pid`: alias to `start` or execute the same command path, not a fourth classification. Keep `start-commissioning` for direction-check mode.
   - `build-commissioning-pid` case: exec `make firmware` (the PID path) with the safety prompt present as an operational guard even though the build no longer enforces the ack. This preserves the operator flow.
   - `characterize-board` case: calls `verify_firmware_artifact.sh LOCKED` — change it to `verify_firmware_artifact.sh PID` **OR** require the operator to build firmware-locked first, per NEXT_STEPS §Post-migration hardware resume guidance (passive checks should run against the new LOCKED recovery image). Per AGENTS.md / NEXT_STEPS: "Keep every actuator disconnected. Build and flash the new explicit locked recovery image … repeat passive board, transport, IMU, and encoder checks." So characterize-board and passive-check must run against the locked recovery. Update the verify call to remain LOCKED and add guidance in the prompt that an explicit `firmware-locked` build is needed first. Passive-check similarly stays on locked recovery until guarded PID motion is authorized by Tutorial 06.
   - `release-software-gates`: call `verify_firmware_artifact.sh PID` (or verify PID + LOCKED independently — the AGENTS requirement is to verify them independently). At minimum verify the default; better verify both.

12. `tools/run_runtime_action.sh`
   - Check if any controller-readiness / service-availability path branches on firmware mode. If it only checks endpoint presence it needs no change.
   - If it enforces a "locked-zero-targets" readiness contract, relax that for PID mode while keeping the same graph/supervisor/heartbeat baseline.

### G. Tests — Update Fixtures and Mode Enums

13. `tools/test_firmware_artifact_verification.sh`
   - Add a PID-mode fixture base analogous to the existing COMMISSIONING_PID base but with empty `commissioning_ack`, new cache flag for default-PID, `motor_mode=PID`, `control_mode=CLOSED_LOOP`.
   - Add cross-mode substitution failures: request LOCKED against PID fixture, request DIRECTION_CHECK against PID fixture, request PID against LOCKED fixture.
   - Keep LOCKED/COMMISSIONING/COMMISSIONING_PID (now legacy) branches passing.
   - Expect the new `INVALID` case to still fail with updated usage text.

14. `tools/test_tutorial_actions.sh`
   - Update the Makefile-target-presence grep to include the new required targets: `firmware-locked`, `start-locked`.
   - Verify that `firmware-commissioning-pid`/`start-commissioning-pid` are still present as aliases (grep for them in Makefile; exact semantics by functional test below).
   - Update any `--firmware-mode COMMISSIONING_PID` greps: still present after migration; ensure PID mode forwarding line exists too.
   - Add an interactive-failure expectation for the new `start-locked` action.

15. `tools/test_flash_firmware.sh` and `tools/test_guided_flash.sh`
   - Extend usage/validation cases to `PID` canonical mode, alias `COMMISSIONING_PID→PID`, and verify the commissioning ack is NOT required for PID flash (only for direction-check/commissioning).

16. `tools/test_package_board_handoff.sh`
   - Update any references to the default firmware build mode so handoff packaging produces/verifies PID artifacts.
   - Keep LOCKED handoff path as an explicit variant.

### H. CI, Reproducibility, Packaging

17. `tools/run_firmware_target_ci.sh` and `tools/run_firmware_target_ci_container.sh`
   - Build and test both the new default PID and the locked recovery independently.
   - Also keep/verify the direction-check build.
   - Artifact verification calls must match the built mode.

18. `tools/check_firmware_reproducibility.sh`
   - Default reproducibility target is now PID, not LOCKED.
   - Optionally add locked reproducibility as a separate CI step.

19. `tools/package_board_handoff.sh`
   - Classify handoff artifact as PID. Update manifest/hash files accordingly.
   - If a separate locked handoff is needed, it must be produced independently.

20. `tools/characterize_board.sh`
   - If it hardcodes LOCKED expectations, keep LOCKED (characterization is post-locked-flash per Tutorial 05). No PID changes here unless the script also performs active motion.

### I. Workspace-Source Paths (Already Migrated; Audit Only)

- CMake/CI/package paths referencing the workspace root: `firmware/mentor_pi_mcu/CMakeLists.txt` line 17–28 already points to `../../mentor_pi_ros2/src/mentor_pi_interfaces/include` — keep unchanged. Target STM32 include path line 423 does the same. No additional workspace changes are required inside this migration.

### J. Framework Contract and Safety Documentation

21. `docs/framework/requirements.md`
   - SAFE-006: re-word so the normal firmware build is PID/CLOSED_LOOP with preserved safety caps, not a motor lock. The locked recovery path is the explicit LOCKED target, not the default. Direction-check remains a separately classified commissioning image.
   - TRN-004, RT-004, REC-003 references to "commissioning image" as the only nonzero-bearing image: replace with "default PID image plus the separately acknowledged commissioning direction-check image". Or equivalent wording that preserves the fail-closed contract.

22. `docs/framework/reliability-and-safety.md`
   - Lines around 42–92 currently describe "normal firmware image is motor-locked" and nonzero arming only in commissioning. Rewrite to normal PID default with the same limits, and describe the explicit LOCKED recovery target. Keep the direction-check section distinct.
   - Keep lease/validation/disarming sections identical; just update mode references.
   - Fault table row 427 mentions "normal lock or commissioning cap": update to "PID limits, explicit locked lock, or commissioning cap".

23. `docs/framework/architecture.md`
   - Lines around 99–112 describing pre-HIL nonzero allowance: now the default PID image nonzero targets are allowed but not release-qualified. The explicit locked image remains for passive bring-up.
   - The commissioning-build wrapper still enforces MOTORS_RAISED only for direction-check.

24. `docs/framework/development-standards.md`
   - Lines 175–181 and 220+: CI/release builds default to PID now; an explicit locked build is a separate verified target; commissioning build wrapper is only for direction-check.

25. `docs/framework/ros-interface-contract.md`
   - Line 199–201, 249, 279, 287, 325: update "normal motor-locked" to "normal PID"; nonzero motion in commissioning direction-check remains separately guarded.

26. `docs/framework/mentor_pi_hardwares.md`
   - Line 97 area: the normal PID firmware is the default for ros2_control hardwares; still warn that PID default is not HIL-qualified.

27. `docs/framework/implementation-roadmap.md`
   - Lines 152, 173–182: D3 default motor lock → now explicit locked target, with PID as default unqualified motion.

28. `docs/framework/verified-hardware-profile.md`
   - Lines 41–43: "locked image cannot energize motor bridge" remains true for locked recovery image. Add that PID default image can energize motor bridges but is not release-qualified.

### K. Top-level Docs

29. `README.md`
   - Tutorial table line 02: `make flash-locked` stays (now the explicit locked recovery). Add step for default PID flash via `make flash` if the tutorial order changes — or keep Tutorial 02 on locked recovery per NEXT_STEPS post-migration hardware resume.
   - Tutorial line 03 "Build and run the Humble host" `make start`: now runs against PID firmware. If tutorials still begin on locked recovery for safety passive checks, add a `start-locked` column note or adjust.
   - "Safety and current status" section (lines 43–55): "default firmware is motor-locked" → default firmware is PID with CLOSED_LOOP; locked recovery path exists separately; explicit locked image remains recommended for Tutorials 02/04/05 passive checks. No HIL-qualified powered motion claim.

30. `docs/NEXT_STEPS.md`
   - Update §Pending next-version firmware migration to mark this migration **IMPLEMENTED**.
   - Record actual PID/LOCKED/direction-check artifact ELF SHA-256 hashes once built during verification.
   - Record test evidence (focused groups passed, full regression passed, which Docker quality stage ran).
   - Update §Current safety boundary before migration to the new post-migration safety boundary.
   - Update §Post-migration hardware resume and §Open physical and release gates accordingly. Do not mark release qualification or PID performance claims done.
   - Update §Historical prepared firmware: mark old binaries superseded, add the new migrated PID + locked candidate entries with current build timestamps/hashes (filled after build).
   - Update §Next coding session to point past this migration.

### L. Tutorials (02 through 08; 09)

31. `docs/tutorials/02-build-and-flash-locked-firmware.md`
   - Title/commands now refer to the explicit locked-recovery image.
   - Commands are `make firmware-locked` then `make flash-locked` (previously `make firmware`).
   - Safety: keep actuators disconnected text; add explicit note that default `make firmware` now builds motor-enabled PID and must not be used here.

32. `docs/tutorials/03-build-and-run-humble-host.md`
   - `make start` now expects PID firmware by default.
   - If starting against locked recovery instead, use `make start-locked`.
   - Align with tutorial sequence (passive bring-up on locked, then PID once guarded).

33. `docs/tutorials/04-run-passive-board-bringup.md`
   - Runs on locked recovery image per NEXT_STEPS §Post-migration hardware resume. Mention `make start-locked`.
   - Do not command nonzero motors.

34. `docs/tutorials/05-characterize-board-hardware.md`
   - Explicit locked recovery. Commands: `make firmware-locked`, `make flash-locked`, `make start-locked`, `make characterize-board`.

35. `docs/tutorials/06-commission-one-motor-safely.md`
   - Guarded first powered motion.
   - The former direction-check phase (250 permille, 0.25 RPS) is the commissioning image path.
   - The former closed-loop commissioning phase now maps to the default PID image (no separate build; optionally run on PID with the same safeguards).
   - Keep exact raised-wheels/current-limited/physical-stop-reachable text.
   - Update build commands if they previously pointed at commissioning-pid only.

36. `docs/tutorials/07-qualify-hardware-and-recovery.md`
   - State that HIL runs against default PID firmware for motion tests and against explicit locked recovery for passive baseline.

37. `docs/tutorials/08-run-stress-soak-and-release-gates.md`
   - Campaign evidence labels PID vs LOCKED artifacts with their independent hashes.
   - Keep `release_qualification: INCOMPLETE` semantics for missing physical metrics unchanged.

38. `docs/tutorials/09-run-mentor-pi-hardwares.md`
   - Default ros2_control hardwares path is PID firmware; preserve the not-HIL-qualified warnings.

---

## 4. Steps for Modifications / Implementation Order

The migration must be done atomically (or in tightly coupled groups) to avoid mismatched mode references between build, flash, runtime, docs, tests.

### Phase 1 — Firmware Mode Classification Core

1. Modify `target/stm32/CMakeLists.txt` to add a default-PID compile path that produces CLOSED_LOOP without the commissioning ack gate, keeps LOCKED explicitly, and keeps commissioning/direction-check gated.
2. Modify `main.cc BuildMotorConfiguration()` for the three-way classification: PID (CLOSED_LOOP), LOCKED (LockedMotorControlConfiguration), DIRECTION_CHECK (Commissioning direction bypass).
3. Modify `build_firmware.sh` to emit the new modes and metadata classifications, mapping COMMISSIONING_PID input to PID output so it does not build a fourth artifact.
4. Update the controller transport-gate check in `check_stm32_motor_transport_gate.cmake`.

### Phase 2 — Artifact Verification and Memory

5. Rewrite `verify_firmware_artifact.sh` mode switch: accept `LOCKED | PID | DIRECTION_CHECK` with backward-compat aliases `{COMMISSIONING→DIRECTION_CHECK, COMMISSIONING_PID→PID}`. Enforce cross-mode mismatch.
6. Update `check_firmware_memory.sh` to accept the new mode enum.

### Phase 3 — Makefile + Flash Scripts

7. Update `Makefile` targets: new defaults, new firmware-locked/start-locked, aliases for commissioning-pid commands, restore-locked→firmware-locked, updated help.
8. Update `guided_flash.sh` and `flash_firmware.sh` mode enums and ack gating (commissioning ack only for direction-check).

### Phase 4 — Runtime and Tutorial Actions

9. Update `run_runtime.sh` firmware-mode enum and defaults.
10. Update `tutorial_action.sh`: `start`→PID, add `start-locked`, alias `start-commissioning-pid`→PID path, characterize-board stays on LOCKED verification, release-software-gates verifies both modes.
11. Audit `run_runtime_action.sh` for mode-dependent logic; minimally adjust.

### Phase 5 — Tests

12. Update each test fixture file in §G to add PID fixtures, locked-recovery target presence checks, cross-mode substitution failures, and alias behavior.

### Phase 6 — CI, Reproducibility, Packaging

13. Extend CI scripts (§H) to build and independently verify PID and LOCKED, plus direction-check.
14. Update reproducibility checker and handoff package default mode to PID.

### Phase 7 — Documentation

15. Edit framework docs (§J): requirements, safety, architecture, dev-standards, interface contract, hardwares, roadmap, verified profile.
16. Edit top-level docs (§K): README and NEXT_STEPS.
17. Edit tutorials (§L): 02–09, matching the command changes.

### Phase 8 — Verification & Recording Evidence

18. Run focused tests per affected group: artifact verifier tests, tutorial action tests, firmware-domain tests, controller tests, host build tests, CI tool tests.
19. Build each of the three modes (PID, LOCKED, DIRECTION_CHECK) via the make targets, verify artifacts independently, and confirm cross-mode verification fails closed.
20. Run the full regression target (`make test`) once. Record actual ELF hashes into NEXT_STEPS.md.
21. Record pass/fail for Docker quality stage explicitly. Do not infer hardware qualification from software-only evidence.

---

## 5. Potential Dependencies or Considerations

- **Docker environment**: Firmware builds require Docker (or `RRCLITE_BUILD_LOCAL=1` with the exact Arm GCC 13.2.1 + cmake + ninja). Use the default Docker adaptive path.
- **micro-ROS library**: must be up-to-date (`make setup` / `./tools/build_microros_library.sh`) before any firmware build; fingerprint hash pins in config are verified; if they change during source editing, rebuild micro-ROS first.
- **STM32CubeProgrammer**: Required only for flash/guided_flash tests; software-only artifact tests don't need hardware.
- **Ubuntu 22.04 native**: preferred for host regression; otherwise it uses pinned Docker Humble (matches the AGENTS rule).
- **Hardware claims**: Explicitly disallow PID performance or release-qualification claims in documentation because no HIL evidence is produced by the software migration.
- **Cross-mode substitution failure**: Must be verified manually in both directions between PID and LOCKED.
- **Commissioning-PID alias idempotence**: The old `make firmware-commissioning-pid`/`start-commissioning-pid` commands should not silently produce a legacy artifact. Verify they route to the PID path without setting a distinct `motor_mode`.
- **No schema restoration**: Never re-add the removed package schema snapshot or xml-model declarations anywhere in CI/tests/docs/fingerprints.

---

## 6. Risk Handling

| Risk | Mitigation |
|---|---|
| Default PID allows nonzero motion but no one changed runtime ack text → operator runs actuators connected unattended | Keep LOCKED_FIRMWARE_ACTUATORS_DISCONNECTED-level ack for `make start` too; tutorials strongly direct operators to Tutorial 06 before any powered motion; update README safety section. |
| Cross-mode flash mismatch (PID flashed over expected LOCKED, or vice versa) | `verify_firmware_artifact.sh` mode contract + snapshot mode re-check in `flash_firmware.sh` are enforced; add tests. |
| LOCKED recovery accidentally depends on the default PID make target and disappears | Introduce explicit `firmware-locked` build target independent of `firmware`; wire `restore-locked` to it; verify in tests. |
| Commissioning-PID alias leaks a fourth mode | Metadata classification in `build_firmware.sh` must write the same PID metadata whether invoked from `firmware` or from `firmware-commissioning-pid`; verify in a focused test by both paths and comparing metadata hash. |
| Build invokes wrong CMake flags for a mode | Add explicit metadata assertions in `verify_firmware_artifact.sh` for the CMake cache entries per mode. |
| Hasty docs claim HIL-qualified PID motion | All documentation edits preserve the "not HIL-qualified / release_qualified=0" baseline; NEXT_STEPS is updated with software-only evidence. |
| Full regression takes too long on first pass | Run focused tests per group first (firmware domain, controller, host build, mode verifier, tutorial actions, flash scripts, container host test). Only then run `make test`. |
| Docker Buildx home-state failure blocks quality-container stage | Record as open in NEXT_STEPS if it reproduces; do not gate the migration on the unavailable Docker home state. |
| Stale build root interferes with mode switches | `build_firmware.sh` already does `RemoveBuildRoot`; keep it; never manually reuse a `build/stm32` directory across modes without rebuild. |
| Changed source invalidates old fingerprint hashes in NEXT_STEPS historical section | Update historical rows with new hashes after migration build, marking them migrated/superseded; keep old locked hashes as historical evidence only. |

---

## 7. What Is and Is Not Tested by the Software-Only Phase

**Tested by focused groups + full regression:**
- All three modes build (default PID, explicit LOCKED, direction-check).
- Metadata classification per mode and independent artifact verification.
- Cross-mode substitution fails closed (flash, verify, runtime).
- Commissioning-PID alias routes to PID without a fourth artifact.
- Host (mentor_pi_ros2) packages build/test via colcon Docker.
- Controller, domain, concurrency, watchdog-retention, DMA, peripheral, timebase CMake contract tests on the native suite.
- Fuzz smoke runs (validation, bus-servo, RX ring, etc.) if the host suite runs them.
- Shell-syntax, whitespace, and active-build-policy checks.
- Workspace layout/schema tests remain passing (no regression of earlier migration).

**Explicitly NOT tested in software-only phase:**
- Flashing to a physical STM32.
- Runtime graph creation, heartbeat, IMU, encoder samples on hardware.
- Powered motor direction, PID gain accuracy, current, full operating range.
- QMI8658 positive rotation axes.
- Battery divider scaling, alarm timing.
- PWM/RGB/buzzer/LED/OLED/bus-servo/button electrical behavior.
- Watchdog, USART1, UART5, I2C, reset/fault behavior.
- USB/MCU/Agent recovery timing, stale-replay prevention.
- Task stack margins, escaped-wire-traffic margins.
- 500 Hz / 60-minute run, three 100-cycle recovery campaigns, 24-hour soak.
- Any release qualification state.

Physical gates remain open and tracked in `docs/NEXT_STEPS.md §Open physical and release gates`; no wording in this migration may close them.
