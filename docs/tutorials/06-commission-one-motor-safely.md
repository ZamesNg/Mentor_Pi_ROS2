# Tutorial 06: Commission One Motor Safely

Build and run the non-release capped firmware for one guarded motor, then
restore the locked image.

**Run on:** the connected native-or-Docker Humble host
**Hardware state:** all wheels raised; current-limited motor supply; physical
power stop reachable

Previous: [Tutorial 05: Characterize Board Hardware](05-characterize-board-hardware.md)
Next: [Tutorial 07: Qualify Hardware and Recovery](07-qualify-hardware-and-recovery.md)

## 1. Build and flash commissioning firmware

**Warning:** Passive encoder direction must already be unambiguous. Disconnect
motor power before building or flashing; leave only one reviewed channel wired.

### 1.1 Direction-check stage

```sh
cd /home/zames/Mentor_Pi && make build-commissioning
```

Type `MOTORS_RAISED`. Expected artifact properties are `COMMISSIONING`,
`release_qualified=0`, `control_mode=DIRECTION_CHECK`, fixed output
`250 permille`, maximum admitted command `0.25 RPS`, and a measured-speed stop
at `0.50 RPS` (30 RPM). PID is not used in this stage.

### 1.2 PID test stage

```sh
cd /home/zames/Mentor_Pi && make build-commissioning-pid
```

Type `MOTORS_RAISED`. Expected artifact properties are `COMMISSIONING_PID`,
`release_qualified=0`, `control_mode=CLOSED_LOOP`, maximum admitted command
`6.0 RPS` as the implementation ceiling, output bounded to `1000 permille`,
and the selected model limit: JGB520 `1.5`, JGB37 `3.0`, JGA27 `6.0`, or
JGB528 `1.1 RPS`.

```sh
cd /home/zames/Mentor_Pi && make flash-commissioning-pid
```

Type `ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED` and
`MOTORS_RAISED_CURRENT_LIMITED`. BOOT0/reset control, programming, read-back,
and normal reset are automatic, with the same pre-programming manual fallback
as the locked flash. Stop on any mode, digest, activation, or verification
failure. The PID-specific verifier rejects locked and direction-check artifacts.

## 2. Start the commissioning runtime

**Warning:** Every wheel remains raised, the current limit is active, and the
physical motor-power stop is within reach.

```sh
cd /home/zames/Mentor_Pi && make start-commissioning-pid
```

Type `MOTORS_RAISED_CURRENT_LIMITED`. This command rejects a locked artifact
and `make start` rejects a commissioning artifact, preventing accidental mode
selection. The PID start path also rejects a direction-check artifact. Require
controller discovery, supervisor `READY`,
`motion_enabled=true`, a nonzero authorization token, and zero targets.

## 3. Run one bounded motor

From a second terminal:

### 3.1 Direction-check stage

```sh
cd /home/zames/Mentor_Pi && make commission-motor
```

Type `MOTORS_RAISED_CURRENT_LIMITED`, then enter motor ID 1–4, a signed
direction command from 0.01–0.25 RPS, and duration 100–5000 ms. Start with
`+0.1` for `1000 ms` to check robot-forward direction. In this temporary mode,
the command magnitude is an admission bound and protocol value; it does not
regulate wheel speed. Its sign alone selects fixed `+250` or `-250` permille.
A 100 ms run is useful for specialized bounded checks but is too short to
classify direction reliably when only one encoder tick occurs. The C++ utility
publishes all four targets at 20 Hz, keeps unselected targets zero, and sends
bounded zero phases before and after motion.

A software pass requires encoder direction matching the requested sign, at
least two encoder ticks, response at least 0.002 RPS or 10% of the command,
selected speed below 0.50 RPS, unselected speed below 0.02 RPS, publish gaps
below 100 ms, no rejection/session/reset change, and final zero. The helper
then asks whether the wheel physically moved in the same requested direction;
answer `y` only after observing it. Run the selected motor again with `-0.1`
to verify reverse. This proves direction and the safety cutoff, not PID or RPM
accuracy.
Remove motor power immediately on wrong direction, excess current, heat,
oscillation, cross-channel response, timeout, or lost communications.

### 3.2 PID test stage

**Warning:** this stage is for controlled closed-loop checks only and is not a
first-pass replacement for direction proof.

Before sending motion commands, set the gains with the PID service while all
motors are stopped:

```sh
ros2 service call /mentor_pi/motors/set_pid mentor_pi_interfaces/srv/SetMotorPid "{
  update_mask: 15,
  proportional_gain: [250.0, 250.0, 250.0, 250.0],
  integral_gain: [0.1, 0.1, 0.1, 0.1],
  derivative_gain: [0.5, 0.5, 0.5, 0.5],
  velocity_filter_new_weight: [0.5, 0.5, 0.5, 0.5]
}"
```

Use `update_mask` values `1`, `2`, `4`, `8`, or any subset up to `15`.
Selected `P`, `I`, and `D` values must stay in `[0, 1000]`; selected filter
weights must stay in `[0, 1]`. On success, `applied_mask` matches
`update_mask`.

The closed-loop controller uses
`error = target_rps - filtered_measured_rps` and the positional form
`output = Kp*error + Ki*integral(error) + Kd*d(error)/dt`. `Kp` is in
permille/RPS, `Ki` in permille/(RPS second), and `Kd` in
permille-second/RPS. Because D acts on error, changing the target produces a
derivative kick; begin with a low target and conservative D while tuning. The
built-in profile values and the example above are commissioning starting
points, not qualified gains for a loaded robot.

Use direct ROS tooling (not `commission-motor`) and command only one channel per
publish. Hold the command at a fixed rate for the test window (rather than
sending once), then stop with a zero command:

```sh
timeout 2s ros2 topic pub --rate 20 /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand "{update_mask: 1, target_rps: [0.100, 0.000, 0.000, 0.000]}"
```

For this stage, `update_mask` is a single bit per motor:

`1 = MOTOR_1`, `2 = MOTOR_2`, `4 = MOTOR_3`, `8 = MOTOR_4`.

Set exactly one `target_rps` entry to a nonzero signed value within the selected
motor model's limit (and always within `[−6.0, 6.0]`) and keep all other entries
`0.0`.

```sh
timeout 2s ros2 topic pub --rate 20 /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand "{update_mask: 1, target_rps: [1.000, 0.000, 0.000, 0.000]}"
timeout 1s ros2 topic pub --rate 20 /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand "{update_mask: 1, target_rps: [0.000, 0.000, 0.000, 0.000]}"
```

To inspect the RPS, in a new terminal:
```sh
ros2 topic echo /mentor_pi/motors/state
```

Expected checks:

- command/response is no longer fixed at 250 permille
- measured RPS tracks the requested target in bounded closed-loop form
- zero target returns measured speed to near zero quickly
- `COMMISSIONING` profile and `release_qualified=0` remain unchanged
- overspeed, per-motor lease, and session kill behavior remain active
- restoring locked firmware returns normal locked behavior after completion

Remove motor power immediately on wrong direction, excess current, heat,
oscillation, cross-channel response, timeout, or lost communications.

## 4. Restore the normal locked image

Stop `make start-commissioning` first.

**Warning:** Disconnect motor and servo power. Never leave commissioning
firmware installed after the guarded test.

```sh
cd /home/zames/Mentor_Pi && make restore-locked
```

Type `ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED`. Expected result is a rebuilt,
verified, read-back `LOCKED` image followed by automatic normal boot. A later
`make passive-check` must again reject the selected nonzero motor probe.

Next: [Tutorial 07: Qualify Hardware and Recovery](07-qualify-hardware-and-recovery.md).
