# 08 — Onboard evidence, ADRC tuning, and qualification

Production actions write below `/var/log/mentor-pi/actions` and require the
verified packaged firmware hash:

```zsh
FIRMWARE_SHA256="$(sha256sum \
  firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf | awk '{print $1}')"

RUNTIME_CONTEXT=production \
PACKAGED_FIRMWARE_SHA256="${FIRMWARE_SHA256}" \
PASSIVE_CHECK_ACK=ACTUATORS_DISCONNECTED \
OLED_PRESENT=1 \
make passive-check
```

Review the generated `SUMMARY.txt`, `command-status.tsv`, `SHA256SUMS`, archive,
and archive digest. Preserve Agent boot/reconnect journals, firmware metadata,
source revision, architecture, board identity, ROS domain, fixture revision,
and supply limit.

After the passive chapters are recorded, use the guarded fixture and root
qualification commands as applicable:

```zsh
RUNTIME_CONTEXT=production PACKAGED_FIRMWARE_SHA256="${FIRMWARE_SHA256}" \
PREFLIGHT_ACK=ACTUATORS_DISCONNECTED make qualification-preflight
```

## ADRC tuning prerequisites

Complete Tutorials 01–07 before this section. Do not begin powered tuning until
the passive encoder-direction result is recorded for all four wheels and the
exact firmware artifact above is archived.

Every powered run requires all of the following:

- a current-limited supply and recorded current limit;
- raised wheels or an equivalent guarded fixture;
- a reachable physical power stop;
- one motor and one sign, or one chassis axis, at a time; and
- a person watching current, temperature, motion, and telemetry throughout.

Stop immediately for a wrong direction, oscillation, unexpected current or
heat, MCU reset, stale telemetry, or motion from an unselected channel. A raw
ROS publisher adds no safety checks beyond its time bound. The independent MCU
lease remains the final software stop path; it does not replace the physical
stop.

Chassis yaw tuning needs a guarded fixture that permits controlled chassis
motion, such as an instrumented roller/dynamometer or a sufficiently bounded
test area. Wheels spinning in free air do not create useful IMU yaw feedback
and therefore cannot tune the chassis yaw controller.

## What this controller does

This repository uses first-order linear active disturbance rejection control
(LADRC). At each update the controller compares a requested motion with the
measured motion. Its extended-state observer (ESO) estimates both the actual
motion and one combined disturbance. That disturbance includes effects the
simple model did not describe well, such as load, friction, battery variation,
and model error.

There are two nested controller layers:

```text
TwistStamped chassis reference
          |
          v
30 Hz chassis LADRC ---- encoder translation/speed + IMU gyro Z
          |
          v
      wheel RPS targets
          |
          v
100 Hz MCU motor LADRC ---- wheel encoders
          |
          v
    PWM permille -> motors
```

Tune the inner 100 Hz motor loop first. An outer chassis loop cannot correct an
inner loop that has the wrong direction, unstable gains, or an unqualified
minimum-drive floor.

For the motor loop, let `r` be target RPS, `y` be filtered measured RPS, `T` be
the actual update period, and `u_previous` be the previously applied PWM output
after clamping and the minimum-drive floor. The firmware evaluates:

```text
e = z1 - y
z1 += T * (z2 + b0 * u_previous - 2 * wo * e)
z2 += T * (-wo * wo * e)
u = (wc * (r - z1) - z2) / b0
```

`z1` is the observer's speed estimate. `z2` is its estimate of all unmodelled
acceleration/disturbance. The controller uses the speed error and subtracts the
estimated disturbance before producing `u`.

## Motor parameters

The four-element arrays are in
`ros2_ws/src/mentor_pi_bringup/config/controller.yaml`. Their order is
`[M1, M2, M3, M4]`, where M1 is front-left, M2 rear-left, M3 front-right, and
M4 rear-right.

| Parameter | Onboard value | Meaning and adjustment |
|---|---:|---|
| `input_gain_rps_per_second_per_permille` (`b0`) | `0.03` | Estimated change in RPS per second from one permille of PWM. A value that is too small produces a larger, more aggressive command; a value that is too large usually produces a weaker initial response. |
| `controller_bandwidth_rad_s` (`wc`) | `4.0` | Desired response speed. Increasing it normally reduces rise time but increases current, overshoot, saturation, and noise sensitivity. A nominal first-order settling estimate is about `4 / wc` seconds before floor or saturation effects. |
| `observer_bandwidth_rad_s` (`wo`) | `12.0` | Speed of the disturbance estimate. A larger value rejects changes sooner but amplifies encoder noise and timing error. The default is three times `wc`. |
| `velocity_filter_new_weight` | `0.8` | Weight of the newest encoder-speed sample. A larger value reacts faster and passes more noise; a smaller value is smoother but adds delay. The firmware fallback remains `0.5`, but the supervisor applies this tuned onboard value before authorizing motion. |

Selected motor values must have `0 < b0 <= 1000`, `wc > 0`,
`wc <= wo <= 50 rad/s`, and filter weight in `[0, 1]`. The limit is not a
recommended tuning range. At 100 Hz, `wo * T` must remain no greater than
`0.5`; timing variation makes operation near the absolute `50 rad/s` limit a
poor starting point.

The minimum-drive floor is separate from ADRC. It is currently compiled as
zero, so it does not raise small nonzero outputs to a fixed duty. With the
defaults, the initial command for a `0.10 RPS` step is approximately
`4 * 0.10 / 0.03 = 13.3` permille. The motor may not move at that duty; record
stall, current, and breakaway behavior rather than treating it as a gain error.
Changing the floor requires a firmware change and renewed motor qualification;
it is not adjustable through these YAML arrays or `set_adrc`.

## Record an unchanged motor baseline

Keep the complete vehicle launch stopped. Direct motor tests must run with the
configuration supervisor but without the Mecanum or Ackermann hardware plugin,
otherwise two processes can publish motor commands.

In the first terminal:

```zsh
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
systemctl is-active mentor-pi-agent.service
ros2 launch mentor_pi_bringup controller.launch.py
```

Replace `0` with the domain installed in Tutorial 05. Wait until the supervisor
reports successful configuration. In a second sourced terminal, require a live
authorization and zero existing motor-command publishers:

```zsh
ros2 topic echo --once /mentor_pi/configuration/motion_authorization
ros2 topic info /mentor_pi/motors/command --verbose
```

Stop if the publisher count is not zero. Start a plainly named capture in a
third sourced terminal; `ros2 bag` continues printing progress until `Ctrl-C`:

```zsh
mkdir -p build/adrc-tuning
ros2 bag record -a -o build/adrc-tuning/motor-m1-default-positive
```

With M1 guarded and the other channels observed, run one visibly active,
three-second positive checkout and then send an explicit all-motor zero:

```zsh
timeout --signal=INT --kill-after=1s 3s \
  ros2 topic pub --rate 20 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 15, target_rps: [0.10, 0.0, 0.0, 0.0]}'

timeout 5s ros2 topic pub --once \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 15, target_rps: [0.0, 0.0, 0.0, 0.0]}'

timeout 5s ros2 topic echo --once /mentor_pi/motors/state
```

Confirm that every reported target is zero and that motion stops. Repeat with
`-0.10` only after positive direction is correct. For M2, M3, and M4, move the
single nonzero value to the corresponding array position. Use a new rosbag name
for every motor, sign, parameter set, and target.

The `0.10 RPS` example is only a low-duty direction checkout and may not break
static friction. Do not jump from it to a full-range target. Each higher target and duration must be
bounded in the reviewed HIL plan for the motor model, fixture, supply limit,
and stop instrumentation.

## Tune the motor loop

Use the unchanged defaults as run zero. For every candidate, change only one
quantity and retain the previous rosbag and instrument record.

1. **Choose the velocity filter.** Run a constant, safely bounded speed and
   inspect the variation of `measured_rps`. Reduce the new-sample weight only
   enough to remove encoder quantization noise; reject a value that adds
   obvious tracking delay. Do not use the endpoints as casual defaults.
2. **Estimate `b0`.** Measure actual applied PWM permille with external
   instrumentation; it is not published in `MotorState`. In a segment that is
   neither stalled nor at the 1000-permille clamp, estimate
   `b0` as `(change in RPS / change in seconds) / applied permille`. Use short
   early-response segments, repeat both signs, and reject segments disturbed by
   fixture contact or load changes.
3. **Adjust `wc`.** Starting from the baseline, change it by only 10–20 percent
   per run. Increase it while rise/settling time improves without unacceptable
   overshoot, current, temperature, floor cycling, or oscillation. Back down to
   the last clean value when any of those worsen.
4. **Adjust `wo`.** Begin with `wo` near three times the accepted `wc`. Increase
   it only if load rejection is too slow and the encoder signal is clean;
   decrease it if measured output chatters or the observer reacts to noise.
   Always keep `wo >= wc` and comfortably inside the timing limit.
5. **Repeat the matrix.** Test both signs, the reviewed RPS range, expected load
   changes, and all four motors independently. One good M1 run does not qualify
   M2–M4.

For each run record target, measured RPS, rise time, peak and percent overshoot,
settling time and tolerance, steady-state error, encoder noise, measured PWM,
current, supply voltage, temperature, watchdog/lease counters, reset reason,
and whether the floor or output clamp was active.

Persist the accepted per-motor values only in the source YAML:

```zsh
nano ros2_ws/src/mentor_pi_bringup/config/controller.yaml
make -C ros2_ws build
```

Never edit `ros2_ws/install`. Stop the supervisor, confirm every motor has
stopped, rebuild, and relaunch it before testing the new arrays. The supervisor
applies all four arrays at session configuration. Manual `set_adrc` overrides
are volatile and are deliberately not used as the evidence source here.

## Chassis ADRC parameters

Tune the chassis layer only after the motor loop has an accepted guarded-HIL
record. Its settings live in exactly one selected source profile:

- `ros2_ws/src/mentor_pi_hardwares/config/mecanum/hardware.yaml`; or
- `ros2_ws/src/mentor_pi_hardwares/config/ackermann/hardware.yaml`.

Mecanum defaults are `b0=5.0`, `wc=1.0`, and `wo=3.0` for translation and the
same values for yaw. The one `linear_*` set is shared by X and Y, so the final
choice must pass both axes. Translation feedback comes from wheel encoders;
yaw feedback comes from `/mentor_pi/imu` gyroscope Z.

Ackermann defaults are `b0=5.0`, `wc=1.0`, and `wo=3.0` for longitudinal
speed. Yaw uses `yaw_adrc_input_gain_per_mps=30.0`, `wc=1.0`, and `wo=3.0`.
The yaw input-gain coefficient is multiplied by measured vehicle speed. Below
`yaw_adrc_minimum_speed_mps=0.1`, yaw ADRC resets and steering returns to its
feed-forward/center behavior, so a slower run cannot tune yaw response.

| Vehicle parameter | Default | Unit and effect |
|---|---:|---|
| Mecanum `linear_adrc_input_gain_per_second` | `5.0` | `1/s`; scales the X/Y correction model. Too small makes the calculated correction more aggressive; too large weakens it. |
| Mecanum `linear_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets the shared X/Y response speed. |
| Mecanum `linear_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets the shared X/Y disturbance-observer speed. |
| Mecanum `yaw_adrc_input_gain_per_second` | `5.0` | `1/s`; scales the yaw-rate correction model. |
| Mecanum `yaw_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets yaw response speed. |
| Mecanum `yaw_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets yaw disturbance-observer speed. |
| Ackermann `linear_adrc_input_gain_per_second` | `5.0` | `1/s`; scales longitudinal-speed correction. |
| Ackermann `linear_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets longitudinal response speed. |
| Ackermann `linear_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets longitudinal disturbance-observer speed. |
| Ackermann `yaw_adrc_input_gain_per_mps` | `30.0` | Coefficient per measured `m/s`; scales steering correction into yaw-rate response. |
| Ackermann `yaw_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets yaw response speed above the minimum-speed gate. |
| Ackermann `yaw_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets yaw disturbance-observer speed above the gate. |
| Ackermann `yaw_adrc_minimum_speed_mps` | `0.1` | `m/s`; below this measured speed the yaw observer resets instead of tuning steering. |

All chassis ADRC gains and bandwidths must be finite and positive, with
`wo >= wc`. The same implementation rejects an update when `wo * T > 0.5`.
Because this loop runs at 30 Hz, keep observer bandwidth well below the nominal
approximately `15 rad/s` timing boundary. The current outer-loop bandwidth of
`1 rad/s` is intentionally lower than the `4 rad/s` motor-loop default; do not
make the outer loop faster than the verified inner loop without evidence.

## Tune Mecanum or Ackermann

Repeat the powered-test boundary before continuing: current-limited supply,
guarded motion-capable fixture, reachable stop, one axis/sign at a time, and
continuous current, temperature, motion, and telemetry observation.

Edit only the selected source profile, rebuild, and start the complete vehicle:

```zsh
nano ros2_ws/src/mentor_pi_hardwares/config/mecanum/hardware.yaml
make -C ros2_ws build
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 launch mentor_pi_hardwares mecanum.launch.py
```

For Ackermann, edit `config/ackermann/hardware.yaml` and launch
`ackermann.launch.py`. Replace `0` with the domain installed in Tutorial 05.
Before publishing a reference, verify that the selected hardware plugin is the
only motor-command publisher:

```zsh
ros2 topic info /mentor_pi/motors/command --verbose
```

Stop if the count is not exactly one. Do not publish directly to
`/mentor_pi/motors/command` while the complete vehicle is running.

Start a new rosbag, then use a time-bounded controller reference. This Mecanum
example tests only positive X at a deliberately small starting reference:

```zsh
ros2 bag record -a -o build/adrc-tuning/mecanum-x-default-positive
```

In another sourced terminal:

```zsh
timeout --signal=INT --kill-after=1s 3s \
  ros2 topic pub --rate 20 \
  /mentor_pi/mecanum_drive_controller/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}}'

timeout 5s ros2 topic pub --once \
  /mentor_pi/mecanum_drive_controller/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}}'
```

Tune Mecanum X first, then Y, then yaw, and repeat both signs. Change only the
corresponding `linear_*` or `yaw_*` group between runs. Because X and Y share
the linear gains, retain the most conservative values that meet both axes.

For Ackermann, first test straight longitudinal response with `angular.z=0` on
`/mentor_pi/ackermann_steering_controller/reference`. Tune yaw only after
straight-speed response passes, at a guarded measured speed above `0.1 m/s`.
Use positive and negative curvature and then repeat in reverse; the yaw input
gain coefficient remains positive, while its effective gain changes sign with
measured speed.

For each chassis run compare the `TwistStamped` reference with encoder-derived
translation/longitudinal speed and IMU gyro-Z yaw rate. Record the same
rise-time, overshoot, settling, steady-error, noise, current, temperature,
watchdog, reset, and saturation observations used for motor tuning. Adjust the
chassis input gain first, then `wc`, then `wo`, by only 10–20 percent per run.
Missing, invalid, or older-than-100-ms encoder/IMU feedback must stop the run;
it is not a poor-gain result.

## Qualification boundary

Campaign commands require their documented fixture variables and exact
acknowledgements; inspect `make help` before use. Retain the source revision,
the controller YAML and selected vehicle YAML, built install metadata, firmware
hash, rosbag, external PWM/current/temperature records, fixture revision, and
the complete console and Agent logs for every accepted candidate.

Do not infer powered motion, ADRC performance, endurance, or release
qualification from a software build, mock, rosbag, passive capture, or green
campaign JUnit result. Candidate gains remain provisional until the required
instrumented HIL matrix is recorded and reviewed against
`VER-HIL-MOT-001` in the
[verification contract](../../framework/verification.md).
