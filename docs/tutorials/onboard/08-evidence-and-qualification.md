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

The inner motor loop uses first-order nonlinear active disturbance rejection
control (NLADRC). At each update its extended-state observer (ESO) estimates
both motor speed and one combined disturbance. That disturbance includes load,
friction, battery variation, and model error. The outer chassis loops remain
first-order linear ADRC.

There are two nested controller layers:

```text
TwistStamped chassis reference
          |
          v
50 Hz chassis LADRC ---- encoder translation/speed + IMU gyro Z
          |
          v
      wheel RPS targets
          |
          v
100 Hz MCU motor NLADRC ---- wheel encoders
          |
          v
    PWM permille -> motors
```

Tune the inner 100 Hz motor loop first. An outer chassis loop cannot correct an
inner loop that has the wrong direction, unstable gains, or an unqualified
minimum-drive floor.

For the motor loop, let `r` be target RPS, `y` be filtered measured RPS, `T` be
the actual update period, and `u_previous` be the previously applied PWM output
after clamping and the directional minimum-drive floor. The nonlinear error
map preserves units and unit slope for small errors:

```text
fal(e, alpha, delta) = e                                      when |e| <= delta
                     = sign(e) * delta * (|e| / delta)^alpha otherwise

eo = z1 - y
z1_dot = -a * z1 + z2 + b0 * u_previous
         - 2 * wo * fal(eo, alpha_velocity, delta_observer)
z2_dot = -wo * wo * fal(eo, alpha_disturbance, delta_observer)
         - leakage * z2
z1 += T * z1_dot
z2 += T * z2_dot
bounded_disturbance = clamp(z2, -disturbance_limit, disturbance_limit)
u = (a * z1
     + wc * fal(r - z1, alpha_controller, delta_controller)
     - bounded_disturbance) / b0
```

`z1` is the observer's speed estimate. `z2` is its estimate of all unmodelled
acceleration/disturbance. Exponent `1` makes `fal` linear everywhere. An
exponent below `1` compresses errors outside its threshold while retaining the
linear small-error region. The disturbance limit bounds only the compensation
used by the controller; leakage controls how quickly the observer estimate
decays.

## Motor parameters

The four-element arrays are in
`ros2_ws/src/mentor_pi_bringup/config/controller.yaml`. Their order is
`[M1, M2, M3, M4]`, where M1 is front-left, M2 rear-left, M3 front-right, and
M4 rear-right.

| Parameter | Onboard value | Meaning and adjustment |
|---|---:|---|
| `known_velocity_decay_rate_s_inverse` (`a`) | `0.0` | Known first-order speed decay in `s^-1`. Keep zero until a guarded coast-down identification supports a nonzero value. |
| `input_gain_rps_per_second_per_permille` (`b0`) | `0.03` | Estimated change in RPS per second from one permille of PWM. A value that is too small produces a larger, more aggressive command; a value that is too large usually produces a weaker initial response. |
| `controller_bandwidth_rad_s` (`wc`) | `4.0` | Desired response speed. Increasing it normally reduces rise time but increases current, overshoot, saturation, and noise sensitivity. A nominal first-order settling estimate is about `4 / wc` seconds before floor or saturation effects. |
| `controller_fal_exponent` | `1.0` | Nonlinear speed-error exponent. Reduce below one only after the linear baseline is stable; it compresses large errors outside the controller threshold. |
| `controller_fal_threshold_rps` | `0.1` | Half-width of the unit-slope controller-error region in RPS. |
| `observer_bandwidth_rad_s` (`wo`) | `12.0` | Speed of the disturbance estimate. A larger value rejects changes sooner but amplifies encoder noise and timing error. The default is three times `wc`. |
| `observer_velocity_fal_exponent` | `1.0` | Exponent applied to the observer innovation in the speed-state correction. |
| `observer_disturbance_fal_exponent` | `1.0` | Exponent applied to the observer innovation in the disturbance-state correction. |
| `observer_fal_threshold_rps` | `0.1` | Shared unit-slope threshold for both observer corrections. |
| `disturbance_leakage_s_inverse` | `0.0` | Disturbance-estimate decay in `s^-1`. A small nonzero value can shed stale friction/load estimates, but too much weakens steady disturbance rejection. |
| `disturbance_estimate_limit_rps_per_second` | `30.0` | Symmetric limit on disturbance compensation in RPS/s. It cannot exceed `b0 * 1000`; start with a limit supported by observed load steps. |
| `velocity_filter_new_weight` | `0.8` | Weight of the newest encoder-speed sample. A larger value reacts faster and passes more noise; a smaller value is smoother but adds delay. The firmware fallback remains `0.5`, but the supervisor applies this tuned onboard value before authorizing motion. |
| `positive_minimum_drive_permille` | `0` | Minimum same-direction positive-target output. Qualify it with the stopped-state breakaway sweep below; maximum is 250 permille. |
| `negative_minimum_drive_permille` | `0` | Independent minimum same-direction negative-target output. Friction and mechanics may be asymmetric; maximum is 250 permille. |

Selected motor values must satisfy `0 <= a <= 50`, `0 < b0 <= 1000`,
`0 < wc <= wo <= 50 rad/s`, exponents in `[0.1, 1]`, thresholds in
`[0.001, 6] RPS`, leakage in `[0, 50] s^-1`, disturbance limit in
`[0, b0 * 1000] RPS/s`, filter weight in `[0, 1]`, and each floor in
`[0, 250]` permille. At 100 Hz, each of `a*T`, `wo*T`, and `leakage*T` must
remain no greater than `0.5`; operation at an absolute boundary is not a
recommended starting point.

The directional floor is actuator compensation, not a `fal` threshold. After
the candidate is clamped to +/-1000 permille, firmware raises only a nonzero
candidate whose sign agrees with the nonzero target. It never raises a braking
or opposing candidate, and zero targets still command zero. With the default
linear exponents and zero floor, the initial command for a `0.10 RPS` step is
approximately `4 * 0.10 / 0.03 = 13.3` permille and may not break static
friction. Record breakaway output separately for each direction: with the motor
stopped before every change, sweep in 25-permille increments, refine around
the first repeatable motion in 5-permille increments, and choose the smallest
repeatable breakaway value plus a 10-permille margin. Abort instead of exceeding
250 permille, or on unexpected current, heat, motion, or oscillation. These
values are runtime-adjustable through `set_adrc` and the supervisor YAML, but
any nonzero production floor still requires renewed guarded motor
qualification.

## Diagnose a low-speed zero before changing the floor

Firmware `MotorCommand.target_rps` and `MotorState.target_rps/measured_rps` are
in output-shaft revolutions per second. `ros2_control` wheel-joint interfaces
are in radians per second: `0.2 rad/s` is only
`0.2 / (2*pi) = 0.0318 RPS`, not `0.2 RPS`. At the default linear-exponent
baseline, a direct `0.2 RPS` motor step initially requests approximately
`4 * 0.2 / 0.03 = 26.7` permille before disturbance compensation, clamping,
and any directional floor.

Before attributing zero motion to stiction, inspect `/mentor_pi/motors/state`.
The selected `target_rps` must retain the intended nonzero RPS, the watchdog
stop bit and selected lease-expiry counter must remain unchanged, and the
command publisher must refresh faster than the independent 198 ms firmware
lease. If the reported target is already zero, diagnose units, the 100 ms
controller reference timeout, configuration authorization/session state,
publisher ownership, and command validation first. Stiction is plausible only
when the nonzero target is accepted and retained while measured RPS remains
near zero; PWM/current instrumentation is still required because `MotorState`
does not publish bridge output.

## Record an unchanged motor baseline

Keep the complete vehicle launch stopped. Direct motor tests must run with the
configuration supervisor but without the Mecanum or Ackermann hardware plugin,
otherwise two processes can publish motor commands.

In the first terminal:

```zsh
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROBOT_NAME=mecanum_1
systemctl is-active mentor-pi-agent.service
ros2 launch mentor_pi_bringup controller.launch.py \
  robot_name:=${ROBOT_NAME}
```

Replace the domain and robot name with the values installed in Tutorial 05.
Wait until the supervisor reports successful configuration. In a second
sourced terminal with the same `ROBOT_NAME`, require a live authorization and
zero existing motor-command publishers:

```zsh
ros2 topic echo --once \
  /${ROBOT_NAME}/configuration/motion_authorization
ros2 topic info /${ROBOT_NAME}/motors/command --verbose
```

Stop if the publisher count is not zero. Start a plainly named capture in a
third sourced terminal; `ros2 bag` continues printing progress until `Ctrl-C`:

```zsh
mkdir -p build/adrc-tuning
ros2 bag record \
  -o build/adrc-tuning/${ROBOT_NAME}-motor-m1-linear-zero-floor-positive \
  /${ROBOT_NAME}/heartbeat \
  /${ROBOT_NAME}/diagnostics \
  /${ROBOT_NAME}/motors/state \
  /${ROBOT_NAME}/motors/command \
  /${ROBOT_NAME}/battery/state \
  /${ROBOT_NAME}/configuration/motion_authorization
```

With M1 guarded and the other channels observed, run the bounded commissioning
node for one three-second positive `0.2 RPS` linear, zero-floor baseline. The
namespace remap is required: it scopes the node's authorization, telemetry,
publisher-conflict check, and commands to the selected robot.

```zsh
ros2 run mentor_pi_bringup motor_commissioning --ros-args \
  -r __ns:=/${ROBOT_NAME} \
  -p acknowledgement:=MOTORS_RAISED_CURRENT_LIMITED \
  -p motor_id:=1 \
  -p target_rps:=0.2 \
  -p duration_ms:=3000

timeout 5s ros2 topic echo --once /${ROBOT_NAME}/motors/state
```

The node publishes all four targets at 20 Hz, includes repeated 500 ms zero
phases before and after the bounded drive phase, and fails closed on stale
telemetry, a changed session, command rejection, watchdog activity, unexpected
motion, wrong direction, or another motor-command publisher. `Ctrl-C` requests
the same bounded post-stop phase; if telemetry or the stop cannot be confirmed,
use the reachable physical power stop. The independent MCU lease remains the
backup.

Require a `PASS` summary, confirm that every reported target is zero and that
motion stops, then stop the rosbag cleanly. Repeat in a new bag with
`target_rps:=-0.2` only after the positive direction is correct. Repeat M2,
M3, and M4 one at a time by changing only `motor_id`. Use a new rosbag name for
every motor, sign, parameter set, and target. The commissioning summary is a
safety result, not the tracking acceptance calculation; retain the bag and
instrument record for rise time, one-second settled mean, overshoot, current,
voltage, and temperature analysis.

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
3. **Measure directional breakaway.** With `fal` exponents still `1`, keep the
   motor stopped before every floor update. Sweep each sign in 25-permille
   increments, then refine around first motion in 5-permille increments. Choose
   the smallest repeatable breakaway value plus 10 permille, repeat current,
   heating, stop, and low-speed tracking checks, and abort rather than exceed
   250 permille. Never infer one direction from the other.
4. **Adjust `wc`.** Starting from the baseline, change it by only 10–20 percent
   per run. Increase it while rise/settling time improves without unacceptable
   overshoot, current, temperature, floor cycling, or oscillation. Back down to
   the last clean value when any of those worsen.
5. **Adjust `wo`.** Begin with `wo` near three times the accepted `wc`. Increase
   it only if load rejection is too slow and the encoder signal is clean;
   decrease it if measured output chatters or the observer reacts to noise.
   Always keep `wo >= wc` and comfortably inside the timing limit.
6. **Bound and decay the disturbance estimate.** Keep known decay and leakage
   zero unless coast-down/load-release data identifies them. Set the
   disturbance limit high enough for measured load rejection but no higher
   than supported by the record; it can never exceed `b0 * 1000`.
7. **Introduce `fal` nonlinearity last.** Keep all exponents at `1` for the
   accepted linear baseline. Then change one controller or observer exponent
   and its threshold at a time. Lower exponents compress large errors; reject
   changes that slow breakaway/load recovery, hide sign errors, or add floor
   cycling near zero.
8. **Repeat the matrix.** Test both signs, the reviewed RPS range, expected load
   changes, and all four motors independently. One good M1 run does not qualify
   M2–M4.

For each run record target, measured RPS, rise time, peak and percent overshoot,
settling time and tolerance, steady-state error, encoder noise, measured PWM,
current, supply voltage, temperature, watchdog/lease counters, reset reason,
and whether the floor or output clamp was active.

For a provisional M1 positive-floor candidate, stop the vehicle controller,
confirm all four reported targets are zero and all measured magnitudes are
below `0.01 RPS`, then call the complete fixed-array service:

```zsh
ros2 service call /mecanum_1/motors/set_adrc \
  mentor_pi_interfaces/srv/SetMotorAdrc \
  '{
    update_mask: 1,
    known_velocity_decay_rate_s_inverse: [0.0, 0.0, 0.0, 0.0],
    input_gain_rps_per_second_per_permille: [0.03, 0.03, 0.03, 0.03],
    controller_bandwidth_rad_s: [4.0, 4.0, 4.0, 4.0],
    controller_fal_exponent: [1.0, 1.0, 1.0, 1.0],
    controller_fal_threshold_rps: [0.1, 0.1, 0.1, 0.1],
    observer_bandwidth_rad_s: [12.0, 12.0, 12.0, 12.0],
    observer_velocity_fal_exponent: [1.0, 1.0, 1.0, 1.0],
    observer_disturbance_fal_exponent: [1.0, 1.0, 1.0, 1.0],
    observer_fal_threshold_rps: [0.1, 0.1, 0.1, 0.1],
    disturbance_leakage_s_inverse: [0.0, 0.0, 0.0, 0.0],
    disturbance_estimate_limit_rps_per_second: [30.0, 30.0, 30.0, 30.0],
    velocity_filter_new_weight: [0.8, 0.8, 0.8, 0.8],
    positive_minimum_drive_permille: [25, 0, 0, 0],
    negative_minimum_drive_permille: [0, 0, 0, 0]
  }'
```

Require `result.code == OK` and `applied_mask == 1` before the next guarded
motion attempt. This candidate is volatile, is cleared by reset or an actual
motor-model change, and may be replaced by supervisor configuration on a new
session. It is a sweep convenience, not qualification evidence; only the
reviewed source YAML and recorded instrument/HIL result establish the accepted
setting.

Persist the accepted per-motor values only in the source YAML:

```zsh
nano ros2_ws/src/mentor_pi_bringup/config/controller.yaml
make -C ros2_ws build
```

Never edit `ros2_ws/install`. Stop the supervisor, confirm every motor has
stopped, rebuild, and relaunch it before testing the new arrays. The supervisor
applies all twelve floating-point arrays and both directional-floor arrays at
session configuration. Manual `set_adrc` overrides are volatile and are
deliberately not used as the evidence source here.

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
| Mecanum `linear_adrc_measurement_lpf_cutoff_hz` | `5.0` | `Hz`; filters encoder-derived X/Y velocity measurements before their ADRCs. |
| Mecanum `yaw_adrc_input_gain_per_second` | `5.0` | `1/s`; scales the yaw-rate correction model. |
| Mecanum `yaw_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets yaw response speed. |
| Mecanum `yaw_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets yaw disturbance-observer speed. |
| Mecanum `yaw_adrc_measurement_lpf_cutoff_hz` | `5.0` | `Hz`; filters IMU gyro-Z before yaw ADRC. |
| Ackermann `linear_adrc_input_gain_per_second` | `5.0` | `1/s`; scales longitudinal-speed correction. |
| Ackermann `linear_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets longitudinal response speed. |
| Ackermann `linear_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets longitudinal disturbance-observer speed. |
| Ackermann `linear_adrc_measurement_lpf_cutoff_hz` | `5.0` | `Hz`; filters encoder-derived longitudinal speed before ADRC. |
| Ackermann `yaw_adrc_input_gain_per_mps` | `30.0` | Coefficient per measured `m/s`; scales steering correction into yaw-rate response. |
| Ackermann `yaw_adrc_controller_bandwidth_rad_s` | `1.0` | `rad/s`; sets yaw response speed above the minimum-speed gate. |
| Ackermann `yaw_adrc_observer_bandwidth_rad_s` | `3.0` | `rad/s`; sets yaw disturbance-observer speed above the gate. |
| Ackermann `yaw_adrc_measurement_lpf_cutoff_hz` | `5.0` | `Hz`; filters IMU gyro-Z before yaw ADRC and resets below the minimum-speed gate. |
| Ackermann `yaw_adrc_minimum_speed_mps` | `0.1` | `m/s`; below this measured speed the yaw observer resets instead of tuning steering. |

All chassis ADRC gains, bandwidths, and measurement-LPF cutoffs must be finite
and positive, with
`wo >= wc`. The same implementation rejects an update when `wo * T > 0.5`.
Because this loop runs at 50 Hz, keep observer bandwidth well below the nominal
approximately `25 rad/s` timing boundary. The current outer-loop bandwidth of
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
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
ros2 launch mentor_pi_hardwares vehicle.launch.py
```

For Ackermann, select the generated Ackermann profile through
`MENTOR_PI_TYPE`. Replace the domain and localhost-only values with those
installed in Tutorial 05. The vehicle launch is command-only. Before publishing a reference, verify that the
selected hardware plugin is the only motor-command publisher:

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
  /mentor_pi/vehicle/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.05, y: 0.0}, angular: {z: 0.0}}}'

timeout 5s ros2 topic pub --once \
  /mentor_pi/vehicle/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}}'
```

Tune Mecanum X first, then Y, then yaw, and repeat both signs. Change only the
corresponding `linear_*` or `yaw_*` group between runs. Because X and Y share
the linear gains, retain the most conservative values that meet both axes.

For Ackermann, first test straight longitudinal response with `angular.z=0` on
`/mentor_pi/vehicle/reference`. Tune yaw only after
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

## External trajectory tracking boundary

Outer trajectory tracking, global map pose, and mocap processing are owned by
the central experiment application. This repository exposes only the bounded
`/<robot>/vehicle/reference` chassis-command boundary.

Do not tune or qualify an external tracker from this repository. Preserve the
post-bound command, controller timeout, chassis feedback, firmware leases, and
all guarded-HIL evidence when evaluating any external controller.

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
