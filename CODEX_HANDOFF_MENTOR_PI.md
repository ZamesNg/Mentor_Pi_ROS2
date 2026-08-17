# Mentor Pi handoff — ackermann_1 PWM/steering diagnosis

Date: 2026-08-15 (Asia/Shanghai)

## Canonical repository and Git workflow

- The correct local repository is `/Users/zamesng/Codes/Mentor_Pi`.
- Do **not** use `/Users/zamesng/Codes/ng_planner_v2/.mentor_pi_edit`; that was inspected only because the earlier chat was opened in the wrong workspace.
- Local `/Users/zamesng/Codes/Mentor_Pi` and robot `sunrise@192.168.2.161:/home/sunrise/Mentor_Pi` are both at commit `7f2d021` (`remove zenoh`).
- Both currently have the same large tracked/untracked onboarding change set. Nothing was committed or pushed in the wrong chat.
- A checksum dry-run found no content differences after excluding `.git`, build/install/log outputs, `.deps`, Python caches, and per-robot generated configuration. It only warned about a local-only nonempty top-level `src/mentor_pi_interfaces/test` hierarchy during the dry-run deletion check; inspect that path before synchronization.
- User-requested workflow from now on:
  1. Make and test source changes in `/Users/zamesng/Codes/Mentor_Pi`.
  2. Commit locally.
  3. Push `main` to `origin` (`https://github.com/ZamesNg/Mentor_Pi.git`).
  4. On devices, preserve/stash any existing dirty work, then `git pull --ff-only`.
  5. Never edit device source first and later copy it back.
- Do not commit per-robot generated identity/configuration files unless repository policy explicitly says otherwise.

## Robot and ROS environment

- Robot: `ackermann_1`
- Address: `192.168.2.161`
- Actual SSH operator on this device: `sunrise` (do not hard-code this username into repository scripts).
- `MENTOR_PI_TYPE=ackermann`
- `MENTOR_PI_NAME=ackermann_1`
- `ROS_DOMAIN_ID=42`
- `ROS_LOCALHOST_ONLY=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DISCOVERY_SERVER=192.168.2.191:11811`
- micro-ROS Agent is active and uses `/dev/mentor_pi_mcu -> /dev/ttyACM0`.
- Firmware was rebuilt and reflashed successfully for `ackermann_1`; CubeProgrammer read-back verification passed.
- Current firmware size from that build: text 152972, data 3844, bss 149448.

## Current runtime/safety state

- `vehicle.launch.py` is not intentionally running now.
- A transient user unit is running only the configuration supervisor:
  - unit: `mentor-pi-controller-test.service`
  - command: `ros2 launch mentor_pi_bringup controller.launch.py robot_name:=ackermann_1`
- No continuous host motor/PWM command publisher was found after stopping vehicle bringup.
- PWM3 was returned to center (`1500 us`) after every direct test.
- Motors are currently commanded to zero.
- Stop the test supervisor if desired with:

  ```bash
  systemctl --user stop mentor-pi-controller-test.service
  ```

## Confirmed drive-side findings

- Ackermann rear motors are correctly mapped:
  - rear-left -> M2 (`target_rps[1]`)
  - rear-right -> M4 (`target_rps[3]`)
  - update mask `0x0A` / decimal `10`
- With a `0.50 m/s` raised-wheel test, host `ros2_control` generated:

  ```text
  update_mask: 10
  target_rps: [0.0, +2.877066, 0.0, -2.877066]
  ```

- MCU feedback is about 10 Hz. Original generated values were exactly 100 ms:

  ```yaml
  feedback_timeout_ms: 100
  imu_timeout_ms: 100
  ```

  This caused ordinary scheduling jitter to make `AckermannHardware::read()` return `ERROR`, dropping `mentor_pi_hardware` to `unconfigured`.
- Device-local generated file was changed for the trial to 500 ms for both values. After restart the hardware remained active beyond the previous failure point. This generated file was intentionally excluded from repository checksum comparison.
- When motors were energized, USB temporarily disconnected and `/dev/ttyACM0` re-enumerated. Later firmware diagnostics clarified the reset cause:

  ```text
  last_reset_reason: 3   # independent watchdog
  last_watchdog_task: 2  # micro-ROS task
  ```

  Therefore the earlier USB event should not be described as proven brownout; it is a firmware watchdog reset unless later electrical evidence says otherwise.

## Confirmed host low-speed steering bug

In `ros2_ws/src/mentor_pi_hardwares/src/ackermann_hardware.cpp`, steering currently does this:

```cpp
const double feedforward_steering = /* controller steering reference */;
double steering_command = 0.0;
if (std::fabs(measured_speed_m_s) < yaw_adrc_minimum_speed_mps_) {
  yaw_adrc_.Reset();
  applied_steering_correction_rad_ = 0.0;
} else {
  // steering_command becomes feedforward + correction only here
}
```

At requested `linear.x=0.10`, the threshold is also `yaw_adrc_minimum_speed_mps=0.1`; measured speed can remain just below it, forcing PWM steering to center. The likely host fix is to initialize:

```cpp
double steering_command = feedforward_steering;
```

and only suppress the ADRC correction below the threshold. This fix has **not** been implemented yet.

Also, at `linear.x=0.10 m/s`, `angular.z=3.0 rad/s` is geometrically impossible and saturates. With wheelbase `0.135 m` and max steering `0.6 rad`, maximum yaw rate at `0.10 m/s` is about `0.51 rad/s`.

## Direct PWM3 test and firmware-side fault

User asked for a test through `controller.launch.py`, without vehicle hardware continuously overwriting the PWM topic.

Commands sent directly to:

```text
/ackermann_1/pwm_servos/command
mentor_pi_interfaces/msg/PwmServoCommand
```

PWM3 selection is correct:

```text
update_mask = 4  # bit 2, third array element, PWM3
```

Tests performed:

- one-shot sweep: `1500 -> 1600 -> 1400 -> 1500 us`
- repeated best-effort test: ten `1600 us` samples at 10 Hz, then ten `1500 us` samples at 10 Hz
- duration was `500 ms`

Firmware definitely received the messages:

- `command_messages` increased by exactly the number sent
- `command_rejections: 0`
- PWM mailbox overwrite counter (`mailbox_overwrites[1]`) incremented during bursts
- session remained active (`session_state: 3`)
- PWM peripheral error counter (`peripheral_errors[4]`) stayed zero

Nevertheless, MCU state always remained:

```text
target_pulse_width_us: [1500, 1500, 1500, 1500]
output_pulse_width_us: [1500, 1500, 1500, 1500]
moving_mask: 0
```

This proves a second, firmware-side problem after ROS reception. It is separate from the host low-speed steering bug.

Relevant inspected firmware path:

- `MicroRosRuntime::OnPwmServoCommand` validates and publishes to the PWM mailbox.
- `ControllerRuntime::PublishPwmServoCommand` tags the mailbox by session generation.
- `ControllerRuntime::ProcessPwmCommands` consumes snapshots and calls `PwmServoController::AcceptCommand`.
- `PreparePendingPwmFrame` submits a shadow.
- state commits only after `pwm_servo_frame_sequence()` advances.
- STM32 TIM13 setup, NVIC handler, and callback exist in source.
- The built/flashed ELF contains strong symbols for:
  - `TIM8_UP_TIM13_IRQHandler`
  - `HandlePwmServoTimerFromIsr()`
  - `g_servo_frame_sequence`

Most likely next investigation: instrument/expose TIM13 `g_servo_frame_sequence`, PWM mailbox consumption, and pending-frame commit. Determine whether the timer sequence is advancing and whether `ProcessPwmCommands` discards a snapshot on the session/generation guard. Add focused firmware tests first, then implement locally, commit/push, pull on the robot, rebuild/reflash, and repeat the direct PWM3 test.

## Testing constraints/preferences

- User previously requested focused tests only; do not run the whole project suite unnecessarily.
- Preserve existing dirty work and unrelated changes.
- Do not assume the operator username in repository code.
- Keep physical tests small and return PWM3 to 1500 after each test.

