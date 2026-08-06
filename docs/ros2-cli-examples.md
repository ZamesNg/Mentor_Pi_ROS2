# ROS 2 CLI Examples

These examples use the final `mentor_pi_interfaces` schemas and absolute v2
names. They are intended for ROS 2 Humble on Ubuntu 22.04 after the interface and
bringup packages have been built.

## Prepare the shell

Source ROS and the workspace that contains the built packages:

```sh
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start the Agent and supervisor as described in the
[bringup package guide](../src/mentor_pi_bringup/README.md), then confirm that
the MCU node is present:

```sh
ros2 node list
ros2 node info /mentor_pi/controller
```

The Agent, USB connection, and flashed MCU are required for live responses. The
`ros2 interface show` commands below work with only the built interface package.

## Inspect types and telemetry

Inspect the generated type before composing a command:

```sh
ros2 interface show mentor_pi_interfaces/msg/MotorCommand
ros2 interface show mentor_pi_interfaces/srv/SetMotorModel
```

Heartbeat is a bounded reliable topic:

```sh
ros2 topic echo \
  /mentor_pi/heartbeat \
  mentor_pi_interfaces/msg/Heartbeat
```

Motor state is best-effort and volatile. Request matching QoS so the CLI does
not ask a best-effort publisher for reliable delivery:

```sh
ros2 topic echo \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /mentor_pi/motors/state \
  mentor_pi_interfaces/msg/MotorState
```

Button events use reliable delivery with writer history depth eight:

```sh
ros2 topic echo \
  --qos-reliability reliable \
  --qos-depth 8 \
  /mentor_pi/buttons/events \
  mentor_pi_interfaces/msg/ButtonEvent
```

## Publish bounded commands

Blink LED 1 three times. This is a discrete reliable command:

```sh
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-depth 1 \
  /mentor_pi/leds/command \
  mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 1, on_time_ms: 200, off_time_ms: 200, repeat: 3}'
```

Replace only OLED line 1. Each string is limited to 23 printable ASCII bytes:

```sh
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-depth 1 \
  /mentor_pi/oled/command \
  mentor_pi_interfaces/msg/OledCommand \
  "{update_mask: 1, line_1: 'RRCLite v2 ready', line_2: ''}"
```

Send an explicit stop to all four motors using the depth-one, best-effort motion
QoS. This command is valid in both the normal motor-locked image and a guarded
commissioning image:

```sh
ros2 topic pub --once \
  --qos-reliability best_effort \
  --qos-durability volatile \
  --qos-depth 1 \
  /mentor_pi/motors/command \
  mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 15, target_rps: [0.0, 0.0, 0.0, 0.0]}'
```

Do not substitute a nonzero target merely to test connectivity. A normal image
rejects it as `UNSUPPORTED`; a commissioning image can move hardware and must
be used only with the fixture and precautions in
[Flashing and first bring-up](flashing-and-first-bringup.md).

### Prove the normal-image motor lock

The first-board checklist deliberately requires two nonzero rejection probes,
but they are **only** for an artifact already verified as `LOCKED`, with motor
power physically disconnected and all wheels contained. Never run this block
against a commissioning image. Run it after the 60-second IMU-characterization
monitor; the expected motor rejections would otherwise contaminate that
monitor's clean diagnostic window.

First archive diagnostics and motor state. Record `command_messages`,
`command_rejections`, `motor_command_rejections`,
`motor_command_consumptions`, `motor_lease_expiries`, `mailbox_overwrites[0]`,
and the four reported targets:

```sh
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics \
  | tee locked-motor-before.yaml
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability volatile \
  /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState \
  | tee locked-motor-state-before.yaml
```

Send one selected nonzero target. This is not a connectivity example: it is a
specific normal-image lock test under the passive fixture above.

```sh
ros2 topic pub --once \
  --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 1, target_rps: [0.01, 0.0, 0.0, 0.0]}'
sleep 2
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics \
  | tee locked-motor-selected-nonzero.yaml
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability volatile \
  /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState \
  | tee locked-motor-state-selected-nonzero.yaml
```

For one delivered message, `command_messages` and `command_rejections` each
increase by one, `motor_command_rejections[0]` increases by one, and the last
error becomes code 6 (`UNSUPPORTED`), source 2 (motors), detail 0. No motor
command is consumed: `motor_command_consumptions`, the motor mailbox overwrite
counter, all lease-expiry counters, all targets, and every physical bridge/PWM
output remain unchanged. If the best-effort one-shot is not reflected in
`command_messages`, repeat only after confirming the image is still locked; do
not infer a pass from a dropped command.

Next send the required mixed selected zero/nonzero command:

```sh
ros2 topic pub --once \
  --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 3, target_rps: [0.0, 0.01, 0.0, 0.0]}'
sleep 2
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics \
  | tee locked-motor-mixed-rejection.yaml
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability volatile \
  /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState \
  | tee locked-motor-state-mixed-rejection.yaml
```

For one delivered mixed message, the two aggregate counters again increase by
one and `motor_command_rejections[0]` and `[1]` each increase by one. Both
selected fields are rejected atomically: no target, lease, mailbox generation,
or output changes. A counter delta without unchanged ROS targets and electrical
zero-output evidence is not a pass.

## Run the locked-image passive peripheral smoke sequence

This sequence is for the normal motor-locked image during first-board bring-up.
It is not peripheral HIL qualification. Before running it, physically
disconnect motor power, unplug all PWM servos and bus-servo devices/mechanisms,
and attach only high-impedance scope or logic-analyzer probes to actuator
outputs. The firmware intentionally starts all four PWM-servo pins at 1500
microseconds during safe boot, so removing only motor power does not make an
attached servo mechanism passive.

Run the sequence one block at a time, observe the named hardware, and archive a
diagnostics snapshot before the first command and after cleanup:

```sh
ros2 topic echo --once \
  --qos-reliability reliable \
  /mentor_pi/diagnostics \
  mentor_pi_interfaces/msg/ControllerDiagnostics
```

First record non-actuating telemetry. PWM state must initially report four
1500 microsecond targets/outputs, zero offsets unless the supervisor applied
reviewed values, and `moving_mask: 0`. Battery state must be finite, at most
20000 mV, and consistent with the bench supply when `valid: true`. Do not run
the host-buzzer observation while `below_threshold` is true: the intentional
battery alarm has priority. Use a reviewed normal-range bench voltage above the
active threshold plus its 200 mV clear hysteresis; do not hide an unexpected
battery reading by changing the threshold merely to make this smoke test pass:

```sh
ros2 topic echo --once \
  --qos-reliability best_effort \
  --qos-durability volatile \
  /mentor_pi/pwm_servos/state \
  mentor_pi_interfaces/msg/PwmServoState

ros2 topic echo --once \
  --qos-reliability reliable \
  /mentor_pi/battery/state \
  mentor_pi_interfaces/msg/BatteryState
```

For the unverified first-board IMU transform, do not wait indefinitely for a
late-joined volatile `/mentor_pi/imu` sample and do not interpret its invalid
zero vectors as raw axes. Use the characterization monitor from the bring-up
checklist for the ROS-visible read-path proof, then use the debugger-only
`rrclite_imu_characterization_snapshot` procedure in
[Flashing and first bring-up](flashing-and-first-bringup.md#raw-imu-axis-characterization-over-swd)
for address, revision, six-face gravity, and three-axis rotation evidence.

Observe buttons in one terminal while pressing and releasing each physical
button separately. Confirm `button_id` 1 and 2, active-low behavior, `PRESSED`,
and the applicable release/click event before interrupting the echo:

```sh
ros2 topic echo \
  --qos-reliability reliable \
  --qos-depth 8 \
  /mentor_pi/buttons/events \
  mentor_pi_interfaces/msg/ButtonEvent
```

Exercise each LED independently with a finite pattern. Send the explicit off
command after observing each LED; do not rely on a shell exit as output cleanup:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 1, on_time_ms: 200, off_time_ms: 200, repeat: 2}'
sleep 1
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 1, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
```

Repeat that pair with `led_id: 2` and `led_id: 3`. Their physical polarity may
differ, but semantic on/off behavior must match. Then exercise a brief finite
buzzer pattern and explicitly turn it off:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
  '{frequency_hz: 1000, on_time_ms: 100, off_time_ms: 100, repeat: 2}'
sleep 1
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
  '{frequency_hz: 0, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
```

Use low intensity to prove the two RGB pixels independently, then drive both
off. The second command must preserve pixel 1 red while changing only pixel 2;
the final all-pixel update is the cleanup state:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
  '{update_mask: 1, red: [16, 0], green: [0, 0], blue: [0, 0]}'
sleep 1
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
  '{update_mask: 2, red: [0, 0], green: [0, 16], blue: [0, 0]}'
sleep 1
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
  '{update_mask: 3, red: [0, 0], green: [0, 0], blue: [0, 0]}'
```

Verify both host OLED lines and the controller-owned battery line, then clear
the two host lines:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/oled/command mentor_pi_interfaces/msg/OledCommand \
  "{update_mask: 3, line_1: 'RRCLite v2 line 1', line_2: 'line 2'}"
sleep 1
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/oled/command mentor_pi_interfaces/msg/OledCommand \
  "{update_mask: 3, line_1: '', line_2: ''}"
```

With all four PWM-servo connectors still physically unplugged, use a scope or
logic analyzer to map the four pins. Publish five copies because this topic is
best effort. Verify the four distinct final pulse widths and common 20 ms
frames in telemetry and electrically, then publish five neutral copies and
verify all four outputs return to 1500 microseconds:

```sh
ros2 topic pub --times 5 --rate 10 \
  --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
  /mentor_pi/pwm_servos/command mentor_pi_interfaces/msg/PwmServoCommand \
  '{update_mask: 15, duration_ms: 200, pulse_width_us: [1400, 1450, 1550, 1600]}'
sleep 1
ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
  /mentor_pi/pwm_servos/state mentor_pi_interfaces/msg/PwmServoState
ros2 topic pub --times 5 --rate 10 \
  --qos-reliability best_effort --qos-durability volatile --qos-depth 1 \
  /mentor_pi/pwm_servos/command mentor_pi_interfaces/msg/PwmServoCommand \
  '{update_mask: 15, duration_ms: 200, pulse_width_us: [1500, 1500, 1500, 1500]}'
sleep 1
ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
  /mentor_pi/pwm_servos/state mentor_pi_interfaces/msg/PwmServoState
```

Do not publish `BusServoCommand` and do not call `configure` in this passive
sequence. To check UART5 with no servo attached, call the read-only state
service while observing the TX waveform; `TIMEOUT` with zero valid fields is
the expected result and increments the documented bus timeout diagnostic. If
exactly one supported servo is instead safely powered and restrained, the same
read may return `OK`. It must not change persistent state or request motion:

```sh
ros2 service call \
  /mentor_pi/bus_servos/get_state \
  mentor_pi_interfaces/srv/GetBusServoState \
  '{servo_id: 1, fields: 511}'
```

Finally capture diagnostics again. Apart from an intentionally recorded
no-device bus `TIMEOUT`, `command_rejections`, `peripheral_errors`, and
`peripheral_timeouts` must not increase. Confirm all three LEDs, buzzer, and
both RGB pixels are off, both host OLED lines are clear, PWM-servo state is
neutral, no bus move/configure frame was sent, and all motor targets and drive
outputs remained zero throughout.

## Run guarded one-motor commissioning

Use the bounded C++ commissioning utility instead of publishing a nonzero
`MotorCommand` directly. Run it only after the locked-image checks have passed,
the commissioning image has been flashed and verified, a second person has
signed off, all wheels are contained, and the recorded current limit is active:

```sh
ros2 run mentor_pi_bringup motor_commissioning --ros-args \
  -p acknowledgement:=MOTORS_RAISED_CURRENT_LIMITED \
  -p motor_id:=1 \
  -p target_rps:=0.05 \
  -p duration_ms:=500
```

`motor_id` must be 1--4, `target_rps` must be finite with magnitude from 0.01
through 0.25 RPS, and `duration_ms` must be 100--5000. The utility requires
exactly one publisher on the transient-local
`/mentor_pi/configuration/motion_authorization` topic, and that publisher must
be the node `/mentor_pi/configuration_supervisor`. It locks the nonzero packed
configuration-generation/session token before motion. It also requires a fresh
`READY` or `DEGRADED` heartbeat with a nonzero matching session, matching
`ACTIVE` diagnostics, no watchdog indication, and fresh all-zero motor targets
with every measured speed below 0.002 RPS. It refuses to run if any other
motor-command publisher exists.

Every command updates `ALL_MOTORS` at 20 Hz; only the selected target can be
nonzero. A drive publish gap over 100 ms aborts the run, preventing a command
after an expired MCU lease from silently re-arming the motor. The utility also
aborts on an authorization change, heartbeat or diagnostic uptime/sequence
regression, session change, command rejection, watchdog mask/counter change,
selected speed above 0.50 RPS, selected response in the wrong direction, or
unselected motion above 0.02 RPS or by at least two encoder ticks.

A repeated zero phase precedes and follows the bounded drive phase. Success
requires the selected target to be reported, correctly directed physical
response of at least the greater of 0.002 RPS or 10% of the target, at least two
correctly directed selected encoder ticks, and the latest motor state to remain
with all four targets zero and all four `abs(measured_rps)` values strictly
below 0.002 RPS after the stop phase. A target that returns after the
zero-target latch was observed is a failure; zero targets with residual speed
also fail.

`SIGINT` and `SIGTERM` request the bounded stop burst. Exceptions and abnormal
ROS shutdown request an immediate best-effort all-motor zero; if ROS can no
longer publish, the MCU's independent 200 ms lease is the final stop path. Exit
status is nonzero on any failed check. Archive the single outcome log line
containing the session, target, encoder delta, peak/final measured response,
physical-response flag, command counts, and result.

## Call services

Select the JGA27 model (`model: 2`). A real model change returns `BUSY` while
any motor target is nonzero:

```sh
ros2 service call \
  /mentor_pi/motors/set_model \
  mentor_pi_interfaces/srv/SetMotorModel \
  '{model: 2}'
```

Set the low-battery threshold to its default 6300 mV value:

```sh
ros2 service call \
  /mentor_pi/battery/set_low_threshold \
  mentor_pi_interfaces/srv/SetBatteryThreshold \
  '{threshold_mv: 6300}'
```

Read all supported fields (`fields: 511`) from bus servo ID 1. This performs a
physical half-duplex bus transaction and can return `TIMEOUT` when no servo is
connected:

```sh
ros2 service call \
  /mentor_pi/bus_servos/get_state \
  mentor_pi_interfaces/srv/GetBusServoState \
  '{servo_id: 1, fields: 511}'
```

Service responses contain the common numeric result codes documented in the
[ROS interface contract](framework/ros-interface-contract.md). Topic commands
do not have per-message acknowledgements; inspect controller diagnostics for
validation rejection and overwrite counters:

```sh
ros2 topic echo \
  /mentor_pi/diagnostics \
  mentor_pi_interfaces/msg/ControllerDiagnostics
```
