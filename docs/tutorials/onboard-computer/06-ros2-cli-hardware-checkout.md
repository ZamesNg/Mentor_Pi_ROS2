# Onboard Computer Tutorial 06: ROS 2 CLI Hardware Checkout

Exercise the Mentor Pi MCU interfaces directly with standard ROS 2 Humble
commands. This is a guarded lab-bench reference, not a substitute for recorded
HIL qualification.

**Run on:** the connected Humble host with the Agent and configuration
supervisor running
**Hardware state:** motor and servo power disconnected for Sections 1--5 and
9. Before Sections 6--8, complete Tutorials 01--05, raise or equivalently
guard every wheel, use a current-limited supply, keep a physical motor-power
stop reachable, and ensure every servo linkage is free to move without binding.

Previous: [Tutorial 05: Characterize Board Hardware](05-characterize-board-hardware.md)
Next: [Tutorial 07: Run Stress, Soak, and Release Gates](07-run-stress-soak-and-release-gates.md)

## 1. Start the graph and inspect the interfaces

Start a fresh validated native launch in terminal A:

```sh
cd /home/zames/Mentor_Pi
source tools/setup_onboard_ros_environment.zsh
export ROS_DOMAIN_ID=0
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  ros2 launch mentor_pi_bringup controller.launch.py \
    serial_device:=/dev/mentor_pi_mcu \
    agent_executable:="${MENTOR_PI_AGENT_EXECUTABLE}"
```

The environment adapter and Python launch validate the PID artifact, CH9102F
identity, serial ownership, native build prefixes, Agent executable, and exact
safety acknowledgement. The launch stops completely if either critical process
exits.

Open a second terminal for the commands below:

```sh
cd /home/zames/Mentor_Pi
source tools/setup_onboard_ros_environment.zsh
export ROS_DOMAIN_ID=0
```

Require the configuration supervisor's transient-local motion gate to be true
before any powered section:

```sh
ros2 topic echo --once --qos-reliability reliable \
  --qos-durability transient_local \
  /mentor_pi/configuration/motion_enabled std_msgs/msg/Bool
```

Confirm the MCU heartbeat:

```sh
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/heartbeat mentor_pi_interfaces/msg/Heartbeat
```

Expected: `state: 1` (`READY`) and a nonzero `agent_session_id`. The normal
healthy idle flags include `TIME_SYNCHRONIZED=1` and, when the IMU is healthy,
`IMU_HEALTHY=8`. `MOTOR_WATCHDOG_ACTIVE=2` is not an idle-health flag: it is
set after a nonzero motor lease expires and clears after a later accepted
command for that channel. `LOW_BATTERY=4` and `BUS_SERVO_BUSY=16` describe
their corresponding current conditions.

Inventory and introspection commands:

```sh
ros2 topic list | grep '^/mentor_pi/'
ros2 service list | grep '^/mentor_pi/'
ros2 node info /mentor_pi/configuration_supervisor
ros2 interface show mentor_pi_interfaces/msg/MotorCommand
ros2 interface show mentor_pi_interfaces/srv/SetMotorPid
```

## 2. Discrete LEDs

LED1 and LED2 are host-controlled. LED3 is reserved for successful heartbeat
publication and rejects host commands.

Blink LED1 three times, then turn it off:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 1, on_time_ms: 200, off_time_ms: 300, repeat: 3}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 1, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
```

For LED patterns, `on_time_ms=0` means off; nonzero on-time with
`off_time_ms=0` means steady on; and `repeat=0` with both times nonzero means
repeat until replaced.

## 3. Host RGB pixel

Only RGB1 is host-controlled. The message retains `PIXEL_2=2` and
`ALL_PIXELS=3` for wire compatibility, but masks 2 and 3 are rejected because
RGB2 reports firmware RX/TX activity.

Set RGB1 to dim green, then turn it off:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
  '{update_mask: 1, red: [0, 0], green: [48, 0], blue: [0, 0]}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
  '{update_mask: 1, red: [0, 0], green: [0, 0], blue: [0, 0]}'
```

## 4. Buzzer

Play two short 1 kHz beeps, then replace the host pattern with silence:

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
  '{frequency_hz: 1000, on_time_ms: 150, off_time_ms: 150, repeat: 2}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
  '{frequency_hz: 0, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
```

Valid tones are 10--20000 Hz. A low-battery alarm has priority over the host
pattern; diagnose the battery state instead of repeatedly silencing an active
alarm.

## 5. Optional OLED

Skip this section when the board has no SSD1306 display installed. Each
selected line accepts at most 23 printable ASCII bytes.

```sh
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/oled/command mentor_pi_interfaces/msg/OledCommand \
  '{update_mask: 3, line_1: "Mentor Pi CLI", line_2: "Hardware checkout"}'
```

The battery indication on the bottom display page remains controller-owned.

## 6. Motors and PID tuning

**Warning:** Keep the direct `ros2 launch mentor_pi_bringup controller.launch.py`
from Section 1 running; do not start a second runtime or a ros2_control vehicle
launch. Verify that no other process publishes `/mentor_pi/motors/command`.
Wheels remain raised or equivalently guarded; supply is current-limited; the
physical stop is reachable; and exactly one channel moves first below 1 RPS.

Inspect publishers and open a motor-state watch in another sourced shell:

```sh
ros2 topic info --verbose /mentor_pi/motors/command
ros2 topic echo --qos-reliability best_effort --qos-durability volatile \
  /mentor_pi/motors/state mentor_pi_interfaces/msg/MotorState
```

Before starting a publisher, the verbose topic report should show the MCU
subscriber and no existing command publisher.

### 6.1 Confirm the motor profile and command zero

The checked-in board configuration uses JGA27 (`model: 2`). Do not select a
different model unless it matches the installed motor and reviewed encoder
profile. An actual model change resets PID overrides and is rejected while a
target is nonzero.

```sh
ros2 service call /mentor_pi/motors/set_model \
  mentor_pi_interfaces/srv/SetMotorModel '{model: 2}'
ros2 topic pub --once --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 15, target_rps: [0.0, 0.0, 0.0, 0.0]}'
```

Expected service response: `result.code: 0`, `active_model: 2`,
`ticks_per_revolution: 1040`, and `max_rps: 6.0`.

### 6.2 Run one bounded motor stream

The independent lease expires at 198 ms, so a one-shot nonzero command stops
almost immediately. Publish M1 at 20 Hz for five seconds:

```sh
timeout 5 ros2 topic pub --rate 20 --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 1, target_rps: [0.5, 0.0, 0.0, 0.0]}'
```

Expected: only the wheel mapped to M1 in Tutorial 05 moves. Stop immediately
on wrong direction, another moving channel, oscillation, unusual current,
noise, heat, or vibration. The measured speed should respond on index 0, but
no tolerance or PID-performance claim is made without recorded HIL evidence.

After `timeout` ends, M1 stops within the lease bound and
`watchdog_stop_mask` contains bit 1. Publish zero to clear that bit:

```sh
ros2 topic pub --once --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 15, target_rps: [0.0, 0.0, 0.0, 0.0]}'
```

### 6.3 Observe atomic rejection

JGA27 permits at most 6 RPS. A selected 8 RPS target returns
`OUT_OF_RANGE` internally, changes no target, and refreshes no lease. Topic
publishers receive no service-style response, so compare diagnostics before
and after:

```sh
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics
ros2 topic pub --once --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 1, target_rps: [8.0, 0.0, 0.0, 0.0]}'
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics
```

Expected: `command_rejections` and M1's `motor_command_rejections[0]`
increase, while targets and output remain zero.

### 6.4 Apply and tune PID values

The provisional firmware defaults are Kp 250.0 permille/RPS, Ki 0.1
permille/(RPS second), Kd 0.5 permille-second/RPS, and new-sample filter weight
0.5. They are bounded starting values, not physically qualified gains.

PID changes are allowed only when every channel is disarmed, every target is
zero, and every measured speed magnitude is below 0.01 RPS. Wait for coastdown,
then explicitly apply the current baseline to M1:

```sh
ros2 service call /mentor_pi/motors/set_pid \
  mentor_pi_interfaces/srv/SetMotorPid \
  '{update_mask: 1,
    proportional_gain: [250.0, 0.0, 0.0, 0.0],
    integral_gain: [0.1, 0.0, 0.0, 0.0],
    derivative_gain: [0.5, 0.0, 0.0, 0.0],
    velocity_filter_new_weight: [0.5, 0.0, 0.0, 0.0]}'
```

Expected: `result.code: 0` and `applied_mask: 1`. The update is atomic:
every failure returns `applied_mask: 0`; it never returns a partial PID update.
A zero or malformed mask is `INVALID_ARGUMENT`, a finite value outside the P/I/D
range 0--1000 or filter range 0--1 is `OUT_OF_RANGE`, and moving/coasting
channels produce `BUSY`.

For a reviewed tuning trial, change one selected M1 value at a time, repeat the
bounded 0.5 RPS stream, record target/measured state and diagnostics, stop and
wait below 0.01 RPS, then make the next change. Stop on the first unstable or
unexpected response. Restore all channels to the provisional defaults before
ending an exploratory session:

```sh
ros2 service call /mentor_pi/motors/set_pid \
  mentor_pi_interfaces/srv/SetMotorPid \
  '{update_mask: 15,
    proportional_gain: [250.0, 250.0, 250.0, 250.0],
    integral_gain: [0.1, 0.1, 0.1, 0.1],
    derivative_gain: [0.5, 0.5, 0.5, 0.5],
    velocity_filter_new_weight: [0.5, 0.5, 0.5, 0.5]}'
```

Overrides survive an Agent reconnect but are cleared by an MCU reset or an
actual motor-model change.

## 7. PWM servos

**Warning:** PWM servos hold their last accepted output across host loss. Keep
linkage free, begin at the reviewed neutral, and disconnect servo power if a
mechanism approaches a stop.

Read the current state, center all channels, then move Servo 1:

```sh
ros2 topic echo --once --qos-reliability best_effort \
  --qos-durability volatile \
  /mentor_pi/pwm_servos/state mentor_pi_interfaces/msg/PwmServoState
ros2 topic pub --once --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/pwm_servos/command mentor_pi_interfaces/msg/PwmServoCommand \
  '{update_mask: 15, duration_ms: 500, pulse_width_us: [1500, 1500, 1500, 1500]}'
ros2 topic pub --once --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/pwm_servos/command mentor_pi_interfaces/msg/PwmServoCommand \
  '{update_mask: 1, duration_ms: 1000, pulse_width_us: [1400, 0, 0, 0]}'
```

Selected pulse widths must be 500--2500 microseconds and duration must be
20--30000 ms. Do not treat that electrical range as a safe mechanical range.

Set and then restore bounded offsets (each must be -100 through +100 us):

```sh
ros2 service call /mentor_pi/pwm_servos/set_offsets \
  mentor_pi_interfaces/srv/SetPwmServoOffsets \
  '{update_mask: 1, offset_us: [20, 0, 0, 0]}'
ros2 service call /mentor_pi/pwm_servos/set_offsets \
  mentor_pi_interfaces/srv/SetPwmServoOffsets \
  '{update_mask: 15, offset_us: [0, 0, 0, 0]}'
```

Return Servo 1 to 1500 us before leaving this section.

## 8. UART bus servos

**Warning:** Confirm the connected servo ID and free linkage before enabling
torque. Bus writes are not automatically replayed after reconnect or reset.

Read ID 1 using all fields, enable torque, and move to position 500:

```sh
ros2 service call /mentor_pi/bus_servos/get_state \
  mentor_pi_interfaces/srv/GetBusServoState '{servo_id: 1, fields: 511}'
ros2 service call /mentor_pi/bus_servos/configure \
  mentor_pi_interfaces/srv/ConfigureBusServo \
  '{servo_id: 1, update_mask: 64, torque_enabled: true}'
ros2 topic pub --once --qos-reliability best_effort \
  --qos-durability volatile --qos-depth 1 \
  /mentor_pi/bus_servos/command mentor_pi_interfaces/msg/BusServoCommand \
  '{count: 1,
    servo_id: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    position: [500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    duration_ms: 1000}'
```

Valid positions are 0--1000. `PARTIAL` is possible for multi-frame bus reads,
configuration, and stop operations after at least one frame succeeds. `BUSY`
means another bus service owns the shared slot; wait for that operation to
finish rather than retrying in a tight loop.

Send the protocol stop frame, then disable torque:

```sh
ros2 service call /mentor_pi/bus_servos/stop \
  mentor_pi_interfaces/srv/StopBusServos \
  '{count: 1, servo_id: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'
ros2 service call /mentor_pi/bus_servos/configure \
  mentor_pi_interfaces/srv/ConfigureBusServo \
  '{servo_id: 1, update_mask: 64, torque_enabled: false}'
```

## 9. Read-only telemetry and battery threshold

```sh
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/battery/state mentor_pi_interfaces/msg/BatteryState
ros2 topic hz --qos-reliability best_effort /mentor_pi/imu
ros2 topic echo --once --qos-reliability best_effort \
  /mentor_pi/imu sensor_msgs/msg/Imu
ros2 topic echo --qos-reliability reliable \
  /mentor_pi/buttons/events mentor_pi_interfaces/msg/ButtonEvent
ros2 topic echo --once --qos-reliability reliable \
  /mentor_pi/diagnostics mentor_pi_interfaces/msg/ControllerDiagnostics
```

The configured low-battery threshold is 6300 mV. This idempotent call tests
the service without silently changing the deployment safety setting:

```sh
ros2 service call /mentor_pi/battery/set_low_threshold \
  mentor_pi_interfaces/srv/SetBatteryThreshold '{threshold_mv: 6300}'
```

Expected: `result.code: 0` and `active_threshold_mv: 6300`. Battery values are
not precision safety measurements until calibrated against an instrument.

## 10. Bench teardown

Stop and disarm actuators before disconnecting the ROS session:

```sh
ros2 topic pub --once --qos-reliability best_effort --qos-depth 1 \
  /mentor_pi/motors/command mentor_pi_interfaces/msg/MotorCommand \
  '{update_mask: 15, target_rps: [0.0, 0.0, 0.0, 0.0]}'
ros2 topic pub --once --qos-reliability best_effort --qos-depth 1 \
  /mentor_pi/pwm_servos/command mentor_pi_interfaces/msg/PwmServoCommand \
  '{update_mask: 15, duration_ms: 50, pulse_width_us: [1500, 1500, 1500, 1500]}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 1, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/leds/command mentor_pi_interfaces/msg/LedCommand \
  '{led_id: 2, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/rgb/command mentor_pi_interfaces/msg/RgbCommand \
  '{update_mask: 1, red: [0, 0], green: [0, 0], blue: [0, 0]}'
ros2 topic pub --once --qos-reliability reliable --qos-depth 1 \
  /mentor_pi/buzzer/command mentor_pi_interfaces/msg/BuzzerCommand \
  '{frequency_hz: 0, on_time_ms: 0, off_time_ms: 0, repeat: 0}'
```

Then issue the bus-servo stop and torque-disable calls from Section 8, remove
actuator power, and stop the launch with Ctrl-C. PWM and bus servos hold their
last accepted state on host loss, so loss of ROS communication is not an
actuator-neutral command.
