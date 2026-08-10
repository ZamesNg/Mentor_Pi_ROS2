# Tutorial 05: Characterize Board Hardware

Confirm passive encoder direction and the IMU board axes without a debug
probe. This does not command a motor or authorize unguarded motion.

**Run on:** RDK X5 while `mentor-pi-controller.target` remains active
**Hardware state:** verified default PID firmware; motor power disconnected; encoders
connected; servos unplugged; all four wheels raised

Previous: [Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)
Next: [Tutorial 06: ROS 2 CLI Hardware Checkout](06-ros2-cli-hardware-checkout.md)

## 1. Run the guided passive characterization

**Warning:** Disconnect the motor supply and every servo before continuing.
Leave the encoder connectors attached and keep every wheel raised. The PID
firmware must remain installed.

```sh
cd "${HOME}/Mentor_Pi" && make characterize-board
```

Type `ACTUATORS_DISCONNECTED_WHEELS_RAISED`. The helper then asks for four
manual wheel rotations and six stationary board orientations. It prints a
table after every measurement and stores the raw ROS samples and summaries
under `build/diagnostics/characterization-TIMESTAMP/`.

For each prompt, rotate the named physical wheel forward by approximately one
revolution; do not choose it from an assumed M1-M4 position diagram. The helper
discovers which ROS channel changed and records the physical-to-ROS mapping.
The fixed board ownership remains M1/TIM5, M2/TIM2, M3/TIM4, and M4/TIM3, but
the chassis harness determines which physical wheel reaches each board input.
Exactly one unique ROS encoder count must change for every wheel.

The recorded physical-forward delta may be positive on one side and negative
on the other. That is expected for mirrored wheel installation; chassis
software owns the corresponding wheel-specific target signs. The JGA27
1040-count reference is retained as an initial scale, but an approximate hand
rotation is not used as a precision calibration or a pass/fail gate.

For the IMU, the helper samples `/mentor_pi/imu` while you follow the prompts
for +X, -X, +Y, -Y, +Z, and -Z. The physical frame is +X toward the USB-C
edge, +Y toward the PWM-servo edge, and +Z out of the component side. A valid
row has gravity mainly on the named axis with a total magnitude near 9.81
m/s². Missing or inconsistent IMU samples are reported as an explicit
limitation; they do not invalidate a successful encoder-direction test. The
first six-face capture on this board rejected the legacy identity mapping and
measured PCB X = sensor Y, PCB Y = -sensor X, and PCB Z = sensor Z. The current
firmware applies that signed permutation. All six rows must report `PASS` after
the corrected PID image is flashed.

Stop before powered commissioning if a wheel changes no channel, changes more
than one channel, or two physical wheels resolve to the same ROS channel.
Positive and negative physical-forward signs are both recorded rather than
misreported as channel failures. If a prior capture already proves all four
unique channels, the helper copies it into a new immutable evidence directory
and proceeds directly to IMU.

## 2. Use the reference-compatible battery defaults

No multimeter or battery calibration is required for this usable bring-up.
Firmware retains the legacy board settings: PB0/ADC1 channel 8, an 11:1
divider, VREFINT ratio conversion, 0.05 filtering, and a 6300 mV low threshold.
A reading at or below 4900 mV means no battery is present and must not sound the
low-battery alarm. Firmware continues publishing `/mentor_pi/battery/state`;
when a valid battery stays below the threshold, the alarm asserts after 10
seconds and repeats every 10 seconds as in the reference behavior. This command
does not grade that reading. Do not use an uncalibrated ROS voltage as a
precision safety measurement.

Formal battery accuracy, raw-register capture, and instrumented electrical
timing remain release-qualification work in Tutorial 07. They do not block the
guarded CLI checkout when every encoder row passes.

Next: [Tutorial 06: ROS 2 CLI Hardware Checkout](06-ros2-cli-hardware-checkout.md).
