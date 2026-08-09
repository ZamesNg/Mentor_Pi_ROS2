# Onboard Computer Tutorial 03: Build and Run Native ROS 2 Humble

Build the standard Humble workspace with `rosdep` and `colcon`, then start the
Agent and configuration supervisor with a direct `ros2 launch` command.

**Run on:** onboard computer (RDK X5), Ubuntu 22.04 arm64
**Hardware state:** verified default PID firmware; all actuators disconnected

Previous: [Onboard Computer Tutorial 02: Build and Flash the Default PID Firmware](02-build-and-flash-default-pid-firmware.md)
Next: [Onboard Computer Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md)

## 1. Resolve native dependencies

```sh
cd /home/zames/Mentor_Pi/mentor_pi_ros2
source /opt/ros/humble/setup.zsh
rosdep check --from-paths src --ignore-src --rosdistro humble
```

If `rosdep check` reports a dependency, return to Tutorial 01 and rerun the
preparation helper. Do not install packages ad hoc or continue with unresolved
dependencies.

## 2. Build and test with colcon

```sh
cd /home/zames/Mentor_Pi/mentor_pi_ros2
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test
colcon test-result --verbose
cd /home/zames/Mentor_Pi
./tools/onboard_colcon_state.sh record
```

Require all three packages to build and every test to pass. The conventional
workspace outputs are `mentor_pi_ros2/build`, `install`, and `log`.
The final command binds the successful install to the current tracked host
inputs so later safety helpers reject a stale build.

## 3. Build the pinned Agent natively

```sh
cd /home/zames/Mentor_Pi
make agent
```

The source-locked micro-ROS Agent builds directly against native Humble. Stop
on a source-lock, patch, dependency, compiler, or executable-hash failure.

## 4. Start the native PID runtime

**Warning:** Disconnect motor power, all four PWM servos, and all bus servos.
Contain every wheel; the PID firmware accepts guarded nonzero commands.

```sh
cd /home/zames/Mentor_Pi
source tools/setup_onboard_ros_environment.zsh
export ROS_DOMAIN_ID=0
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
  ros2 launch mentor_pi_bringup controller.launch.py \
    serial_device:=/dev/mentor_pi_mcu \
    agent_executable:="${MENTOR_PI_AGENT_EXECUTABLE}"
```

The sourced adapter verifies the conventional colcon install, Agent, and PID
artifact and exports only the launch preflight paths. The Python launch checks
the CH9102F identity and ownership, starts the Agent at 1,000,000 baud/8N1 and
the supervisor together, and shuts down if either exits.

## 5. Inspect from a second native terminal

```sh
cd /home/zames/Mentor_Pi
source tools/setup_onboard_ros_environment.zsh
export ROS_DOMAIN_ID=0
ros2 node list
```

Require `/mentor_pi/controller` and `/mentor_pi/configuration_supervisor`.
Keep the launch terminal open through the connected checks.

Next: [Onboard Computer Tutorial 04: Run Passive Board Bring-Up](04-run-passive-board-bringup.md).
