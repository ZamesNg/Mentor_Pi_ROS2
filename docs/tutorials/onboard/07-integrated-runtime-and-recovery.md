# 07 — Integrated runtime and recovery

Keep wheels raised or equivalently guarded and use the current-limited supply.
Verify the external Agent first:

```zsh
systemctl is-active mentor-pi-agent.service
```

Confirm that firmware-owned LED1 blinks with a 500 ms off/500 ms on system
heartbeat. This indication is independent of ROS. If LED1 remains static, stop:
the firmware peripheral path is not making progress. LED2 and LED3 are the only
LEDs controlled by `/mentor_pi/leds/command`.

With the Agent active and the MCU connected, observe firmware-owned RGB1:

- red toggles after each successful micro-ROS heartbeat publication;
- green pulses when transmitted UART bytes advance;
- blue pulses when received UART bytes advance.

If the ROS graph contains the firmware topics but RGB1 red remains static, stop
here: entity discovery succeeded but the firmware telemetry path is not live.
If LED1 blinks but the graph is absent, repeat Tutorial 05 service installation
and verify the packaged UDPv4 Fast DDS profile plus Discovery Server setting.
Retain the current Agent journal; do not start the applications or enable
actuator power until heartbeat data is visible.

Start applications manually:

```zsh
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
: "${ROS_DOMAIN_ID:?export the deployment ROS_DOMAIN_ID first}"
: "${ROS_LOCALHOST_ONLY:?export the Agent ROS_LOCALHOST_ONLY value first}"
ros2 launch mentor_pi_hardwares vehicle.launch.py
```

The generated profile selects the vehicle type. The Agent remains a separately
managed service and does not start this launch.

In another terminal, source the workspace and inspect the safety endpoints:

```zsh
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
: "${ROS_DOMAIN_ID:?export the deployment ROS_DOMAIN_ID first}"
ros2 node list
ros2 topic echo --once /mentor_pi/heartbeat
ros2 topic echo --once \
  /mentor_pi/configuration/motion_authorization
```

Every ROS terminal must inherit the same exported `ROS_DOMAIN_ID` used for
Agent installation. The service stores that value directly in its installed
unit; there is no separate Agent environment file to source.

Test these cases while commanded targets are zero:

1. stop the application; the Agent remains active;
2. restart the Agent; the previous authorization becomes invalid;
3. unplug/replug USB; the service reconnects and a new session is configured;
4. stop the Agent; hardware components and controllers remain active and
   claimed, while hardware outputs are inhibited and wheel velocities read
   zero;
5. start the Agent; motion stays zero until the supervisor publishes a fresh
   generation for the current session and every required feedback stream has
   supplied a new valid sample.

On Humble 2.54, do not use a recoverable transport or feedback interruption to
force a hardware `ERROR`: that transition removes interfaces and leaves the
component unconfigured without automatically restoring controller claims.
The Mentor Pi hardware plugins instead keep their lifecycle active during
transparent reconnect recovery. Genuine local plugin failures still enter the
normal `ros2_control` error transition and run a complete endpoint teardown so
the component can be configured again.

Use `journalctl -u mentor-pi-agent.service` and ROS diagnostics to retain the
session transitions. Never configure systemd to start the ROS application.
