# 07 — Integrated runtime and recovery

Keep wheels raised or equivalently guarded and use the current-limited supply.
Verify the external Agent first:

```zsh
systemctl is-active mentor-pi-agent.service
```

With the Agent active and the MCU connected, confirm that firmware-owned LED3
toggles. LED3 changes only after a successful ROS heartbeat publication. If
the ROS graph contains the firmware topics but LED3 remains static, stop here:
entity discovery succeeded but the firmware telemetry path is not live. If
LED3 toggles but every firmware topic is silent, repeat Tutorial 05 service
installation and verify its required UDPv4 Fast DDS environment. Retain the
current Agent journal; do not start the applications or enable actuator power
until heartbeat data is visible.

Start applications manually:

```zsh
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
: "${ROS_DOMAIN_ID:?export the deployment ROS_DOMAIN_ID first}"
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
ros2 launch mentor_pi_hardwares mecanum.launch.py
```

For the Ackermann model, substitute `ackermann.launch.py`. The Agent remains a
separately managed service and does not start this launch.

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
4. stop the Agent; applications remain disarmed and stale feedback is rejected;
5. start the Agent; motion stays zero until supervisor configuration succeeds.

Use `journalctl -u mentor-pi-agent.service` and ROS diagnostics to retain the
session transitions. Never configure systemd to start the ROS application.
