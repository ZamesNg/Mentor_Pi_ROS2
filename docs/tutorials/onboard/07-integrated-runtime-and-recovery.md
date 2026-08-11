# 07 — Integrated runtime and recovery

Keep wheels raised or equivalently guarded and use the current-limited supply.
Verify the external Agent first:

```sh
systemctl is-active mentor-pi-agent.service
```

Start applications manually:

```sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source /etc/mentor-pi/agent.env
export ROS_DOMAIN_ID
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
ros2 launch mentor_pi_hardwares mecanum.launch.py
```

For the Ackermann model, substitute `ackermann.launch.py`. The Agent remains a
separately managed service and does not start this launch.

In another terminal, source the workspace and inspect the safety endpoints:

```sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 node list
ros2 topic echo --once /mentor_pi/heartbeat
ros2 topic echo --once \
  /mentor_pi/configuration/motion_authorization
```

Test these cases while commanded targets are zero:

1. stop the application; the Agent remains active;
2. restart the Agent; the previous authorization becomes invalid;
3. unplug/replug USB; the service reconnects and a new session is configured;
4. stop the Agent; applications remain disarmed and stale feedback is rejected;
5. start the Agent; motion stays zero until supervisor configuration succeeds.

Use `journalctl -u mentor-pi-agent.service` and ROS diagnostics to retain the
session transitions. Never configure systemd to start the ROS application.
