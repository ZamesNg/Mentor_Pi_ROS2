# 07 — Connect and run

ROS applications start manually. The serial Agent must run as a native service
on an Ubuntu 22.04 onboard computer; never use the Dev Container as the robot
runtime.

On that onboard computer, clone the same revision and run:

```zsh
make -C micro_ros_agent setup
make -C micro_ros_agent build
make -C micro_ros_agent test
```

With exactly one connected CH9102F, install by stable identity:

```zsh
make -C micro_ros_agent find-device
sudo make -C micro_ros_agent install-service \
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
systemctl is-enabled mentor-pi-agent.service
systemctl is-active mentor-pi-agent.service
journalctl -u mentor-pi-agent.service -n 50 --no-pager
```

Replace the `ROS_DOMAIN_ID` value if the deployment uses a different domain.
For a Zenoh bridge that confines DDS to the onboard computer, install with
`ROS_LOCALHOST_ONLY=1` and use that same value for the Agent, bridge, onboard
applications, and onboard ROS CLI daemon. The installer makes this effective
for the native micro-ROS Agent by installing a Fast DDS UDPv4 profile whose
interface whitelist contains only `127.0.0.1`.

The installer discovers USB `1a86:55d4` and chooses its stable serial, falling
back to `ID_PATH`. If multiple matching adapters are connected, select the
intended one with `ID_SERIAL_SHORT=...` or `ID_PATH=...`. Build the ROS
workspace natively on that computer as in Tutorial 06.

After the passive gates are complete, the wheels are raised or equivalently
guarded, and the supply is current-limited, start the applications manually:

```zsh
source /opt/ros/humble/setup.zsh
source ros2_ws/install/setup.zsh
: "${ROS_DOMAIN_ID:?export the deployment ROS_DOMAIN_ID first}"
: "${ROS_LOCALHOST_ONLY:?export the Agent ROS_LOCALHOST_ONLY value first}"
ros2 launch mentor_pi_hardwares mecanum.launch.py
```

The Agent service must not start this launch. Stop the application with
Ctrl-C. Disconnecting or restarting the Agent must invalidate the old session,
disarm motion, and require the supervisor to configure the new session.

For the Ackermann model, substitute `ackermann.launch.py`.
