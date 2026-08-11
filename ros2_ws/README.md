# Mentor Pi ROS 2 workspace

This is the conventional ROS 2 Humble workspace for the Mentor Pi host stack.
It contains the firmware and polynomial-trajectory interfaces, bringup,
ros2_control hardware packages, and opt-in mecanum/Ackermann ALTO trackers
under `src/`.

Ubuntu 22.04 builds this workspace natively. macOS and other Linux
distributions use the repository VS Code Dev Container:

```sh
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test
```

`colcon build` run from this directory sees only the five packages below
`src/`; it never builds firmware or the micro-ROS Agent. The Agent is a
separate boot service on the onboard computer, while ROS applications are
started manually.

Tracking is disabled by default. Onboard runtime uses the checked-in ROS launch
files directly after sourcing Humble, this workspace, and
`/etc/mentor-pi/agent.env`, then exporting its `ROS_DOMAIN_ID`. Select
`mecanum.launch.py` or `ackermann.launch.py`; opt into tracking with the matching
`tracking_controller` launch argument. The host clock must be synchronized, and
the publisher must provide future-scheduled `odom`-frame polynomial
trajectories.
