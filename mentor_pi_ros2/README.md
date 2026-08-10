# Mentor Pi ROS 2 workspace

This is the conventional ROS 2 Humble workspace for the Mentor Pi host stack.
It contains the firmware and polynomial-trajectory interfaces, bringup,
ros2_control hardware packages, and opt-in mecanum/Ackermann ALTO trackers
under `src/`.

The supported build and test interface on every Ubuntu host is the
repository-root Docker path:

```sh
make setup
make host
```

The single architecture-native project image supplies Ubuntu 22.04, ROS 2
Humble, firmware and Agent tools, `rosdep`, and `colcon`; a host ROS
installation is neither used nor required. Use
`make start` for the development runtime and `make shell ROS_DOMAIN_ID=0` for
the enhanced zsh shell in its running container.

Tracking is disabled by default. After building, opt in with
`TRACKING_CONTROLLER=mecanum make start-mecanum` or
`TRACKING_CONTROLLER=ackermann make start-ackermann`. The host clock must be
synchronized, and the publisher must provide future-scheduled `odom`-frame
polynomial trajectories.
