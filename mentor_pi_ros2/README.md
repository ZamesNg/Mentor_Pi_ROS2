# Mentor Pi ROS 2 workspace

This is the conventional ROS 2 Humble workspace for the Mentor Pi host stack.
It contains the interfaces, bringup, and ros2_control hardware packages under
`src/`.

The supported build and test interface on every Ubuntu host is the
repository-root Docker path:

```sh
make setup
make host
```

The architecture-native image supplies Ubuntu 22.04, ROS 2 Humble, `rosdep`,
and `colcon`; a host ROS installation is neither used nor required. Use
`make start` for the development runtime and `make shell ROS_DOMAIN_ID=0` for
the enhanced zsh shell in its running container.
