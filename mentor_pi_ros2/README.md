# Mentor Pi ROS 2 workspace

This is the conventional ROS 2 Humble workspace for the Mentor Pi host stack.
It contains the interfaces, bringup, and ros2_control hardware packages under
`src/`.

On Ubuntu 22.04 with ROS 2 Humble installed, build and test it directly:

```sh
cd mentor_pi_ros2
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

On other supported Ubuntu releases, use the repository-root `make host` path;
it builds and tests in the pinned Ubuntu 22.04/Humble container. The root Make
targets remain the supported convenience interface on every host.
