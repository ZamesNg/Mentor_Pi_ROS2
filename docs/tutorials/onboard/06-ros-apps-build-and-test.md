# 06 — Onboard ROS applications build and test

Import dependencies and build only the five packages under `ros2_ws/src`:

```zsh
make -C ros2_ws clean
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test
```

The clean step prevents generated setup files or C++ artifacts from retaining
a previously sourced ROS workspace. The build then isolates itself to the
supported `/opt/ros/humble` binary underlay.

Confirm the colcon boundary:

```zsh
cd ros2_ws
source /opt/ros/humble/setup.zsh
colcon list --base-paths src
colcon test-result --verbose
cd ..
```

Firmware and Agent sources must not appear in `colcon list`. Resolve all test
failures before runtime. Install the native evidence capture tool:

```zsh
sudo make install-evidence-tools
```

This creates `/opt/mentor_pi/tools/capture_board_diagnostics` and the protected
`/var/log/mentor-pi/actions` tree. It does not enable or start ROS applications.
