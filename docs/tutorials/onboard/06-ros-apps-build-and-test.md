# 06 — Onboard ROS applications build and test

Import dependencies and build only the five packages under `ros2_ws/src`:

```sh
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test
```

Confirm the colcon boundary:

```sh
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon list --base-paths src
colcon test-result --verbose
cd ..
```

Firmware and Agent sources must not appear in `colcon list`. Resolve all test
failures before runtime. Install the native evidence capture tool:

```sh
sudo make install-evidence-tools
```

This creates `/opt/mentor_pi/tools/capture_board_diagnostics` and the protected
`/var/log/mentor-pi/actions` tree. It does not enable or start ROS applications.
