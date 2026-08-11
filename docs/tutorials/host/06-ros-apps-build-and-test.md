# 06 — ROS applications build and test

Build and test the workspace in the native Ubuntu 22.04 terminal or Dev
Container terminal:

```zsh
make -C ros2_ws build
make -C ros2_ws test
```

The equivalent standard workflow is:

```zsh
cd ros2_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
colcon test
colcon test-result --verbose
cd ..
```

No firmware or Agent source is compiled by these commands. Resolve all test
failures before connecting a board. In particular, interface, publisher
ownership, session authorization, stale-feedback, invalid-command, and launch
contract failures are safety failures rather than optional lint.

The Dev Container result is development evidence only. It cannot be cited as
onboard installation, boot-service, hardware, or release evidence.
