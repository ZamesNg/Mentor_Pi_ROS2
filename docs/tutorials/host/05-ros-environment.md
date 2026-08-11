# 05 — ROS environment

Use the same environment choice as Tutorial 01: native Ubuntu 22.04/Humble, or
the VS Code Dev Container on macOS and other Linux distributions.

Verify that the environment exposes Humble and only the intended workspace:

```sh
make -C ros2_ws doctor
cd ros2_ws
colcon list --base-paths src
cd ..
```

Exactly these five project packages must be listed:

```text
mentor_pi_interfaces
mentor_pi_bringup
mentor_pi_hardwares
mentor_pi_tracking_interfaces
mentor_pi_tracking
```

Import and validate the pinned external ROS source:

```sh
make -C ros2_ws deps
```

The micro-ROS Agent is a separate component and is not brought into this
workspace. Firmware is also outside the colcon graph. Build and test the Agent
independently in this same Ubuntu 22.04 environment (native or Dev Container):

```sh
make -C micro_ros_agent setup
make -C micro_ros_agent build
make -C micro_ros_agent test
```

This offboard Agent build is development evidence only. The onboard service
must be built and installed natively on its own Ubuntu 22.04 architecture.
