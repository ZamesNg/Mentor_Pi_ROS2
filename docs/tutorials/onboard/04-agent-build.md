# 04 — Onboard micro-ROS Agent build

Build the Agent independently from the ROS workspace:

```zsh
make -C micro_ros_agent setup
make -C micro_ros_agent build
make -C micro_ros_agent test
```

The component verifies its pinned upstream revisions, applies the CH9102F
DTR/RTS patch, builds the patched Micro-XRCE-DDS-Agent with CMake, and builds
`micro_ros_agent` with colcon. Inspect its metadata:

```zsh
sed -n '1,200p' \
  micro_ros_agent/build/native/install/AGENT-BUILD-METADATA.txt
```

The build is native Ubuntu 22.04/Humble and architecture-specific. Do not copy
an amd64 release to arm64 or vice versa. No ROS application and no firmware is
built by this component.
