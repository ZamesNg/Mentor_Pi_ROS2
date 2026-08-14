# Mentor Pi micro-ROS Agent

This Linux-only component builds the pinned Humble Agent and the reviewed
CH9102F reset patch independently from the firmware and ROS application
workspace.

On Ubuntu 22.04 amd64 or arm64:

```sh
make setup
make build
make test
make find-device
sudo make install-service ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
```

`make setup` recovers an interrupted initial source fetch when the partial Git
checkout still has the pinned origin and no working-tree state. It continues
to reject origin mismatches, dirty partial checkouts, and existing checkouts at
an unexpected revision.

`find-device` identifies the connected CH9102F by USB vendor/product identity
instead of assuming a tty name. Installation revalidates the same shared
serial-access policy used by the root tutorial façade: it uses stable
`ID_SERIAL_SHORT`, falling back to `ID_PATH`, and refuses a semantically
different existing udev rule. If multiple matching adapters are connected,
rerun installation with the intended `ID_SERIAL_SHORT=...` or `ID_PATH=...`
selector.

Service installation never selects a default ROS domain. Pass the deployment's
`ROS_DOMAIN_ID` explicitly to `make`; change that value when necessary. The
validated value is rendered directly into the installed systemd unit.
`ROS_LOCALHOST_ONLY` defaults to `0` and accepts only `0` or `1`.

For a `zenoh-bridge-ros2dds` deployment that confines DDS to the onboard
computer, install with `ROS_LOCALHOST_ONLY=1`. Use that same value and
`ROS_DOMAIN_ID` for the Agent, bridge, ROS applications, and ROS CLI daemon;
mixing localhost-only and non-local participants separates the local graph.
The Agent remains UDPv4-only, so local cross-user traffic does not use Fast DDS
shared memory. The installer removes the former `/etc/mentor-pi/agent.env`
file.

The default versioned release ID represents the complete installed Agent tree,
so a supporting-file change creates a new side-by-side release even when the
Agent executable itself is unchanged.

macOS and other Linux distributions use the repository VS Code Dev Container
for build and test. Service installation is supported only on the native
Ubuntu 22.04 onboard computer. The service owns only the serial Agent; ROS
applications are started manually from `ros2_ws/`.

The service constrains its Fast DDS participant to UDPv4. Fast DDS shared
memory is owner-restricted on Ubuntu 22.04 and cannot safely carry local samples
between the dedicated `mentor-pi` service account and an interactive ROS user;
UDP loopback preserves that account boundary while allowing discovery and data
delivery. ROS applications keep their normal Fast DDS transport defaults.

At every serial open, the installed service enables the reviewed RRCLite
autoreset patch through its unit-level default
`MENTOR_PI_RRCLITE_AUTORESET=1`. Manual Agent invocations retain the
environment-controlled opt-in so the board-specific sequence is not applied
to an unrelated serial device. The patched Agent performs separate RTS-set,
DTR-clear, and RTS-clear ioctls with the required settle delays so the MCU
starts in normal application mode; a modem-line failure aborts initialization.
