# 05 — Onboard Agent service installation

Identify exactly one connected CH9102F:

```zsh
make -C micro_ros_agent find-device
```

Install the built Agent. It revalidates the live USB identity and reuses the
same Agent-owned device-access policy configured in Tutorial 03, automatically
choosing `ID_SERIAL_SHORT`, falling back to `ID_PATH`. The installer requires
the deployment's `ROS_DOMAIN_ID` as an explicit argument and refuses to choose
a default:

```zsh
sudo make -C micro_ros_agent install-service \
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
```

Replace the `ROS_DOMAIN_ID` value if the deployment uses a different domain.

If `zenoh-bridge-ros2dds` is the only intended offboard ROS path, confine the
onboard DDS graph to loopback instead:

```zsh
sudo make -C micro_ros_agent install-service \
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
```

Set `ROS_LOCALHOST_ONLY=1` and the same `ROS_DOMAIN_ID` for the bridge, every
onboard ROS application, and every onboard ROS CLI daemon. Confirm that the
loopback interface has multicast enabled with `ip link show lo`. Do not mix
localhost-only and non-local participants within the onboard graph; discovery
may appear partial while samples and services remain separated.

`ROS_LOCALHOST_ONLY` is a ROS/RMW policy that the native micro-ROS Agent does
not interpret directly. The installer therefore selects a managed Fast DDS
UDPv4 participant profile as well: value `1` installs a profile that whitelists
only `127.0.0.1`, while value `0` installs a profile without an interface
whitelist. Rerunning `install-service` applies a profile or unit update without
rebuilding the Agent.

To change an existing deployment back to network-visible DDS, do not edit the
unit or XML profile manually. Rerun the installer with value `0`:

```zsh
sudo make -C micro_ros_agent install-service \
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
```

Export `ROS_LOCALHOST_ONLY=0` for every onboard ROS application and CLI, apply
the same value to the bridge configuration, and restart those processes. Stop
the ROS CLI daemon with `ros2 daemon stop` so its next invocation inherits the
new boundary. The installer selects the network profile and restarts only the
Agent service; no Agent rebuild is required.

This same installation step is safe to rerun when applying an Agent service
definition update: concurrent installer runs are rejected, and it reuses an
existing release only after verifying the complete installed tree still matches
the current build. A newly built release is staged and verified before it is
published; every safe rerun refreshes the service unit and Fast DDS profile,
then restarts the service.

If multiple matching adapters are connected, select the intended board with
`ID_SERIAL_SHORT=YOUR_SERIAL` or `ID_PATH=YOUR_STABLE_PATH`; do not pass a
guessed tty number. The installer creates
`/dev/mentor_pi_mcu`, grants the non-login `mentor-pi` service user access,
installs a versioned root-owned release below
`/opt/mentor_pi/agent/`, and enables the non-root service.

Verify boot and reconnect handling:

```zsh
systemctl is-enabled mentor-pi-agent.service
systemctl is-active mentor-pi-agent.service
systemctl show mentor-pi-agent.service \
  -p DevicePolicy -p DeviceAllow -p Environment
systemctl status mentor-pi-agent.service --no-pager
journalctl -u mentor-pi-agent.service -n 100 --no-pager
```

The device policy must include `char-ttyACM rw`; this preserves clock
protection without blocking `/dev/mentor_pi_mcu` inside the service cgroup.
The environment must contain the configured `ROS_DOMAIN_ID`, selected
`ROS_LOCALHOST_ONLY`, `MENTOR_PI_RRCLITE_AUTORESET=1`, and
`FASTRTPS_DEFAULT_PROFILES_FILE=/etc/mentor-pi/agent-fastdds.xml`. Inspect that
root-owned profile and confirm that a localhost-only deployment contains
`<address>127.0.0.1</address>`. The profile selects UDPv4 and disables built-in
transports, preventing the dedicated Agent account from selecting
owner-restricted Fast DDS shared memory. The environment values are stored
directly in the unit; the installer removes the former
`/etc/mentor-pi/agent.env`.

Unplug/replug USB once while actuators remain disconnected. The service must
restart and reconnect without starting any ROS application. A device identity
mismatch must not acquire `/dev/mentor_pi_mcu`.
