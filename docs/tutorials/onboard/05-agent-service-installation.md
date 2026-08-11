# 05 — Onboard Agent service installation

Identify exactly one connected CH9102F:

```zsh
make -C micro_ros_agent find-device
```

Install the built Agent. It revalidates the live USB identity and reuses the
same Agent-owned device-access policy configured in Tutorial 03, automatically
choosing `ID_SERIAL_SHORT`, falling back to `ID_PATH`. The installer requires
the deployment's `ROS_DOMAIN_ID` to already be exported and refuses to choose a
default:

```zsh
: "${ROS_DOMAIN_ID:?export the deployment ROS_DOMAIN_ID first}"
sudo --preserve-env=ROS_DOMAIN_ID \
  make -C micro_ros_agent install-service
```

This same installation step is safe to rerun when applying an Agent service
definition update: concurrent installer runs are rejected, and it reuses an
existing release only after verifying the complete installed tree still matches
the current build. A newly built release is staged and verified before it is
published; every safe rerun refreshes the service files and restarts the
service.

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
The environment must contain the exported `ROS_DOMAIN_ID`,
`MENTOR_PI_RRCLITE_AUTORESET=1`, and
`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`. The last setting prevents the dedicated
Agent account from selecting owner-restricted Fast DDS shared memory for
samples consumed by the interactive ROS user. These values are stored directly
in the unit; the installer removes the former `/etc/mentor-pi/agent.env`.

Unplug/replug USB once while actuators remain disconnected. The service must
restart and reconnect without starting any ROS application. A device identity
mismatch must not acquire `/dev/mentor_pi_mcu`.
