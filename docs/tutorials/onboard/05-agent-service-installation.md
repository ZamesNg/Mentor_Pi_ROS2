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
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 \
  ROS_DISCOVERY_SERVER=192.168.2.191:11811
```

The Agent launcher loads its Fast DDS UDPv4 profile from immutable package data
inside the versioned release. The systemd unit records domain, localhost, and
Discovery Server settings but contains no profile or source-checkout path.

This same installation step is safe to rerun when applying an Agent service
definition update: concurrent installer runs are rejected, and it reuses an
existing release only after verifying the complete installed tree still matches
the current build. A newly built release is staged and verified before it is
published; every safe rerun refreshes the service unit and restarts the
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
The environment must contain the configured `ROS_DOMAIN_ID`, selected
`ROS_LOCALHOST_ONLY`, `ROS_DISCOVERY_SERVER`, and
`MENTOR_PI_RRCLITE_AUTORESET=1`. The launcher resolves
`share/micro_ros_agent/config/fastdds.xml` inside the immutable current Agent
release and exports its path only to the real Agent process. The profile
selects UDPv4 and disables built-in transports, preventing the dedicated Agent
account from selecting owner-restricted Fast DDS shared memory.

Unplug/replug USB once while actuators remain disconnected. The service must
restart and reconnect without starting any ROS application. A device identity
mismatch must not acquire `/dev/mentor_pi_mcu`.
