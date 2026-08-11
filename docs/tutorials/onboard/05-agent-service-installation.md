# 05 — Onboard Agent service installation

Identify exactly one connected CH9102F:

```sh
udevadm info --query=property --name=/dev/ttyUSB0
```

Install the built Agent using one stable selector:

```sh
sudo DEVICE=/dev/ttyUSB0 ID_SERIAL_SHORT=YOUR_SERIAL \
  ROS_DOMAIN_ID=0 make -C micro_ros_agent install-service
```

If no unique serial exists, use `ID_PATH=YOUR_STABLE_PATH` and omit
`ID_SERIAL_SHORT`. The installer verifies the live identity, creates
`/dev/mentor_pi_mcu`, installs a versioned root-owned release below
`/opt/mentor_pi/agent/`, and enables the non-root service.

Verify boot and reconnect handling:

```sh
systemctl is-enabled mentor-pi-agent.service
systemctl is-active mentor-pi-agent.service
systemctl status mentor-pi-agent.service --no-pager
journalctl -u mentor-pi-agent.service -n 100 --no-pager
```

Unplug/replug USB once while actuators remain disconnected. The service must
restart and reconnect without starting any ROS application. A device identity
mismatch must not acquire `/dev/mentor_pi_mcu`.
