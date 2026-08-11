# Mentor Pi micro-ROS Agent

This Linux-only component builds the pinned Humble Agent and the reviewed
CH9102F reset patch independently from the firmware and ROS application
workspace.

On Ubuntu 22.04 amd64 or arm64:

```sh
make setup
make build
make test
sudo make install-service DEVICE=/dev/ttyUSB0 ID_SERIAL_SHORT=BOARD_SERIAL
```

macOS and other Linux distributions use the repository VS Code Dev Container
for build and test. Service installation is supported only on the native
Ubuntu 22.04 onboard computer. The service owns only the serial Agent; ROS
applications are started manually from `ros2_ws/`.
