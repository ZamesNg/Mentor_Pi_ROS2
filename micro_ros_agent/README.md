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
sudo make install-service
```

`find-device` identifies the connected CH9102F by USB vendor/product identity
instead of assuming a tty name. Installation automatically uses its stable
`ID_SERIAL_SHORT`, falling back to `ID_PATH`. If multiple matching adapters are
connected, rerun installation with the intended `ID_SERIAL_SHORT=...` or
`ID_PATH=...` selector.

macOS and other Linux distributions use the repository VS Code Dev Container
for build and test. Service installation is supported only on the native
Ubuntu 22.04 onboard computer. The service owns only the serial Agent; ROS
applications are started manually from `ros2_ws/`.

At every serial open, the installed service enables the reviewed RRCLite
autoreset patch. The patched Agent performs separate RTS-set, DTR-clear, and
RTS-clear ioctls with the required settle delays so the MCU starts in normal
application mode; a modem-line failure aborts Agent initialization.
