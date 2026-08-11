# 02 — Firmware setup and build

Run this chapter inside the Dev Container, or natively on Ubuntu 22.04:

```sh
make -C firmware setup
make -C firmware test
make -C firmware build
make -C firmware verify
make -C firmware package
```

`setup` obtains the pinned STM32 sources and checksummed Arm GNU 13.2.Rel1
toolchain. CMake and Ninja remain authoritative. `extract-sdk` verifies the
checked Humble micro-ROS SDK before compilation; it does not build the ROS
workspace.

The package is written below
`firmware/build/packages/<UTC>/firmware-pid-release/` unless `PACKAGE_OUTPUT`
is set. It contains one ELF/Hex/Bin/Map set plus build mode, metadata, and
hashes.

The SDK manifest binds the editable interfaces under
`ros2_ws/src/mentor_pi_interfaces`, the generated SDK tree, archive, upstream
source locks, and toolchain. If interfaces change, verification fails until a
maintainer regenerates the SDK on Ubuntu 22.04/Humble:

```sh
make -C firmware microros-sdk
make -C firmware build verify
```

Run SDK generation as the normal developer user, never through `sudo`. The VS
Code Dev Container initializes rosdep for that user and copies any micro-ROS
helper that must be adjusted into the disposable generation tree. If rosdep is
reported unavailable, choose **Dev Containers: Rebuild and Reopen in
Container** before retrying.

Commit the interface change and regenerated SDK in the same commit. Do not
accept a stale SDK by copying generated headers into the firmware source tree.
