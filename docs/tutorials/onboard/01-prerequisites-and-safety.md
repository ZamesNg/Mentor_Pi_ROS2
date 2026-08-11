# 01 — Onboard prerequisites and safety

This production track runs natively on Ubuntu 22.04 amd64 or arm64 with ROS 2
Humble. Do not use the Dev Container onboard.

Install Git, build-essential, CMake, Ninja, vcs, rosdep, colcon, udev, and ROS 2
Humble. Clone this repository and run:

```sh
make doctor
make check-compatibility
```

Before continuing, disconnect actuator power, support the chassis with every
wheel clear or equivalently guarded, configure a current-limited supply, and
make an emergency stop reachable. Inspect wiring and polarity. Stop on any
unexpected heat, sound, motion, reset, or telemetry.

The firmware has one normal closed-loop PID classification. Software gates do
not replace the physical precautions. Tutorials 01–05 remain passive; powered
motion requires their recorded completion and the guarded HIL procedure.
