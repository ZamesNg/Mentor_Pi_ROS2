# 01 — Host prerequisites and safety

This track covers firmware and offboard ROS development from a host computer.
Ubuntu 22.04 may run the component commands natively. macOS and every other
Linux distribution must build and test all three components in the repository
VS Code Dev Container.

Install VS Code, its Dev Containers extension, Git, and Docker Desktop or a
compatible Docker engine. Clone this repository, open its root in VS Code, and
choose **Dev Containers: Reopen in Container**. In the terminal that opens:

```zsh
make doctor
make check-compatibility
```

The Dev Container terminal starts Zsh with Oh My Zsh, completion,
autosuggestions, and syntax highlighting. Press the up arrow or begin typing a
component command to use the seeded build/test history. The seed is idempotent
and does not include flashing, systemd installation, runtime, or HIL actions.

On native Ubuntu 22.04, run the same commands in a normal terminal after ROS 2
Humble is installed. Do not install host ROS on macOS or an unsupported Linux
distribution.

Before connecting actuator power:

- support the chassis so every wheel is clear, or provide equivalent guarding;
- use a current-limited supply and an accessible emergency stop;
- disconnect motor power for build, passive inspection, and flashing;
- confirm the CH9102F adapter is the intended board before changing access;
- stop on unexpected heating, sound, motion, reset, or telemetry.

The checked firmware is the single `NORMAL_CLOSED_LOOP_DEFAULT` ADRC artifact.
No successful build, mock, Dev Container test, or passive observation qualifies
powered motion. Record HIL evidence before making a release claim.

Continue only after both root checks pass and the physical fixture is passive.
