# 08 — Host evidence and qualification

Host and Dev Container builds establish reproducibility only. Preserve:

```zsh
git rev-parse HEAD
sha256sum firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf
make check-compatibility
```

On native Ubuntu 22.04, software-only passive evidence may be collected after
the Agent and manual ROS launch are healthy:

```zsh
sudo make install-evidence-tools
PASSIVE_CHECK_ACK=ACTUATORS_DISCONNECTED \
OLED_PRESENT=1 \
make passive-check
```

Development evidence is stored below `build/`. Production evidence, boot
service validation, recovery cycles, powered motion, and release qualification
belong to the onboard track and an instrumented HIL fixture. A Dev Container
archive is never production evidence.

Record the exact firmware hash, source revision, host architecture, ROS domain,
board identity, fixture revision, supply limit, and test result. Do not state
that PID tuning, wheel direction under power, endurance, or release
qualification passed unless the corresponding HIL files were actually
recorded and reviewed.
