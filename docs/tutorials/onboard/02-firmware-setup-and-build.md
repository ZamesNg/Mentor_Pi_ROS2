# 02 — Onboard firmware setup and build

Keep actuator power disconnected. Build natively:

```zsh
make -C firmware setup
make -C firmware test
make -C firmware build
make -C firmware verify
make -C firmware package
```

The build uses CMake/Ninja, the checksummed Arm GNU 13.2.Rel1 toolchain, and the
checked Humble micro-ROS SDK. It does not invoke colcon. The verified package
contains a single `NORMAL_CLOSED_LOOP_DEFAULT` PID ELF/Hex/Bin/Map set.

Record:

```zsh
sha256sum firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf
sed -n '1,200p' \
  firmware/mentor_pi_mcu/build/stm32/rrclite-build-metadata.txt
```

If SDK/interface compatibility fails, stop. Interface changes require SDK
regeneration and review in the same source revision; they must not be bypassed
on the board.
