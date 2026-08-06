# RRCLite v2 handoff and next steps

This is the short current-state handoff. Start with the top-level `README.md`
for normal use and consult `docs/framework/` only when exact contract details
are needed.

## Current software status

- The active stack is Ubuntu 22.04, ROS 2 Humble, micro-ROS, FreeRTOS, STM32
  HAL, and C++17. The developer entry point is the root `Makefile`.
- The default STM32F407 firmware is motor-locked. Its reproducible Humble
  micro-ROS build, artifact validation, memory-headroom checks, native tests,
  fuzz smoke tests, and direct CubeProgrammer wrapper have passed software-only
  verification.
- The arm64 Humble host build and complete `make test` gate passed. The pinned
  arm64 micro-ROS Agent source build and compatibility-evidence gate also
  passed.
- The amd64 Humble host build passed. The long emulated amd64 Agent build was
  deliberately deferred at the user's request; run it later on native amd64
  hardware if it is still useful.
- Software tests do not qualify motor polarity, PID tuning, analog scaling,
  IMU axes, peripheral timing, watchdog timing, or real USB reconnect behavior.

## Start on the new development computer

Ubuntu 24.04 should have Git, Make, Docker Engine, and optionally
STM32CubeProgrammer, but no native ROS installation. After transferring and
checking out the repository, run:

```sh
make doctor
make setup HOST_ARCH=arm64        # use amd64 on a native amd64 computer
make firmware
make host HOST_ARCH=arm64
make agent HOST_ARCH=arm64        # optional if existing evidence is sufficient
make test HOST_ARCH=arm64         # release checkpoint, not every small edit
```

`make setup` recreates the ignored pinned dependency checkouts and pulls the
architecture-matched build images. Firmware and host build outputs, generated
micro-ROS files, dependencies, logs, and Agent evidence are intentionally not
in Git.

There is currently no Git remote. Transfer the committed repository with a Git
bundle, archive, or other trusted copy, then verify `git log`, `git status`, and
the current branch before continuing. Except for its tracked policy README, the
ignored `docs/reference/` legacy snapshot must be copied separately if it is
needed; normal builds do not use it.

## First real-hardware sequence

1. Keep actuators disconnected and do not apply unrestricted motor power.
2. Build the normal motor-locked firmware with `make firmware`.
3. Connect the USB-C serial/download connector through CH9102F. Hold `BOOT`,
   tap `RST`, release `BOOT`, and identify its exact `/dev/serial/by-id/...`
   device.
4. Flash through the STM32 factory USART1 bootloader:

   ```sh
   make flash PORT=/dev/serial/by-id/REPLACE_ME \
     FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
   ```

5. Reset normally, start the Humble Agent, and confirm heartbeat, diagnostics,
   battery, buttons, IMU, and other passive telemetry. Test Agent restart and
   physical USB disconnect/reconnect while motors remain unavailable.
6. Record passive encoder direction for every motor. Correct and retest any
   polarity discrepancy before enabling commissioning motion.
7. With wheels raised or equivalently guarded and the supply current-limited,
   follow `docs/flashing-and-first-bringup.md` to build and flash the capped
   commissioning image. Characterize each motor separately before PID tuning.
8. Complete functional HIL for every retained peripheral, then the documented
   500 Hz/60-minute stress, reconnect/reset cycles, fault injection, stack
   measurements, traffic budget, and 24-hour soak. Keep the normal firmware
   motor-locked until the release gates have evidence.

Record failures and measurements in
`docs/board-arrival-bringup-checklist.md`; do not mark a hardware gate complete
from software-only or mocked evidence.
