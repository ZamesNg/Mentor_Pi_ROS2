# 04 — Firmware flash

Flashing runs on the physical host, not inside the Dev Container. Install
STM32CubeProgrammer on that host and ensure `STM32_Programmer_CLI` is on
`PATH`. The firmware artifacts built in Tutorial 02 remain in the shared
repository directory.

Keep motor power disconnected, stop any Agent that could own the port, and run
from a physical Linux host terminal:

```sh
make -C firmware verify
make -C firmware flash \
  PORT=/dev/mentor_pi_mcu \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

The default Linux flow builds the CMake/Ninja CH9102F helper, performs separate
RTS/DTR set/clear operations to enter the ROM bootloader, probes it, programs
and verifies the immutable image, and only then resets into the application.
If automatic activation fails before programming, the physical BOOT/RST
sequence remains the fallback; repeat with `AUTOMATIC_BOOT_CONTROL=0` only
after manually entering the ROM bootloader. On macOS, use that manual fallback
with the matching `/dev/cu.*` path.

After flashing, leave actuators disconnected. Save the firmware metadata and
reported SHA-256. A successful programmer verification and automatic reset do
not authorize motion.
