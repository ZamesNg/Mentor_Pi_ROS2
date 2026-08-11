# 04 — Firmware flash

Flashing runs on the physical host, not inside the Dev Container. Install
STM32CubeProgrammer on that host and ensure `STM32_Programmer_CLI` is on
`PATH`. The firmware artifacts built in Tutorial 02 remain in the shared
repository directory.

Put the MCU in its ROM bootloader state. Keep motor power disconnected, then
run from a host terminal:

```sh
make -C firmware verify
make -C firmware flash \
  PORT=/dev/ttyUSB0 \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

On macOS, use the matching `/dev/cu.*` path instead. If the programmer uses a
different transport selector at your site, follow its documented device
selection while preserving the exact acknowledgement and verified image.

After flashing, return the boot pins to normal, power-cycle only the logic
supply, and leave actuators disconnected. Save the firmware metadata and
reported SHA-256. A successful programmer verification does not authorize
motion.
