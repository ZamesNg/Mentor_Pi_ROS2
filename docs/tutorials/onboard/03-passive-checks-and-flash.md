# 03 — Onboard passive checks and flash

With logic power only, inspect the connected adapter:

```sh
udevadm info --query=property --name=/dev/ttyUSB0
```

Require vendor `1a86`, product `55d4`, and one stable `ID_SERIAL_SHORT` or
`ID_PATH`. Check board wiring, mechanically verify every encoder direction,
and confirm that motor and servo power remain disconnected.

Install STM32CubeProgrammer and place the MCU in ROM bootloader mode. Then:

```sh
make -C firmware flash \
  PORT=/dev/ttyUSB0 \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

Return the boot pins to normal and power-cycle logic only. Preserve the
programmer verification and firmware hash. Do not continue if the connected
identity, passive direction, flash verification, or zero-power fixture is
uncertain.
