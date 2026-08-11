# 03 — Onboard passive checks and flash

With logic power only, inspect the connected adapter:

```sh
MENTOR_PI_PORT="$(make -s -C micro_ros_agent find-device)"
printf '%s\n' "${MENTOR_PI_PORT}"
udevadm info --query=property --name="${MENTOR_PI_PORT}"
```

Require vendor `1a86`, product `55d4`, and one stable `ID_SERIAL_SHORT` or
`ID_PATH`. Check board wiring, mechanically verify every encoder direction,
and confirm that motor and servo power remain disconnected.

Install the stable alias and dedicated serial group, then activate the group
immediately in a new shell:

```sh
SERIAL_SETUP_ACK=CONFIGURE_SERIAL_ACCESS make serial-setup
newgrp mentor-pi-serial
id -nG
udevadm info --query=property --name=/dev/mentor_pi_mcu
```

Run the remaining tutorial commands inside that shell; `exit` returns to the
original shell afterward.

Install STM32CubeProgrammer, keep actuator power disconnected, and run:

```sh
make -C firmware flash \
  PORT=/dev/mentor_pi_mcu \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

The default flow uses separate CH9102F RTS/DTR set/clear operations to enter
the ROM bootloader and resets into the application only after read-back
verification. If automatic activation fails before programming, manually use
BOOT/RST and repeat with `AUTOMATIC_BOOT_CONTROL=0`. Preserve the programmer
verification and firmware hash. Do not continue if the connected identity,
passive direction, flash verification, or zero-power fixture is uncertain.
