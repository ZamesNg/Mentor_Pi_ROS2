# 03 — Passive hardware checks

Keep motor and servo power disconnected. Do not run ROS applications yet.

Inspect the board for damaged wiring, reversed polarity, loose conductors, and
unintended contact. Verify the controller and CH9102F adapter are the devices
you intend to use. With the board connected over USB, list candidate ports:

```sh
# Linux
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true

# macOS
ls -l /dev/cu.*
```

On Linux, inspect the selected device without opening it:

```sh
udevadm info --query=property --name=/dev/ttyUSB0
```

The expected USB identity is vendor `1a86`, product `55d4`. If more than one
matching adapter exists, record its `ID_SERIAL_SHORT` or stable `ID_PATH` and
do not proceed until the intended board is unambiguous.

On Ubuntu, install the stable development alias and dedicated serial group:

```sh
SERIAL_SETUP_ACK=CONFIGURE_SERIAL_ACCESS make serial-setup
```

Log out and back in after the first group change. The Agent service installer
performs its own production identity and udev installation onboard.

Check encoder channel continuity and direction mechanically with actuator power
off. Record each wheel's observed sign. A mismatch is a wiring/configuration
fault; do not compensate by restoring removed firmware direction modes.

Continue only when the fixture is stable, motor power is disconnected, the
serial identity is known, and passive encoder direction agrees with the
documented chassis convention.
