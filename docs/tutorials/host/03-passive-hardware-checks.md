# 03 — Passive hardware checks

Keep motor and servo power disconnected. Do not run ROS applications yet.

Inspect the board for damaged wiring, reversed polarity, loose conductors, and
unintended contact. Verify the controller and CH9102F adapter are the devices
you intend to use. With the board connected over USB, locate it by USB
identity:

```zsh
# Linux
MENTOR_PI_PORT="$(make -s -C micro_ros_agent find-device)"
printf '%s\n' "${MENTOR_PI_PORT}"
udevadm info --query=property --name="${MENTOR_PI_PORT}"

# macOS
ls -l /dev/cu.*
```

The Linux discovery command requires vendor `1a86`, product `55d4`, and
exactly one matching tty. If it lists multiple candidates and stops, identify
the intended board by its reported `ID_SERIAL_SHORT` or stable `ID_PATH` and
disconnect the others before continuing.

On Ubuntu, install the stable development alias and dedicated serial group:

```zsh
SERIAL_SETUP_ACK=CONFIGURE_SERIAL_ACCESS make serial-setup
```

This invokes the Agent-owned identity-based device-access policy and creates
`/dev/mentor_pi_mcu`; it does not depend on the transient kernel tty number.

After the first group change, activate it immediately in a new shell and
confirm that shell's groups:

```zsh
newgrp mentor-pi-serial
id -nG
```

Run the remaining commands inside that shell. Type `exit` when finished to
return to the original shell. Onboard service installation revalidates and
reuses this same device-access policy for its non-login service user.

Check encoder channel continuity and direction mechanically with actuator power
off. Record each wheel's observed sign. A mismatch is a wiring/configuration
fault; do not compensate by restoring removed firmware direction modes.

Continue only when the fixture is stable, motor power is disconnected, the
serial identity is known, and passive encoder direction agrees with the
documented chassis convention.
