# 08 — Onboard evidence and qualification

Production actions write below `/var/log/mentor-pi/actions` and require the
verified packaged firmware hash:

```zsh
FIRMWARE_SHA256="$(sha256sum \
  firmware/mentor_pi_mcu/build/stm32/mentor_pi_mcu.elf | awk '{print $1}')"

RUNTIME_CONTEXT=production \
PACKAGED_FIRMWARE_SHA256="${FIRMWARE_SHA256}" \
PASSIVE_CHECK_ACK=ACTUATORS_DISCONNECTED \
OLED_PRESENT=1 \
make passive-check
```

Review the generated `SUMMARY.txt`, `command-status.tsv`, `SHA256SUMS`, archive,
and archive digest. Preserve Agent boot/reconnect journals, firmware metadata,
source revision, architecture, board identity, ROS domain, fixture revision,
and supply limit.

After the passive chapters are recorded, use the guarded fixture and root
qualification commands as applicable:

```zsh
RUNTIME_CONTEXT=production PACKAGED_FIRMWARE_SHA256="${FIRMWARE_SHA256}" \
PREFLIGHT_ACK=ACTUATORS_DISCONNECTED make qualification-preflight
```

Campaign commands require their documented fixture variables and exact
acknowledgements; inspect `make help` before use. Do not infer powered motion,
PID performance, endurance, or release qualification from a software build,
mock, passive capture, or green campaign JUnit result. Those claims remain open
until the required instrumented HIL files are recorded and reviewed.
