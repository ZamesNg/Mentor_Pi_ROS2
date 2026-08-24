# 04 — Firmware flash

Flashing runs on the physical host, not inside the Dev Container. Install
STM32CubeProgrammer on that host and ensure `STM32_Programmer_CLI` is on
`PATH`. The firmware artifacts built in Tutorial 02 remain in the shared
repository directory.

Keep motor power disconnected, stop any Agent that could own the port, and run
from a physical Linux host terminal:

```zsh
make -C firmware verify
make -C firmware flash \
  PORT=/dev/mentor_pi_mcu \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

The default Linux flow builds the CMake/Ninja CH9102F helper, performs separate
RTS/DTR set/clear operations to enter the ROM bootloader, probes it, programs
and verifies the read-only snapshot, and only then resets into the application.
If automatic activation fails before programming, the physical BOOT/RST
sequence remains the fallback; repeat with `AUTOMATIC_BOOT_CONTROL=0` only
after manually entering the ROM bootloader. On macOS, use that manual fallback
with the matching `/dev/cu.*` path.

After flashing, leave actuators disconnected. Save the firmware metadata and
reported SHA-256. A successful programmer verification and automatic reset do
not authorize motion.

## Flash a package built in the Dev Container

When the firmware was built and packaged on the development host, copy the
entire `firmware-adrc-release/` directory to the physical Ubuntu robot. From a
checkout containing the firmware flash tools, pass that exact directory
instead of rebuilding onboard:

```zsh
make -C firmware flash-package \
  PACKAGE=/path/to/firmware-adrc-release \
  EXPECTED_NAMESPACE=/mecanum_1 \
  PACKAGE_MANIFEST_SHA256=64_HEX_DIGEST_RECORDED_ON_BUILD_HOST \
  PORT=/dev/mentor_pi_mcu \
  FLASH_ACK=ROM_BOOTLOADER_ACTIVE_MOTORS_DISCONNECTED
```

`flash-package` accepts exactly the four release artifacts plus
`BUILD-METADATA.txt`, `BUILD-MODE.txt`, and `SHA256SUMS`. It verifies every
payload digest, the v3 provenance metadata, and the
`NORMAL_CLOSED_LOOP_DEFAULT`/`CLOSED_LOOP` classification before and after
creating the read-only upload snapshot. The separately recorded digest binds
the transferred `SHA256SUMS` to the package produced on the build host, while
`EXPECTED_NAMESPACE` prevents selecting another robot's otherwise-valid
image. The verifier also rejects a non-ARM executable and rechecks the map-file
memory budget. The acknowledgement, exclusive serial ownership,
physical-host restriction, CH9102F boot control, programmer timeout, read-back
verification, and final application reset behavior are identical to `flash`.
