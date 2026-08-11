# RRCLite v2 status and next steps

This is the implementation and evidence handoff as of 2026-08-11. Begin with
one complete tutorial track from the [root README](../README.md).

## Current source contract

The repository is a native component monorepo:

- `firmware/` builds one PID artifact independently with CMake/Ninja;
- `micro_ros_agent/` owns pinned sources, the CH9102F patch, native build,
  versioned `/opt/mentor_pi/agent/` installation, udev, and the non-root
  `mentor-pi-agent.service`;
- `ros2_ws/` contains the five ROS packages and uses rosdep/vcs/colcon only;
- the Agent is an external boot prerequisite, while ROS applications start
  manually and shut down when their supervisor fails;
- Ubuntu 22.04 amd64/arm64 is the sole native production platform;
- macOS and other Linux distributions build/test all components in the VS Code
  Dev Container; and
- all production Docker, dev-runtime, OCI, QEMU, and container handoff paths
  are removed.

`mentor_pi_interfaces` is still editable under `ros2_ws/src`. The checked
firmware SDK currently records:

```text
interfaces_sha256=b52c43925d1dc41b7c98ceeb965cc84d67a6c22d5a84e22b5d4f438330bafbf4
archive_sha256=1aed7ed2dc7e98244762552aecacae6bde27d1c38f54f6231e9563618b21a9d9
tree_sha256=bbd42d209c9c75fb2b75b2f9ead22c9629aaaa4c24c0f851e273044c9b08f7ac
toolchain_amd64_sha256=6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb
toolchain_arm64_sha256=8fd8b4a0a8d44ab2e195ccfbeef42223dfb3ede29d80f14dcf2183c34b8d199a
```

Compatibility validation rejects interface edits until the generated SDK is
regenerated and committed with the change.

## Next validation session

Perform these in order on native Ubuntu 22.04:

1. Run `make check-compatibility` and all three component `doctor` targets.
2. On amd64, run firmware `setup`, `test`, `build`, `verify`, and `package`.
   Repeat the build/verification on native arm64. Record toolchain, SDK, source,
   and artifact hashes.
3. Mutate an interface only in a disposable worktree and prove firmware SDK
   validation rejects it. Do not commit that mutation.
4. Build and test the Agent on native amd64 and arm64. Verify its upstream
   revisions and patched DTR/RTS behavior.
5. Install the Agent on an Ubuntu 22.04 board by stable CH9102F identity. Record
   boot startup, non-root ownership, hardening, unplug/replug reconnect, and
   repeated restart behavior.
6. Run `colcon list/build/test/test-result` in `ros2_ws` and prove that only the
   five workspace packages are discovered.
7. Exercise manual ROS startup with the Agent available, unavailable, stopped,
   restarted, and serially disconnected. Confirm old generation/session tokens,
   invalid commands, lost sessions, stale feedback, and expired leases keep
   motion disarmed.
8. Smoke all component builds/tests through the VS Code Dev Container on one
   non-native development host. Record it as development evidence only.
9. Execute every command in both ordered tutorial tracks. Only the onboard
   track may feed a production handoff.

Do not run long stress, soak, powered-motion, or full architecture-matrix work
without the corresponding reviewed fixture and evidence plan.

## Physical and release boundary

The last physically exercised historical image predates this monorepo/native
build migration and is not evidence for the current artifact. The following
remain unqualified until new machine-generated HIL or instrument evidence is
recorded:

- powered motor direction, ticks/revolution, PID/filter/deadband, current,
  temperature, and operating range;
- positive-rotation IMU orientation and extended timing;
- battery, PWM, RGB, buzzer, LED, OLED, bus-servo, and button electrical
  behavior;
- watchdog, UART, I2C, reset/fault, Agent restart, and USB recovery timing;
- stack/resource and escaped-wire-traffic margins; and
- the 500 Hz/60-minute run, three 100-cycle recovery campaigns, and 24-hour
  soak.

Software tests, Dev Container results, mocks, and passive evidence never close
these gates. Preserve unrelated changes and never commit build outputs,
downloaded dependencies, logs, `.pio/` remnants, or HIL evidence. The ignored
`docs/reference/` legacy snapshot remains non-reproducible and active builds
must not depend on it.
