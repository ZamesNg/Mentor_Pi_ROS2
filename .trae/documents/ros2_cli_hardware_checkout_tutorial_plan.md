# ROS 2 CLI Hardware Checkout Tutorial — Implementation Record

Status: implemented

The original plan is now delivered as one complete eight-tutorial Docker-first sequence
documented in the repository [README](../../README.md). The implementation uses the active
default PID firmware contract and corrects obsolete assumptions from the draft
about locked and commissioning firmware modes, firmware-owned indicators, PID
defaults, and topic rejection codes.

## Delivered documentation

- [Tutorial 06: ROS 2 CLI Hardware Checkout](../../docs/tutorials/06-ros2-cli-hardware-checkout.md)
  provides direct Humble commands for endpoint inventory, LEDs, RGB1, buzzer,
  optional OLED, bounded single-motor checkout, PID updates, PWM servos, bus
  servos, telemetry, battery configuration, and explicit teardown.
- [Tutorial 07: Stress, Soak, and Release Gates](../../docs/tutorials/07-run-stress-soak-and-release-gates.md)
  retains the zero-command campaign and HIL evidence boundary.
- [Tutorial 08: `mentor_pi_hardwares`](../../docs/tutorials/08-run-mentor-pi-hardwares.md)
  covers the coordinated ros2_control launches.
- Tutorials 01--05, repository maps, package READMEs, and framework cross-links
  are stitched to the new sequence.

## Delivered launch behavior

All project launch files use Python. The controller launch is
`mentor_pi_bringup controller.launch.py`; it starts the compiled micro-ROS Agent
and configuration supervisor together, performs the guarded runtime preflight
inside the hardened runtime container, and shuts down the launch if either
critical process exits. The root `make start` target is Docker-only.

## Contract and regression coverage

- Handoff tutorial lists and source fingerprints contain all eight tutorials.
- Documentation validation requires the Python launch command and the CLI
  endpoint families.
- Tutorial action tests reject XML launch files, compile every Python launch,
  and check the acknowledgement, artifact validation, and coupled lifecycle
  hooks in the controller launch.
- The active framework consistently describes the single
  `NORMAL_CLOSED_LOOP_DEFAULT` / `CLOSED_LOOP` PID artifact. Powered-motion and
  PID-performance claims remain blocked without recorded HIL evidence.
