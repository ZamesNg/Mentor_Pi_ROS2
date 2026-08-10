# mentor_pi_bringup

This package contains the C++17 host configuration supervisor and declarative
deployment assets for RRCLite v2. It never opens the serial device and does not
translate ROS messages. The native micro-ROS Agent is the only serial owner.

## Runtime behavior

`/mentor_pi/configuration_supervisor` validates exactly three startup
parameters, observes `/mentor_pi/heartbeat`, and applies configuration in this
order after every new MCU/Agent session:

1. `/mentor_pi/motors/set_model`;
2. `/mentor_pi/pwm_servos/set_offsets`;
3. `/mentor_pi/battery/set_low_threshold`.

The transient-local topic `/mentor_pi/configuration/motion_enabled` remains
false until all three calls return `OK`. Project-owned motion publishers should
subscribe to that gate and publish no motion command while it is false. The
supervisor also publishes the transient-local `std_msgs/msg/UInt64` topic
`/mentor_pi/configuration/motion_authorization`. Zero means disabled. A nonzero
value packs the low 32 bits of the host configuration generation in the upper
word and the current `agent_session_id` in the lower word. Each call times out
after 100 ms and `BUSY`, returned `TIMEOUT`, and client timeout alone receive up
to four attempts with 100/200/400 ms backoff.

A true value means only that deployment configuration was applied successfully;
it does not establish physical safety or qualify motor PID/polarity data. The
normal PID image accepts bounded, validated nonzero commands after its
configuration/session gates are satisfied. Nonzero bench motion therefore
requires passive completion of Tutorials 01--05, raised or equivalently
guarded wheels, a current-limited supply, and the guarded precautions in
Tutorial 06. The commissioning utility accepts authorization only when exactly one publisher
owns the authorization topic and that endpoint is the node
`/mentor_pi/configuration_supervisor`. It locks the generation/session pair
before motion and aborts if the publisher disappears, the token changes, MCU
uptime or sequence regresses, or any other motor-command publisher appears.

Configuration is immutable after startup. Missing or unknown keys, wrong YAML
types, the wrong PWM-offset count, and out-of-range values keep the gate false
and produce a precise error. An `OK` motor-model response is accepted only when
`active_model`, `ticks_per_revolution`, and `max_rps` exactly match the
project-owned profile contract for the requested model; an inconsistent
response is converted to `IO_ERROR` and keeps both authorization outputs
disabled. Use `config/controller.yaml` as the deployment schema source.

## Read-only hardware preflight

`/mentor_pi/qualification_monitor` is a finite-duration qualification tool,
not a controller. By default it creates no command publisher and only observes
the seven documented telemetry publishers. It verifies graph discovery,
session continuity, periodic rates, per-topic non-regressing timestamps,
monotonic diagnostics, transport-error/reset stability, ACTIVE/READY state,
valid telemetry, and the 70 kB/s transport ceiling for both the whole-run
average and every interval between received diagnostics samples. The button
publisher must be discoverable, but no button event is required because that
topic is event-driven. The default 60-second run applies the contract's 5% rate
tolerance and exits zero only on `PREFLIGHT PASS`. Release qualification still
uses the required physical wire capture to evaluate every complete one-second
window; diagnostics-interval preflight is not a substitute for that evidence.

With `make start` and the MCU already running, perform both the safe read-only
check and the zero-only 500 Hz receive-path check from the repository root:

```sh
make qualification-preflight
```

That default is the strict preflight for a candidate whose IMU transform has
already been characterized and reviewed. For the initial passive session with
the reference-derived provisional identity transform, use the guided board
workflow:

```sh
make passive-check
```

A successful run prints `CHARACTERIZATION PASS`, never `PREFLIGHT PASS`. It
enforces discovery, session continuity, time synchronization, a final `READY`
heartbeat with `IMU_HEALTHY` set, valid 50 Hz IMU telemetry, transport/error
stability, traffic, and the normal rates and validity of every other stream. A
USB-only fixture may report only the exact absent-battery state of 0 mV,
invalid, and not below threshold; strict preflight still requires a valid
calibrated battery input. If the OLED is declared absent, only its exact
initialization NACK is excused. Any IMU error, timeout, invalid sample, missing
sample, degraded heartbeat, or additional peripheral fault fails the run.

`CHARACTERIZATION PASS` proves live sensor transport but does not validate the
IMU axes and is not D3 HIL or release evidence. After the six-face transform is
reviewed into a candidate, run the default strict preflight instead.

The parameters `discovery_timeout_sec` and `rate_tolerance_percent` may also be
set at startup. All parameters are immutable once the node starts. Interrupting
the run or receiving a failing summary produces a nonzero process exit status.

The second phase of `make qualification-preflight` explicitly opts into the
MCU receive path at 500 Hz.

That mode always sends one four-channel `MotorCommand` with `update_mask=15`
and four literal `0.0F` targets. There is no target-value parameter, service
call, motion-gate publication, firmware-unlock operation, or nonzero motion
path in this executable. Keep the robot supported with wheels clear even for a
zero-only receive-path test, and stop if observed motor targets are nonzero.

The timer sends nothing until the first diagnostics sample proves a fresh
zero baseline for `motor_command_consumptions`,
`motor_command_age_over_20_ms`, and `motor_command_max_age_us`. A contaminated
baseline never opens the timer gate. The run also requires this monitor to be
the only `/mentor_pi/motors/command` publisher. Between matching first/last
diagnostics boundaries it checks that consumptions plus motor-mailbox
overwrites account for at least 95% of host publications, with one final
pending generation allowed, and do not exceed host publications by more than
one pre-baseline pending generation. The command-age p99 passes only when the
strict-over-20-ms count is no greater than one percent of consumption samples;
every sample must remain below 100,000 us. The final summary prints every
counter used for those decisions. This is bounded software/transport preflight
evidence; it does not replace the physical 60-minute HIL run, wire capture,
PID/polarity qualification, or 24-hour soak.

## Guarded qualification campaigns

`/mentor_pi/qualification_campaign` generates the documented mixed command and
service load while observing telemetry, diagnostics, sessions, and ROS endpoint
recovery. Its modes are `load500`, `soak`, `reconnect_usb`,
`reconnect_agent`, and `reset_mcu`. The first two use fixed 60-minute and
24-hour zero-motor-command profiles. The recovery modes finish after 100 observed session cycles
or fail at `campaign_timeout_sec`.

This executable is deliberately incapable of nonzero motor motion. Motor
publications repeat the observable update-mask sequence `1, 2, 4, 8, 15` while
all four targets remain literal zero; at the 500 Hz profile the full-stop mask
therefore appears every 10 ms. The one-channel masks exercise subset merging,
but never select a nonzero value. There is no target,
motor-authority or firmware-mode parameter. It can still move PWM
and bus servos and operate LEDs, the buzzer, RGB pixels, and OLED. Put the robot
on the reviewed guarded fixture, isolate hazardous mechanisms and motor power,
and stop on any unexpected output. The required
`fixture_acknowledgement` value is exactly:

```text
PERIPHERALS_DISCONNECTED_OR_GUARDED
```

That value is an operator assertion, not a safety bypass. Do not supply it
until the physical statement is true. Before a run, read the connected bus
servo and explicitly supply its actual `bus_servo_id` (1--253), safe
`bus_hold_position` (0--1000), allowed `bus_position_tolerance` (0--100),
current volatile `bus_current_offset` (-125--125), and current
`bus_torque_enabled` state. The configure-service exercise reapplies the
declared current volatile values; it does not save an offset or change ID or
persistent limits. A wrong declaration can still move or unload a servo, so do
not copy fixture values from another robot.

The one-line campaign commands generate an immutable output directory,
auto-detect revisions, firmware digest and board serial, then interactively
validate every fixture-specific value:

```sh
make campaign-load
make campaign-soak
make campaign-recovery
```

`duration_sec=-1.0` selects the canonical duration. A positive override is
available only for `load500` and `soak` setup checks; it produces a visibly
noncanonical record and is not endurance acceptance. `discovery_timeout_sec`
may bound initial graph discovery. For the 24-hour profile, use `mode:=soak`, a
new run ID/output directory, and keep `duration_sec:=-1.0`.

For `reconnect_usb`, `reconnect_agent`, or `reset_mcu`, use a new evidence
directory, omit `duration_sec`, and set a reviewed `campaign_timeout_sec`. The
executable only observes the resulting session and endpoint changes. It never
unplugs USB, kills or starts the Agent, asserts NRST, or switches MCU power. The
external operator or fixture must perform the exact 100-cycle procedure in
[`verification.md`](../../../docs/framework/verification.md), including the USB
outage rotation and connected intervals where applicable.
For `reset_mcu`, only the published `RESET_POWER_ON`, `RESET_PIN`,
`RESET_SOFTWARE`, or `RESET_BROWNOUT` causes count. Watchdog, low-power,
unknown, and unrecognized causes fail the campaign and are never counted as an
operator reset; every accepted cause is bound to its exact session-transition
record.
`require_button_stimulus` and `require_valid_imu` control whether those
observable functions are required; they do not create physical stimulus.

At completion the new directory contains `summary.json`, `metrics.csv`,
`session-transitions.csv`, and `junit.xml`, all bound to the supplied revision,
firmware digest, board, fixture, mode, and observed session IDs. Internal
diagnostic traffic, command-age counters, and ROS graph recovery are enforced
where observable. Independent wire traffic, physical motor/servo behavior,
actual USB outage/reset timing, target stack headroom, allocation traces, and
other HIL measurements remain `NOT_OBSERVED` or skipped until their real
instruments are attached. Consequently `CampaignSummary::release_qualified`
is always false and `summary.json` always says
`release_qualification: INCOMPLETE`; a zero exit status or green JUnit record
never closes D5 by itself. Follow
[Tutorial 07](../../../docs/tutorials/07-run-stress-soak-and-release-gates.md) and
retain generated campaign and independent instrument files; an absent or
`NOT_OBSERVED` physical metric keeps the release gate open.

## Read-only diagnostic and bug-feedback bundle

The installed `capture_board_diagnostics` utility creates one finite support
bundle without opening the serial transport, publishing a ROS command, calling
a service, changing systemd state, or flashing firmware. It records the
installed host/Agent paths, systemd state and recent service journals, rendered
USB identity, current serial owner, ROS graph and QoS, bounded telemetry
snapshots, command-publisher ownership, and optional handoff/source checks. It
writes a per-command status table, an internal SHA-256 manifest, a compressed
archive, and the archive digest even when a prerequisite or requested
qualification fails.

For the passive default-PID session, create the complete diagnostic bundle through
the guided passive action. It binds the current authoritative artifact directly
and does not require a handoff path:

```sh
make passive-check
```

The utility returns nonzero when its ROS prerequisites or requested monitor
fail, but it still prints the preserved archive path. `SUMMARY.txt` identifies
that outcome and `command-status.tsv` points to each failing command's output.
The underlying installed tool retains a read-only no-qualification mode for
support automation, but operators should use the tutorial action so output
paths, ownership and current source identity are selected consistently.

Send the `.tar.gz`, its `.sha256`, firmware digest, and relevant scope/logic
traces with a bug report. The archive contains
the reviewed controller configuration, recent journals, and USB identity, so
inspect it for site-sensitive data before sharing. Do not reboot before this
capture when it is safe to retain the live reset/session evidence.

## Docker build and tests

Build and test this package with its sibling interfaces and hardware package
through the repository-root Docker interface:

```sh
make host
```

The architecture-native Ubuntu 22.04/Humble image supplies `rosdep`, `colcon`,
and the native CMake test dependencies. Host ROS and host build toolchains are
not used.

The ROS build also runs three C++ integration tests with generated Humble types.
The in-process controller peer verifies exact non-default service payloads,
motor-model -> PWM-offset -> battery-threshold ordering, immutable deployment
parameters, session-ID recovery, motion-gate closure, and bounded retry after
an injected `BUSY` response. A real-rcl/rmw fault test withholds service
responses across the 100 ms client timeout and an Agent-session change. It
proves retries can complete through a reentrant peer while each late reply is
discarded without changing the generation-bound motion authorization. A
separate process test starts the installed Python launch description, verifies the
exact Agent command line with a compiled fake Agent, and drives the externally
launched supervisor through a C++ controller peer. These tests do not emulate
XRCE traffic or replace the physical reconnect campaign. A separate
deployment-asset test checks the coordinated systemd dependencies, shared ROS
environment and writable directories, serial-device isolation, restart
policies, installed paths, and the supervisor launcher's exact argument
forwarding with a mock executable.

The qualification-campaign ROS-node fault tests use a test-only C++ factory
that preserves each mode's session, duration, cycle, and service semantics but
caps the motor-command schedule at 10 Hz and gives fake telemetry a one-second
gap floor. This prevents emulated ARM wall-clock jitter from masking the
session, validation, service, and abort fault under test. The production
factory exposes neither override as a ROS parameter, canonical evidence rejects
an overridden profile, and the test is not 500 Hz performance evidence. The
canonical 500 Hz schedule remains covered by deterministic core timestamps and
must pass the physical stress campaign.

## Interactive launch

The host build, compiled Agent, and ROS 2 Humble environment are supplied by
architecture-native pinned Docker images. Build and start them through the
repository interface:

```sh
make host
make agent
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED make start
```

Open a second interactive terminal in the running container with:

```sh
make shell ROS_DOMAIN_ID=0
```

The shell is zsh with pinned Oh My Zsh, standard and ROS 2 completion,
autosuggestions, and syntax highlighting. Host dotfiles and host ROS are not
used. The controller launch remains Bash-driven and fail-couples the Agent and
configuration supervisor.

## Production installation

Production uses immutable versioned host and Agent releases below
`/opt/mentor_pi`, plus the exact runtime image ID recorded in
`/etc/mentor-pi/runtime-image`. On amd64, `make rdk-handoff` is the production
route for the RDK: verify the complete arm64 bundle, `docker load` its OCI
archive, confirm the recorded image ID and `linux/arm64` platform, install its
versioned Agent, and promote its host prefix without rebuilding the workspace.
The amd64 builder uses up to eight package workers and automatically resumes an
interrupted handoff. Changed inputs invalidate only affected stages while
unaffected checksummed outputs and incremental colcon state remain;
`RDK_HANDOFF_FRESH=1 make rdk-handoff` is the explicit full reset.
Identify one CH9102F by stable serial or `ID_PATH`; install/verify systemd while
the controller is stopped and enable it only after flashing the packaged PID
firmware. Native RDK compilation is optional diagnostics, not release evidence.
The bundle contains both release prefixes, the PID firmware release, and a
checksummed OCI runtime-image archive. The installed host path is an atomically
replaced symlink; never build into it.

For connected development, follow
[Tutorial 03](../../../docs/tutorials/03-build-and-run-humble-host.md). For a
production handoff, use the dedicated target and the transfer/install commands
in Tutorials 01--03.

```sh
make rdk-handoff
```

On the RDK, the root Make targets load the pinned runtime image, install the
packaged Agent prefix, and promote the already-tested host prefix. The
underlying promotion tool refuses an incomplete prefix, an
existing release ID, or a non-managed `/opt/mentor_pi/host` path. Its layout
check includes both package-index records, interface metadata, and the
generated C/C++/Fast-RTPS runtime libraries required by the supervisor. New
releases must also contain the read-only diagnostic collector and the shared
deployment idle guard. It copies a complete root-owned, non-group-writable
release before switching the symlink; older releases remain available for
rollback. The release promoter and production-asset installer require an exact
loaded-or-not-found/inactive target state before mutation. Use the compact
receive, identity, install, and lifecycle commands:

```sh
make rdk-receive
make production-install PORT=/dev/ttyUSB0 ROS_DOMAIN_ID=37 \
  IDENTITY_KIND=serial IDENTITY_VALUE=RRCLITE_A1B2C3
sudo systemctl enable --now mentor-pi-controller.target
```

Use `IDENTITY_KIND=id-path` with the exact `ID_PATH` only when the adapter has
no unique serial. The compact installer verifies the received and host
manifests, symlinks, image identity/platform, Agent executable, connected
CH9102F identity, promoted host layout, site files, and systemd units. It
creates the dedicated `mentor-pi-serial` group and service account, removes the
service account from the broad `dialout` group, and renders the unique udev
rule. First-install mode refuses every pre-existing site file.

The required `/etc/default/mentor-pi` is authoritative and contains exactly one
active setting, `ROS_DOMAIN_ID`. The runtime service loads it last and its launcher
rejects a missing, malformed, or out-of-range value. Service-specific defaults
must contain no `ROS_*` keys. The fixed production serial path is
`/dev/mentor_pi_mcu`; production does not accept a path override.

Upgrade mode deliberately preserves `/etc/mentor-pi/controller.yaml`, the
shared ROS identity, controller configuration, and the rendered device rule.
It fails on a ROS-domain or device-identity mismatch instead of replacing them.
Transfer another RDK handoff, select it explicitly only if it is not the newest,
install in upgrade mode while validating the preserved site state, and restart:

```sh
make production-install INSTALL_MODE=upgrade PORT=/dev/ttyUSB0 \
  ROS_DOMAIN_ID=37 IDENTITY_KIND=serial IDENTITY_VALUE=RRCLITE_A1B2C3
sudo systemctl start mentor-pi-controller.target
```

If validation or live checks fail, keep the target stopped, reactivate the
previous complete handoff in upgrade mode, and start the target only after
verification:

```sh
make rdk-receive \
  RDK_HANDOFF=/absolute/path/to/rdk-arm64-<previous-timestamp>
make production-install INSTALL_MODE=upgrade \
  RDK_HANDOFF=/absolute/path/to/rdk-arm64-<previous-timestamp> \
  PORT=/dev/ttyUSB0 ROS_DOMAIN_ID=37 \
  IDENTITY_KIND=serial IDENTITY_VALUE=RRCLITE_A1B2C3
sudo systemctl start mentor-pi-controller.target
```

Changing motor model, PWM offsets, battery threshold, ROS domain, or USB
identity is a separate reviewed site-configuration operation; neither install
mode performs such a change implicitly. Archive and remove any obsolete
`/etc/default/mentor-pi-agent` after confirming its values were migrated—the
production unit no longer loads that file.

Enable only `mentor-pi-controller.target` for normal production startup. It
owns one `mentor-pi-runtime.service`. The root systemd process talks to Docker,
but the container runs as the unprivileged `mentor-pi` account with the serial
device's supplemental group. The runtime mounts only the reviewed serial
device, `/run/udev`, installed host/Agent prefixes, immutable configuration,
and its log directory. It uses host networking, a read-only root filesystem,
all capabilities dropped, `no-new-privileges`, and temporary home/log state.
The Bash entrypoint starts `controller.launch.py`; a critical Agent or
supervisor exit ends the container, and systemd owns restart/stop behavior.

After changing the shared environment or immutable controller YAML, restart
the coordinated target and inspect the runtime service:

```sh
sudo systemctl restart mentor-pi-controller.target
systemctl status mentor-pi-runtime.service
journalctl -u mentor-pi-runtime.service
```

The journal is the primary operator log; ROS also writes beneath
`/var/log/mentor-pi`. Never start a second Agent or development runtime while
the production target owns the port.

Production support is the pinned Ubuntu 22.04/ROS 2 Humble runtime image on
arm64. Development uses the same architecture-native image model on supported
Ubuntu hosts. ROS 2 Jazzy is future migration work
to complete before Humble reaches end of life in May 2027; mixed-distribution
deployment is unsupported.

## Verification status

The pure C++ validator/state-machine tests and the systemd static/mock test pass
on non-ROS hosts. The complete package, generated `mentor_pi_interfaces`, and
the ROS integration/lint gates—including in-process and external-launch
supervisor tests—must pass through authoritative `make host` runs for both
amd64 and arm64 before release. Remaining deployment evidence includes
`systemd-analyze verify` and live boot on the supported Ubuntu image,
XRCE/middleware fault injection with the installed native micro-ROS Agent, and
hardware tests for graph-disappearance timing, reconnect behavior, and ordered
reapplication against the MCU.
