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
it is not motor authority. It cannot unlock the normal MCU image, change the
commissioning limits, or qualify motor PID/polarity data. The normal image
accepts zero/stop commands but rejects selected nonzero targets as
`UNSUPPORTED`. Nonzero bench motion additionally requires the exact guarded
commissioning build and physical precautions in
[`docs/flashing-and-first-bringup.md`](../../docs/flashing-and-first-bringup.md).
The commissioning utility accepts authorization only when exactly one publisher
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

With the Agent and MCU already running, perform the safe read-only check:

```sh
ros2 run mentor_pi_bringup qualification_monitor --ros-args \
  -p duration_sec:=60.0
```

That default is the strict preflight for a candidate whose IMU transform has
already been characterized and reviewed. The first-board locked image
intentionally leaves that transform unverified, publishes one initial invalid
zero-stamp IMU sample, and reports a `DEGRADED` heartbeat with `IMU_HEALTHY`
clear. It cannot and must not pass the strict preflight. For that initial
passive session, select the bounded characterization mode explicitly:

```sh
ros2 run mentor_pi_bringup qualification_monitor --ros-args \
  -p duration_sec:=60.0 \
  -p imu_characterization_mode:=true
```

A successful run prints `CHARACTERIZATION PASS`, never `PREFLIGHT PASS`. This
mode still enforces discovery, session continuity, time synchronization,
transport/error stability, traffic, and the normal rates and validity of every
other stream. It positively requires the intentional unverified state: a final
`DEGRADED` heartbeat with `TIME_SYNCHRONIZED` set and `IMU_HEALTHY` clear, and
exactly one diagnostic IMU `UNSUPPORTED` error with detail 1, no IMU timeout,
and no other fault. Because the best-effort volatile IMU topic publishes that
initial state only once, a late-joining monitor may receive zero IMU samples.
Zero or one sample is accepted; if one arrives it must be invalid, finite, and
zero-stamped. A duplicate sample, `READY`/`FAULT` heartbeat, healthy or valid
IMU, nonzero IMU stamp, or additional peripheral fault fails the run.

Firmware emits the expected `UNSUPPORTED` marker only after it has acquired a
raw QMI8658 sample successfully. A real raw-I2C error is recorded instead and
fails characterization; the marker still does not establish the board-axis
transform.

`CHARACTERIZATION PASS` is only first-board transport/peripheral evidence. It
does not validate the IMU axes and is not D3 HIL or release evidence. After the
six-face and positive-rotation transform is reviewed into a candidate, run the
default strict preflight instead.

The parameters `discovery_timeout_sec` and `rate_tolerance_percent` may also be
set at startup. All parameters are immutable once the node starts. Interrupting
the run or receiving a failing summary produces a nonzero process exit status.

An explicit opt-in can exercise only the MCU receive path at up to 500 Hz:

```sh
ros2 run mentor_pi_bringup qualification_monitor --ros-args \
  -p duration_sec:=60.0 \
  -p publish_zero_motor_commands:=true \
  -p zero_command_rate_hz:=500.0
```

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
`reconnect_agent`, and `reset_mcu`. The first two use the locked 60-minute and
24-hour profiles. The recovery modes finish after 100 observed session cycles
or fail at `campaign_timeout_sec`.

This executable is deliberately incapable of nonzero motor motion. Motor
publications repeat the observable update-mask sequence `1, 2, 4, 8, 15` while
all four targets remain literal zero; at the 500 Hz profile the full-stop mask
therefore appears every 10 ms. The one-channel masks exercise subset merging,
but never select a nonzero value. There is no target,
commissioning-authority, or firmware-unlock parameter. It can still move PWM
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

The evidence parent must already be an absolute, real directory, and the final
`evidence_directory` must not exist. The tool refuses to overwrite a prior
record. Substitute reviewed identity and fixture values in this canonical
`load500` example; the intentionally invalid `REPLACE_*` values make an
unreviewed copy fail closed:

```sh
readonly RUN_ID="load500-REPLACE_WITH_UTC_START"
readonly EVIDENCE_PARENT="${PWD}/evidence"
readonly EVIDENCE_DIRECTORY="${EVIDENCE_PARENT}/${RUN_ID}"
readonly SOURCE_REVISION="source-REPLACE_WITH_REVISION"
readonly HOST_REVISION="host-REPLACE_WITH_REVISION"
readonly FIRMWARE_SHA256="REPLACE_WITH_EXACT_64_HEX_DIGEST"
readonly BOARD_SERIAL="REPLACE_WITH_BOARD_SERIAL"
readonly FIXTURE_REVISION="REPLACE_WITH_FIXTURE_REVISION"

mkdir -p -- "${EVIDENCE_PARENT}"
test -d "${EVIDENCE_PARENT}" && test ! -L "${EVIDENCE_PARENT}"
test ! -e "${EVIDENCE_DIRECTORY}"

ros2 run mentor_pi_bringup qualification_campaign --ros-args \
  -p mode:=load500 \
  -p duration_sec:=-1.0 \
  -p evidence_directory:="${EVIDENCE_DIRECTORY}" \
  -p run_id:="${RUN_ID}" \
  -p source_revision:="${SOURCE_REVISION}" \
  -p firmware_sha256:="${FIRMWARE_SHA256}" \
  -p host_revision:="${HOST_REVISION}" \
  -p board_serial:="${BOARD_SERIAL}" \
  -p fixture_revision:="${FIXTURE_REVISION}" \
  -p fixture_acknowledgement:=PERIPHERALS_DISCONNECTED_OR_GUARDED \
  -p bus_servo_id:=REPLACE_WITH_MEASURED_ID \
  -p bus_hold_position:=REPLACE_WITH_SAFE_POSITION \
  -p bus_position_tolerance:=REPLACE_WITH_REVIEWED_TOLERANCE \
  -p bus_current_offset:=REPLACE_WITH_MEASURED_OFFSET \
  -p bus_torque_enabled:=REPLACE_WITH_CURRENT_BOOLEAN \
  -p require_button_stimulus:=true \
  -p require_valid_imu:=true
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
[`verification.md`](../../docs/framework/verification.md), including the USB
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
never closes D5 by itself. Record the artifacts and independent measurements in
the [qualification evidence ledger](../../docs/framework/qualification-evidence-ledger.md).

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

For the initial locked-image session, use its explicit read-only
characterization mode in place of a separate monitor invocation. Substitute
the reviewed handoff path, run from the repository root, and choose a new
absolute evidence directory:

```sh
readonly HANDOFF_DIRECTORY="${PWD}/build/board-handoff/REPLACE_WITH_REVIEWED_DIRECTORY"
readonly EVIDENCE_DIRECTORY="${PWD}/evidence/first-board-$(date -u +%Y%m%dT%H%M%SZ)"
sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/capture_board_diagnostics \
  --output "${EVIDENCE_DIRECTORY}" \
  --handoff-directory "${HANDOFF_DIRECTORY}" \
  --repository-root "${PWD}" \
  --qualification imu-characterization \
  --qualification-duration-sec 60
```

The utility returns nonzero when its ROS prerequisites or requested monitor
fail, but it still prints the preserved archive path. `SUMMARY.txt` identifies
that outcome and `command-status.tsv` points to each failing command's output.
When run through `sudo`, new bundle files are returned to the invoking user's
UID/GID. For a fault that should be captured immediately, first remove motor
power and make the fixture safe, then omit `--qualification`; this avoids a
60-second run and captures the current failure state without creating a ROS
command publisher:

```sh
readonly FAILURE_EVIDENCE="${PWD}/evidence/fault-$(date -u +%Y%m%dT%H%M%SZ)"
sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/capture_board_diagnostics \
  --output "${FAILURE_EVIDENCE}"
```

Send the `.tar.gz`, its `.sha256`, the completed board checklist, firmware
digest, and relevant scope/logic traces with a bug report. The archive contains
the reviewed controller configuration, recent journals, and USB identity, so
inspect it for site-sensitive data before sharing. Do not reboot before this
capture when it is safe to retain the live reset/session evidence.

## Build and native tests

The production build requires ROS 2 Jazzy and the sibling
`mentor_pi_interfaces` package:

```sh
colcon build --packages-up-to mentor_pi_bringup
colcon test --packages-select mentor_pi_bringup
```

The state machine, configuration validator, and deterministic qualification
monitor core can be tested without ROS:

```sh
cmake -S src/mentor_pi_bringup -B build/mentor_pi_bringup-native \
  -DMENTOR_PI_BUILD_ROS2=OFF -DBUILD_TESTING=ON
cmake --build build/mentor_pi_bringup-native
ctest --test-dir build/mentor_pi_bringup-native --output-on-failure
```

The ROS build also runs three C++ integration tests with generated Jazzy types.
The in-process controller peer verifies exact non-default service payloads,
motor-model -> PWM-offset -> battery-threshold ordering, immutable deployment
parameters, session-ID recovery, motion-gate closure, and bounded retry after
an injected `BUSY` response. A real-rcl/rmw fault test withholds service
responses across the 100 ms client timeout and an Agent-session change. It
proves retries can complete through a reentrant peer while each late reply is
discarded without changing the generation-bound motion authorization. A
separate process test starts the installed XML launch description, verifies the
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

The Jazzy Agent is not assumed to exist in `/opt/ros`. Install the pinned
native build once on Ubuntu 24.04:

```sh
sudo ./tools/install_microros_agent.sh
```

The installer verifies Ubuntu 24.04 from `/etc/os-release`, accepts only
`amd64`/`arm64`, and checks out immutable official Agent and message revisions.
Before its first package or source-tree mutation, it proves
`mentor-pi-controller.target` is inactive; a not-yet-installed target is
accepted only when systemd reports `LoadState=not-found` and
`ActiveState=inactive`. A loaded inactive target is also safe; an active,
transitional, failed, malformed, or indeterminate state fails closed.
Before building, it requires both source trees to have the exact origin, a
detached pinned commit, and no modified or untracked files. It builds a release
prefix under `/opt/mentor_pi` and installs a native-exec wrapper. Python is used
only by upstream ROS build tooling; it is not present in the running transport
path.

The XML launch description starts that Agent executable. Stop the production
target first: the installed Agent wrapper takes a nonblocking serial-owner lock
and refuses a second Agent. A production installation also grants the device
only to the `mentor-pi-serial` group, so run an interactive launch under the
`mentor-pi` service account after starting its runtime-directory unit:

```sh
sudo systemctl stop mentor-pi-controller.target
sudo systemctl start mentor-pi-runtime.service
sudo -u mentor-pi -- env \
  ROS_DOMAIN_ID=37 \
  ROS_LOG_DIR=/var/log/mentor-pi \
  XDG_RUNTIME_DIR=/run/mentor-pi \
  bash -c 'source /opt/mentor_pi/host/setup.bash && \
    ros2 launch mentor_pi_bringup controller.launch.xml \
      serial_device:=/dev/mentor_pi_mcu'
```

`serial_device` may be overridden, but only the Agent receives it. The
supervisor has no serial-device parameter. Developers using another verified
native Agent prefix may also override `agent_executable`.

## Production installation

Production uses immutable versioned host releases below
`/opt/mentor_pi/releases/host`; `/opt/mentor_pi/host` is an atomically replaced
symlink. Never build into that live path. Build and test a new staging prefix as
an unprivileged operator:

For a clean-machine, fail-closed dependency bootstrap and a checksummed host
handoff that also proves the copied prefix works with its build path absent,
follow [`docs/host-preparation-and-handoff.md`](../../docs/host-preparation-and-handoff.md).
The manual commands below remain the underlying native deployment sequence.

```sh
source /opt/ros/jazzy/setup.bash
readonly RELEASE_ID="2026-08-06.1"
readonly STAGED_PREFIX="${PWD}/artifacts/mentor-pi-host-${RELEASE_ID}"
test ! -e "${STAGED_PREFIX}"
colcon build --merge-install \
  --install-base "${STAGED_PREFIX}" \
  --packages-up-to mentor_pi_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --merge-install \
  --install-base "${STAGED_PREFIX}" \
  --packages-select mentor_pi_interfaces mentor_pi_bringup
colcon test-result --verbose
```

Install the pinned Agent, then promote the already-tested host prefix. The
promotion tool refuses an active controller target, an incomplete prefix, an
existing release ID, or a non-managed `/opt/mentor_pi/host` path. Its layout
check includes both package-index records, interface metadata, and the
generated C/C++/Fast-RTPS runtime libraries required by the supervisor. New
releases must also contain the read-only diagnostic collector and the shared
deployment idle guard. It copies a complete root-owned, non-group-writable
release before switching the symlink; older releases remain available for
rollback. The Agent installer, release promoter, and production-asset installer
all require an exact loaded-or-not-found/inactive target state before mutation:

```sh
sudo ./tools/install_microros_agent.sh
sudo "${STAGED_PREFIX}/lib/mentor_pi_bringup/promote_host_release" \
  --staged-prefix "${STAGED_PREFIX}" \
  --release-id "${RELEASE_ID}"
```

Before first installation, connect exactly one target CH9102F and record either
its nonempty `ID_SERIAL_SHORT` or its physical `ID_PATH` together with the
current tty node:

```sh
readonly DEVICE=/dev/ttyUSB0
udevadm info --query=property --name="${DEVICE}" | \
  grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)='
```

Use the guarded installer, substituting the reviewed ROS domain and recorded
identity. `--identity-kind id-path` with the exact `ID_PATH` is supported when
the adapter has no unique serial. The installer verifies that exactly one
connected CH9102F matches before writing anything. It creates the dedicated
`mentor-pi-serial` group and service account, removes the service account from
the broad `dialout` group, renders a unique udev rule, and installs coordinated
units. First-install mode refuses every pre-existing site file:

```sh
sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/install_production_assets \
  --mode first-install \
  --ros-domain-id 37 \
  --identity-kind serial \
  --identity-value RRCLITE_A1B2C3 \
  --device "${DEVICE}"

sudo systemd-analyze verify \
  /etc/systemd/system/mentor-pi-runtime.service \
  /etc/systemd/system/mentor-pi-agent.service \
  /etc/systemd/system/mentor-pi-configuration-supervisor.service \
  /etc/systemd/system/mentor-pi-controller.target
sudo systemctl enable --now mentor-pi-controller.target
```

The required `/etc/default/mentor-pi` is authoritative and contains exactly one
active setting, `ROS_DOMAIN_ID`. Both services load it last and their launchers
reject a missing, malformed, or out-of-range value. Service-specific defaults
must contain no `ROS_*` keys. The fixed production serial path is
`/dev/mentor_pi_mcu`; production does not accept a path override.

Upgrade mode deliberately preserves `/etc/mentor-pi/controller.yaml`, the
shared ROS identity, supervisor path overrides, and the rendered device rule.
It fails on a ROS-domain or device-identity mismatch instead of replacing them.
Build another staged release, stop the target, promote it, install only the new
units while validating the preserved site state, and then restart:

```sh
readonly PREVIOUS_RELEASE_ID="2026-08-06.1"
readonly NEW_RELEASE_ID="2026-09-01.1"
readonly NEW_STAGED_PREFIX="${PWD}/artifacts/mentor-pi-host-${NEW_RELEASE_ID}"
sudo systemctl stop mentor-pi-controller.target
sudo "${NEW_STAGED_PREFIX}/lib/mentor_pi_bringup/promote_host_release" \
  --staged-prefix "${NEW_STAGED_PREFIX}" \
  --release-id "${NEW_RELEASE_ID}"
sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/install_production_assets \
  --mode upgrade \
  --ros-domain-id 37 \
  --identity-kind serial \
  --identity-value RRCLITE_A1B2C3 \
  --device "${DEVICE}"
sudo systemctl start mentor-pi-controller.target
```

If validation or live checks fail, keep the target stopped, reactivate the
previous complete release, reinstall its units in upgrade mode, and start the
target only after verification:

```sh
sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/promote_host_release \
  --activate-release "${PREVIOUS_RELEASE_ID}"
sudo /opt/mentor_pi/host/lib/mentor_pi_bringup/install_production_assets \
  --mode upgrade \
  --ros-domain-id 37 \
  --identity-kind serial \
  --identity-value RRCLITE_A1B2C3 \
  --device "${DEVICE}"
```

Changing motor model, PWM offsets, battery threshold, ROS domain, or USB
identity is a separate reviewed site-configuration operation; neither install
mode performs such a change implicitly. Archive and remove any obsolete
`/etc/default/mentor-pi-agent` after confirming its values were migrated—the
production unit no longer loads that file.

Enable only `mentor-pi-controller.target` for normal production startup. It
coordinates three units:

- `mentor-pi-runtime.service` owns `/run/mentor-pi` for the complete stack
  lifetime and `/var/log/mentor-pi` for ROS log files;
- `mentor-pi-agent.service` is the sole serial owner, is limited to the uniquely
  selected device, holds the wrapper lock, and retains the required one-second
  `Restart=always` policy; and
- `mentor-pi-configuration-supervisor.service` starts after the Agent and
  immediately replaces its small environment launcher with the installed C++
  executable. It uses `Restart=always` and intentionally remains alive
  during Agent reconnects so its graph/session logic can close the motion gate
  and reapply configuration.

The Agent and supervisor require the same domain-only `/etc/default/mentor-pi`,
use the same `ROS_DOMAIN_ID`, set `XDG_RUNTIME_DIR=/run/mentor-pi`, and set
`ROS_LOG_DIR=/var/log/mentor-pi`. Supervisor-path overrides are separate and
cannot override the authoritative domain. The supervisor command has no device
argument, and its closed device policy has no `DeviceAllow` exception for the
USB serial adapter. The Agent has a closed policy with only that device allowed.
The units do not use a private `/dev` mount because ROS
middleware may use the host's shared-memory namespace; the device cgroup
restriction blocks physical device nodes without separating that middleware
transport. The launcher sources the merged ROS environment and uses `exec`;
neither `ros2 run` nor a Python node remains in the running process tree.

After changing the shared environment or immutable controller YAML, restart
the coordinated target and inspect both services:

```sh
sudo systemctl restart mentor-pi-controller.target
systemctl status mentor-pi-agent.service \
  mentor-pi-configuration-supervisor.service
journalctl -u mentor-pi-agent.service \
  -u mentor-pi-configuration-supervisor.service
```

The journal is the primary operator log; ROS also writes beneath
`/var/log/mentor-pi`. Never put a serial-device option in the supervisor unit
or launcher, and never start a second Agent or interactive launch while the
production target owns the port.

Production support is Ubuntu 24.04 amd64 or arm64. Apple Silicon development
uses an Ubuntu 24.04 ARM64 VM with exclusive USB passthrough.

## Verification status

The pure C++ validator/state-machine tests and the systemd static/mock test pass
on non-ROS hosts. The complete package, generated `mentor_pi_interfaces`, and
the ROS integration/lint gates—including in-process and external-launch
supervisor tests—pass in the official ROS 2 Jazzy ARM64 container. Remaining
deployment evidence includes `systemd-analyze verify` and live boot on the
supported Ubuntu image, XRCE/middleware fault injection with the installed
native micro-ROS Agent, and hardware tests for graph-disappearance timing,
reconnect behavior, and ordered reapplication against the MCU.
