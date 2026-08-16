# Host reconnect incident report — 2026-08-16

## Outcome

The host-side loss of control after an Agent or MCU reconnect was caused by
the `mentor_pi_hardwares` plugins returning `ERROR` after authorized motor,
IMU, or PWM feedback exceeded the configured 100 ms timeout. This was a
recoverable communication condition, but ROS 2 Humble controller manager
2.54 treated it as a hardware lifecycle error.

The shared Ackermann and Mecanum recovery gate now keeps each hardware
component active and its controller interfaces claimed while immediately
publishing safe commands. It resumes only after current heartbeat,
authorization, and required feedback have all crossed a fresh recovery
boundary. Firmware, public ROS interfaces, QoS, the 100 ms feedback limits,
and the independent 198 ms firmware motor leases are unchanged.

## Observed incident

At `20:24:53` on `ackermann_0`, the host kernel recorded:

```text
usb 1-1-port4: disabled by hub (EMI?), re-enabling
```

The serial device disappeared and re-enumerated while the Agent systemd
service itself remained running. The MCU's retained diagnostics reported the
prior reset as independent-watchdog with `last_watchdog_task=2`
(`MicroRosTask`). The Discovery Server had remained up since `17:21`; these
records do not support attributing this incident to Discovery Server outage or
duplicate ROS node names.

After feedback stopped, the active hardware plugin returned `ERROR`.
Humble 2.54 called the SystemInterface error transition, removed its state and
command interfaces, and left the hardware component `unconfigured`. The drive
controllers could still appear `active`, but their hardware interfaces were
unavailable and no automatic configure/activate/reclaim sequence occurred.

The kernel event and retained MCU watchdog record describe the transport/MCU
incident. They do not prove the electrical cause of the USB hub disable or the
internal cause of the earlier MicroRosTask watchdog reset. Those remain
separate hardware/firmware investigations.

## Permanent correction

Both plugins now share one thread-safe reconnect gate. It enters recovery on:

- missing, invalid, or stale required feedback;
- missing, stale, or not-ready heartbeat;
- missing, invalid, duplicated, or wrongly owned supervisor authorization;
- Agent-session change; or
- wrap-aware MCU-uptime regression, including a reboot that reuses a session
  ID.

During recovery, every control cycle returns `OK`, publishes zero motor
commands, centers Ackermann PWM3, resets ADRCs and measurement filters,
reports zero wheel velocity, and freezes the last exported positions. A later
heartbeat and later valid sample from every required stream are mandatory.
Session or uptime restarts additionally require a different nonzero supervisor
configuration generation matching the current session.

An uninterrupted reference stream may resume after the gate passes. A
publisher that stopped during the outage cannot replay an old command because
the Ackermann and Mecanum controllers retain their existing 100 ms reference
timeouts.

True local failures still return `ERROR`. Both plugins now implement
`on_error()` to send safe commands, stop and join the private executor, release
all ROS endpoints, clear cached state and commands, and leave the component
reconfigurable.

Primary behavior reference:
[ros2_control 2.54 System implementation](https://github.com/ros-controls/ros2_control/blob/2.54.0/hardware_interface/src/system.cpp#L213-L253).

## Verification boundary

Focused software coverage exercises initial inhibition, same-session feedback
recovery, Agent-session change, uptime regression with a reused session ID,
old-generation rejection, current-generation acceptance, safe outputs,
live-reference resumption, and configure-after-`on_error()` for both plugins.

Power-disconnected reconnect checks on the six vehicles are passive evidence
only. They do not qualify powered motion, ADRC performance, USB signal
integrity, watchdog endurance, or release readiness.
