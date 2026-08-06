# RRCLite v2 Qualification Evidence Ledger

Status: blank release-candidate template; **not qualification evidence**  
Applies to: D5 qualification and D6 release review

## Purpose

Copy this template into an immutable evidence directory for each release
candidate and fill it from the raw records required by
[Verification](verification.md). The copy binds every result to one source
revision, host revision, firmware digest, board, fixture, environment, and time
window. This framework copy remains `NOT_RUN` and must never be edited to imply
that a candidate passed.

An automated campaign result is only one input to this ledger. In particular,
`qualification_campaign` deliberately reports
`release_qualification: INCOMPLETE`, even when its observable execution checks
pass. A software-observed ROS graph, diagnostic counter, or heartbeat gap is
not a substitute for an independent serial capture, a physical measurement, or
an operator record required by a verification case.

## Candidate identity

Replace every `REQUIRED` value in the copied ledger before review.

| Field | Candidate value |
| --- | --- |
| Release-candidate ID | `REQUIRED` |
| Firmware source revision | `REQUIRED` |
| Host source revision | `REQUIRED` |
| Interface revision | `REQUIRED` |
| Firmware ELF SHA-256 | `REQUIRED` |
| Firmware BIN/HEX SHA-256 | `REQUIRED` |
| Host artifact SHA-256 or manifest | `REQUIRED` |
| micro-ROS Agent revision and binary SHA-256 | `REQUIRED` |
| ROS distribution | `jazzy` |
| Host OS and architecture | `REQUIRED` |
| Board model and PCB revision | `RRCLite V1.0 / REQUIRED` |
| Board or fixture serial | `REQUIRED` |
| Fixture revision and calibration record | `REQUIRED` |
| Motor image authority | `LOCKED / COMMISSIONING / REVIEWED_PRODUCTION` |
| Test start/end UTC | `REQUIRED` |
| Primary operator | `REQUIRED` |
| Independent reviewer | `REQUIRED` |
| Immutable evidence root | `REQUIRED` |

Record the exact build, flash, launch, campaign, capture, and analysis commands
in the candidate evidence root. Archive the final ROS parameters and controller
YAML beside this ledger. If any identity field changes, create a new candidate
record and rerun every affected case; do not merge results from incompatible
candidates.

## Result vocabulary

Use exactly one of these values for every verification row:

- `PASS`: every acceptance clause was exercised on the bound candidate and all
  required raw evidence is linked.
- `FAIL`: at least one acceptance clause failed. Record the issue and preserve
  the failing raw evidence.
- `NOT_RUN`: the case has not been executed completely.
- `BLOCKED`: execution cannot proceed; record the owner, reason, and unblock
  condition.
- `NOT_APPLICABLE`: permitted only where the normative case explicitly allows
  it, with reviewer rationale.

`NOT_OBSERVED`, `NOT_EVALUATED`, or a skipped JUnit case describes an individual
metric and cannot make a verification row `PASS`. A rerun after a failed
endurance or recovery campaign covers the complete normative duration or cycle
count.

## Automated campaign artifacts

For each `load500`, `soak`, `reconnect_usb`, `reconnect_agent`, or `reset_mcu`
run, link its immutable output directory. The C++ campaign tool writes:

| Artifact | Use | Release limitation |
| --- | --- | --- |
| `summary.json` | Revision/session-bound outcome, counters, rates, and explicit unobservable metrics. | `release_qualification` remains `INCOMPLETE`. |
| `metrics.csv` | Machine-readable metric status and thresholds. | Internal diagnostic traffic is not independent wire evidence. |
| `session-transitions.csv` | Bounded ledger of observed session changes and ROS endpoint recovery. | It does not prove the physical outage duration or actuator safe state. |
| `junit.xml` | CI ingestion of observable pass/fail and skipped metrics. | A green observable suite is not D5 acceptance. |

Attach the independent records called for by the relevant verification case,
including the escaped-wire capture, logic-analyzer/current traces, target stack
watermarks, allocation trace, physical outage/reset timestamps, actuator-state
observations, button stimulus, and bus-servo baseline/restore record. State
`NOT_OBSERVED` when an instrument was absent; never synthesize a pass from a ROS
counter.

## Endurance and recovery campaign register

| Verification ID | Campaign output | Independent evidence | Exact duration/cycles | Result | Reviewer/date |
| --- | --- | --- | --- | --- | --- |
| `VER-LOAD-500-001` | `REQUIRED` | `REQUIRED` | 3,600 s / 1,800,000 motor messages | `NOT_RUN` | `REQUIRED` |
| `VER-SOAK-001` | `REQUIRED` | `REQUIRED` | 24 uninterrupted hours | `NOT_RUN` | `REQUIRED` |
| `VER-RECONNECT-USB-001` | `REQUIRED` | `REQUIRED` | 100 operator/fixture-driven physical cycles | `NOT_RUN` | `REQUIRED` |
| `VER-RECONNECT-AGENT-001` | `REQUIRED` | `REQUIRED` | 100 operator/fixture-driven Agent cycles | `NOT_RUN` | `REQUIRED` |
| `VER-RESET-MCU-001` | `REQUIRED` | `REQUIRED` | 100 operator/fixture-driven MCU cycles | `NOT_RUN` | `REQUIRED` |

The campaign executable observes these recovery campaigns; it does not unplug
USB, kill the Agent, drive NRST, or power-cycle the MCU. Record the external
operator/fixture procedure and timestamp source in `Independent evidence`.

## Complete verification ledger

The evidence link points to raw data plus a short case report that checks every
acceptance clause in `verification.md`. Record a numeric margin where the case
has a limit; use `N/A` only for a genuinely nonnumeric clause.

| Verification ID | Result | Evidence link | Environment / measured margin | Issue or waiver | Reviewer/date |
| --- | --- | --- | --- | --- | --- |
| `VER-TRACE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-REVIEW-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-BUILD-HOST-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-BUILD-FW-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-BUILD-MOTOR-GATE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-SCOPE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-API-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-SERIALIZATION-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-CONFIG-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HOST-COMMISSION-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-VAL-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-MOTOR-GATE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-MBOX-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-BTN-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-LEASE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-SVC-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-MW-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-RXDMA-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-QOS-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-STATE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-TIME-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-UNIT-DIAG-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FUZZ-VAL-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FUZZ-TRN-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-ANALYSIS-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-INT-TRN-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-LED-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-BUZ-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-MOT-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-PWM-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-BUS-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-RGB-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-OLED-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-IMU-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-BTN-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-HIL-BAT-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-SAFE-WDG-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-LOAD-500-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-SOAK-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-TRAFFIC-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-SAFE-LEASE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-RECONNECT-USB-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-RECONNECT-AGENT-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-RESET-MCU-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-OVERFLOW-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-OVERFLOW-BTN-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FAULT-TX-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FAULT-TX-002` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FAULT-UART-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FAULT-BUS-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-FAULT-I2C-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-SAFE-BAT-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |
| `VER-RESOURCE-001` | `NOT_RUN` | `REQUIRED` | `REQUIRED` | — | `REQUIRED` |

## Deviations and approval

| Deviation ID | Affected requirement/case | Safety impact | Compensating evidence | Owner / expiry | Approval |
| --- | --- | --- | --- | --- | --- |
| None recorded | — | — | — | — | — |

D5 closes only when every mandatory row is `PASS`, every evidence link resolves
inside the immutable candidate record, all required independent measurements
are present, deviations are accepted under the project rules, and the reviewer
signs below.

| Approval | Name | UTC date | Signature or review record |
| --- | --- | --- | --- |
| Test lead | `REQUIRED` | `REQUIRED` | `REQUIRED` |
| MCU reviewer | `REQUIRED` | `REQUIRED` | `REQUIRED` |
| Host/ROS reviewer | `REQUIRED` | `REQUIRED` | `REQUIRED` |
| Release approver | `REQUIRED` | `REQUIRED` | `REQUIRED` |
