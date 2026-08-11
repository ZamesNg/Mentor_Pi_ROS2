# mentor_pi_bringup

This package contains the C++17 configuration supervisor, qualification tools,
and Python launch files. It does not open the serial device or start the
micro-ROS Agent. The Agent is an external systemd prerequisite on the onboard
Ubuntu 22.04 computer.

## Runtime contract

After each new nonzero MCU/Agent session, `/mentor_pi/configuration_supervisor`
applies the immutable controller configuration in this order:

1. `/mentor_pi/motors/set_model`
2. `/mentor_pi/pwm_servos/set_offsets`
3. `/mentor_pi/battery/set_low_threshold`

The transient-local topic `/mentor_pi/configuration/motion_enabled` stays false
until all three calls return a contract-consistent `OK`. The companion
`/mentor_pi/configuration/motion_authorization` value is zero while disarmed.
A nonzero value packs the configuration generation in its upper 32 bits and
the current `agent_session_id` in its lower 32 bits.

Hardware adapters, commissioning tools, and trackers accept this value only
from the single publisher named `/mentor_pi/configuration_supervisor`, require
a nonzero generation, and require its session to match the live heartbeat.
Invalid configuration, a lost session, stale feedback, a duplicate publisher,
or supervisor exit therefore leaves motion at zero. PWM and bus servos retain
their documented last accepted state on host loss.

## Manual launch

Build the workspace from the repository root:

```sh
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test
```

Onboard, first verify the separately managed Agent:

```sh
systemctl is-active mentor-pi-agent.service
```

Only after completing the passive safety gates, launch an application manually:

```sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
source /etc/mentor-pi/agent.env
export ROS_DOMAIN_ID
RRCLITE_RUNTIME_ACK=PID_FIRMWARE_ACTUATORS_PREPARED \
ros2 launch mentor_pi_hardwares mecanum.launch.py
```

`controller.launch.py` starts only the configuration supervisor. Its exit shuts
down the containing launch. `vehicle.launch.py` keeps the supervisor, hardware
adapters, controller manager, and optional tracker in the same launch graph;
the Agent remains outside that graph.

## Evidence and qualification

The `qualification_monitor`, `qualification_campaign`, and
`capture_board_diagnostics` executables remain bounded evidence tools. The
campaign motor path publishes literal zero targets only; it may still operate
other peripherals and therefore requires the documented guarded fixture.
Install the native capture tool and its protected evidence directory with:

```sh
sudo make install-evidence-tools
```

Then use the root integration commands from the onboard tutorial. A successful
software preflight or campaign never establishes PID performance, powered
motion safety, or release qualification. Those claims require recorded HIL and
instrument evidence from the ordered onboard tutorial.

## Build environment

Ubuntu 22.04 with ROS 2 Humble is the native production platform. macOS and
other Linux distributions build and test this package through the repository's
VS Code Dev Container. The Dev Container is not an onboard runtime, service
installer, HIL fixture, or release-evidence environment.
