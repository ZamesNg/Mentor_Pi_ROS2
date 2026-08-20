# Mentor Pi ROS 2 workspace

This ROS 2 Humble workspace owns the Mentor Pi command and hardware boundary.
It contains three project packages under `src/`:

- `mentor_pi_interfaces`
- `mentor_pi_bringup`
- `mentor_pi_hardwares`

The workspace also builds the pinned, reviewed
`ackermann_steering_controller`, `mecanum_drive_controller`, and
`steering_controllers_library` overlay declared in `dependencies.repos`.
The overlay preserves receipt-time handling for zero-stamped commands without
emitting a warning.

On Ubuntu 22.04, or inside the repository Dev Container on other hosts:

```sh
make -C ros2_ws deps
make -C ros2_ws build
make -C ros2_ws test
```

The micro-ROS Agent remains a separate boot service. ROS applications start
manually. `vehicle.launch.py` starts the physical command-only stack selected
by the generated profile; `simulation.launch.py` starts the corresponding
command-only ros2_control simulation.

This repository does not own global pose, TF, polynomial trajectories, or an
outer trajectory tracker. An external application publishes bounded
`geometry_msgs/msg/TwistStamped` messages to
`/<robot>/vehicle/reference`. Commands use a 100 ms receipt-time timeout.
