# Lightweight vehicle simulation

The lightweight simulator is a deterministic ros2_control plant for controller,
planner, pose, TF, and visualization development. It does not use Gazebo,
emulate the MCU ROS API, or provide hardware or release evidence. Its launch
starts the vehicle-matched ADRC trajectory tracker by default.

Build only the affected package in the Dev Container or on native Ubuntu 22.04:

```sh
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  mentor_pi_tracking mentor_pi_hardwares
source install/setup.bash
```

Start exactly one vehicle plant. `vehicle_type` is required; the initial pose
is the geometry center expressed in `map`:

```sh
ros2 launch mentor_pi_hardwares simulation.launch.py \
  vehicle_type:=ackermann \
  robot_name:=ackermann_sim \
  initial_x_m:=0.0 \
  initial_y_m:=0.0 \
  initial_yaw_rad:=0.0 \
  tracking_controller:=auto \
  tracking_algorithm:=adrc
```

Use `vehicle_type:=mecanum` for a Mecanum plant. Multiple plants may run under
different `robot_name` namespaces. Select MPC with
`tracking_algorithm:=mpc`. The tracker consumes
`/<robot>/trajectory_tracker/reference_trajectory`, consumes geometry-center
`/<robot>/vehicle/pose`, and publishes `/<robot>/vehicle/reference`. Scheduled
trajectories must use `header.frame_id: map`.

For a direct controller-reference test, launch without the competing tracker:

```sh
ros2 launch mentor_pi_hardwares simulation.launch.py \
  vehicle_type:=ackermann \
  robot_name:=ackermann_sim \
  tracking_controller:=none
```

Then publish the same reference used by the physical vehicle:

```sh
ros2 topic pub --rate 20 \
  /ackermann_sim/vehicle/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.1}, angular: {z: 0.2}}}'
```

The public outputs are `/ackermann_sim/vehicle/pose`, `/joint_states`, `/tf`,
`/tf_static`, and `/ackermann_sim/robot_description`. TF contains
`map -> ackermann_sim/base_footprint`; controller odometry remains only on its
internal remapped topic. The standard controller reference timeout
still applies. The tracker uses the sum of trajectory segment durations as its
execution horizon, then actively holds the terminal pose with zero reference
derivatives until replacement or cancellation. The default tracker also exposes
`/ackermann_sim/trajectory_tracker/cancel` and publishes
`diagnostic_msgs/msg/DiagnosticArray` on
`/ackermann_sim/trajectory_tracker/diagnostics`.

## Foxglove

Foxglove is separate from the plant so one bridge can visualize multiple
namespaces:

```sh
ros2 launch mentor_pi_hardwares foxglove.launch.py
```

Connect Foxglove to `ws://localhost:8765`. In a 3D panel, select
`map` as the display frame and add a URDF layer sourced from the
`/ackermann_sim/robot_description` topic. Select `/tf` and `/tf_static` for
transforms. Leave the URDF layer's **Frame prefix** empty: the generated
description already uses `robot_name` in every link frame, exactly matching the
published TF tree. The description uses the original Mentor Pi body, wheel,
camera, and lidar STL visuals from the recorded `ng_planner` source commit;
collision and inertial geometry remain the lightweight primitives. The bridge
defaults to loopback; override `address` only when deliberate network access is
required. The Dev Container forwards port 8765. Mesh provenance and the
separate `NOASSERTION` license status are recorded in the installed
`config/MESH_PROVENANCE.md`.

## Model boundary

The controller manager runs at 30 Hz using wall time. Each simulated wheel is
limited to `37.699112 rad/s` and `188.495559 rad/s^2`; position uses
trapezoidal integration. Ackermann uses the mean front steering command as its
single physical steering state, bounded to `+/-0.6 rad` and `60 rad/s`. The
physical ROS-to-MCU connector signs are not applied because simulated joint
states already use ROS chassis-direction semantics.

The existing vehicle controllers reconstruct chassis motion from those joint
states. The common adapter converts their hidden numerical estimate to a
geometry-center `PoseStamped` in `map`; Ackermann applies its `0.0675 m`
rear-axle offset and Mecanum uses a zero offset. The tracker uses that pose plus
its static `37.699112 rad/s` driven-wheel limit and creates no motor-state,
heartbeat, authorization, supervisor, or firmware endpoints.
Simulation adds no `/clock`, noise, slip, collision, terrain, IMU, motor time
constant, or servo time constant. The acceleration and steering-rate limits
are provisional software ceilings documented in
[constraint.md](constraint.md), not measured physical actuator performance.
