# Lightweight vehicle simulation

The lightweight simulator is a deterministic ros2_control plant for controller,
planner, odometry, TF, and visualization development. It does not use Gazebo,
start the trajectory tracker, emulate the MCU ROS API, or provide hardware or
release evidence.

Build only the affected package in the Dev Container or on native Ubuntu 22.04:

```sh
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mentor_pi_hardwares
source install/setup.bash
```

Start exactly one vehicle plant. `vehicle_type` is required; the initial pose
is the geometry center expressed in the vehicle's namespaced odometry frame:

```sh
ros2 launch mentor_pi_hardwares simulation.launch.py \
  vehicle_type:=ackermann \
  robot_name:=ackermann_sim \
  initial_x_m:=0.0 \
  initial_y_m:=0.0 \
  initial_yaw_rad:=0.0
```

Use `vehicle_type:=mecanum` for a Mecanum plant. Multiple plants may run under
different `robot_name` namespaces. Send the same reference used by the physical
vehicle:

```sh
ros2 topic pub --rate 20 \
  /ackermann_sim/vehicle/reference \
  geometry_msgs/msg/TwistStamped \
  '{twist: {linear: {x: 0.1}, angular: {z: 0.2}}}'
```

The public outputs are `/ackermann_sim/vehicle/odometry`,
`/ackermann_sim/vehicle/tf_odometry`, `/joint_states`, `/tf`, `/tf_static`, and
`/ackermann_sim/robot_description`. The standard controller reference timeout
still applies.

## Foxglove

Foxglove is separate from the plant so one bridge can visualize multiple
namespaces:

```sh
ros2 launch mentor_pi_hardwares foxglove.launch.py
```

Connect Foxglove to `ws://localhost:8765`. In a 3D panel, select
`ackermann_sim/odom` as the display frame and add a URDF layer sourced from the
`/ackermann_sim/robot_description` topic. Select `/tf` and `/tf_static` for
transforms. The bridge defaults to loopback; override `address` only when
deliberate network access is required. The Dev Container forwards port 8765.

## Model boundary

The controller manager runs at 30 Hz using wall time. Each simulated wheel is
limited to `37.699112 rad/s` and `188.495559 rad/s^2`; position uses
trapezoidal integration. Ackermann uses the mean front steering command as its
single physical steering state, bounded to `+/-0.6 rad` and `60 rad/s`. The
physical ROS-to-MCU connector signs are not applied because simulated joint
states already use ROS chassis-direction semantics.

The existing vehicle controllers reconstruct chassis motion from those joint
states. Ackermann raw rear-axle odometry is converted to geometry-center
odometry by the common adapter; Mecanum uses its zero-offset path. Simulation
adds no `/clock`, noise, slip, collision, terrain, IMU, motor time constant, or
servo time constant. The acceleration and steering-rate limits are provisional
software ceilings documented in [constraint.md](constraint.md), not measured
physical actuator performance.
