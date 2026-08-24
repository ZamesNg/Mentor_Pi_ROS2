# Command-only ros2_control simulation

The simulation launch starts robot state publication, the simulation hardware
plugin, controller manager, and grouped controller spawner. It intentionally
does not start a global-pose adapter or trajectory tracker.

```sh
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch mentor_pi_hardwares simulation.launch.py \
  vehicle_type:=ackermann robot_name:=ackermann_sim
```

Publish a bounded stamped reference from an external application:

```sh
ros2 topic pub --rate 60 /ackermann_sim/vehicle/reference \
  geometry_msgs/msg/TwistStamped \
  '{header: {stamp: {sec: 0, nanosec: 0}}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}'
```

Every callback records the controller computer's local receipt time for
freshness; a zero, stale, current, or unsynchronized sender header stamp cannot
shorten or extend the command lifetime. The controller manager runs at 50 Hz
and consumes the newest best-effort reference available on each update.
Publishing above 50 Hz exercises that latest-value behavior; if a new reference
is not received within 100 ms, the next controller update commands zero.

Controller odometry and TF odometry are remapped to private diagnostic topics
under `/<robot>/vehicle/_controller_*`. Global map pose and
`map -> base_footprint` are owned by the external state adapter.
