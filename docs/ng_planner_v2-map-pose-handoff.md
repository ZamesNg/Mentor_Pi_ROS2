# ng_planner_v2 map-pose migration handoff

## Baselines and ownership

This handoff targets `ng_planner_v2` branch `experiment` as inspected at
commit `0869bceb8492140970388224e6c1e1341cc3df46`. The corresponding Mentor Pi
interface is implemented by commit
`b979b446c9a367a51b03813452ff86730e767287`.

Make the changes in `ng_planner_v2`; do not patch, vendor, or duplicate Mentor
Pi code there. The Mentor Pi tracker and simulator have one common public pose
contract. The `vehicle_pose` name identifies the adapter node; `vehicle/pose`
is the topic carrying its data.

## Authoritative ROS contract

For each robot namespace `<robot>`:

```text
raw mocap input:    /vrpn_mocap/<robot>/pose
                    geometry_msgs/msg/PoseStamped, frame_id=map

vehicle pose:       /<robot>/vehicle/pose
                    geometry_msgs/msg/PoseStamped, frame_id=map

vehicle transform:  map -> <robot>/base_footprint

trajectory output:  /<robot>/trajectory_tracker/reference_trajectory
                    mentor_pi_tracking_interfaces/msg/PolynomialTrajectory
                    header.frame_id=map
```

The mocap rigid body and `base_footprint` both describe the vehicle geometry
center. Do not apply an Ackermann rear-axle offset to physical mocap data.
Rigid-body names must match robot namespaces; otherwise remap each raw VRPN
topic explicitly before it reaches Mentor Pi.

There is no physical `odom` frame, public Mentor encoder odometry, or
`map -> odom` transform. Wheel links, `imu_link`, `base_link`, and Ackermann
`rear_axle_footprint` remain below `base_footprint` in the Mentor URDF.

## Required ng_planner_v2 changes

### Common state and trajectory frame

- Require every experiment scenario `frame_id` to be exactly `map`; remove the
  current `odom` default and reject any other value.
- Change `BuildPolynomialTrajectory`, `ValidatePolynomialTrajectory`, replay,
  artifact, visualization, and experiment tests to require `map`.
- Keep the existing trajectory message type, scheduling semantics, and
  `/<robot>/trajectory_tracker/reference_trajectory` topics unchanged.
- Use `/<robot>/vehicle/pose` as the authoritative geometry-center pose in
  both physical and simulation modes.

The planner still needs measured body-frame velocities to initialize its
Ackermann and Mecanum state vectors. It may retain `nav_msgs/msg/Odometry`
inside the experiment packages as a pose-plus-filtered-twist container. Feed
the existing pose-to-velocity estimator from `/<robot>/vehicle/pose` and set:

```text
source_frame_id: map
output_frame_id: map
source_to_output:       (0 m, 0 m, 0 rad)
rigid_body_to_base:     (0 m, 0 m, 0 rad)
```

That internal state estimate must publish no TF and must not become a Mentor
Pi API. Its linear twist remains expressed in the vehicle body frame, as the
current `OdometryEstimator` already produces. Update comments and diagnostics
to call it a mocap state estimate rather than an encoder-derived pose.

### Physical experiment launch

- Keep exactly one `vrpn_mocap` receiver. Its current
  `/vrpn_mocap/client` configuration already publishes `frame_id: map` at
  100 Hz and uses receipt timestamps.
- Stop feeding raw VRPN poses directly into a second geometry transform.
  Mentor Pi consumes the raw topic and publishes the validated
  `/<robot>/vehicle/pose`; the planner velocity estimator consumes that common
  vehicle pose.
- Remove `source_to_odom` and per-robot geometry-center correction from the
  physical experiment launch. The mocap pose is already the geometry center.
- Do not add a TF broadcaster to `optitrack_odometry`; Mentor Pi owns
  `map -> <robot>/base_footprint`.

### Simulation experiment launch

The copies of `mentor_simulation_actions.py` in `follower_experiment` and
`stabilizer_experiment` currently use the retired `vehicle_odometry`
executable. Update both copies to match the Mentor simulation contract:

- keep controller odometry remapped to the internal
  `/<robot>/vehicle/_controller_odometry` topic;
- launch `mentor_pi_hardwares/vehicle_pose` in `controller_odometry` mode;
- publish `/<robot>/vehicle/pose` and `map -> <robot>/base_footprint`;
- remove `simulation_odometry_alias_node` and every static common-odom
  transform from both experiment launches.

Use these adapter parameters:

```text
input_type: controller_odometry
geometry_center_frame_id: <robot>/base_footprint
output_frame_id: map
output_origin_yaw_rad: initial_yaw_rad
```

For Mecanum, use zero source offset and set the output origin directly to the
requested initial geometry-center position. For Ackermann, use
`source_to_geometry_center_m = 0.0675` and:

```text
output_origin_x_m = initial_x_m - 0.0675 * cos(initial_yaw_rad)
output_origin_y_m = initial_y_m - 0.0675 * sin(initial_yaw_rad)
```

This compensation makes the first public simulation pose exactly the requested
geometry-center pose. Feed that same public pose through the planner's
pose-to-velocity estimator, so physical and simulation state acquisition have
the same boundary.

Apply the migration consistently to `follower_experiment` and
`stabilizer_experiment`, including their scenario loaders, scenario YAML,
launch contracts, runtime tests, READMEs, and visualization fixed frames.

## Focused acceptance criteria

1. Physical and simulation trajectories are accepted only with
   `header.frame_id == "map"`.
2. Both experiment applications build their state position and yaw from the
   geometry-center `/<robot>/vehicle/pose` without another spatial offset.
3. The retained velocity estimator produces finite body-frame `vx`, `vy`, and
   yaw rate in a `map`-framed internal state estimate.
4. Ackermann state construction continues deriving rear speed and steering
   from the body-frame speed/yaw-rate estimate; Mecanum continues using body
   `vx`, `vy`, and yaw rate.
5. Simulation starts at the requested map pose for zero, positive, and negative
   yaw and follows the same Mentor tracker API as physical vehicles.
6. The graph contains no `vehicle/odometry`, `vehicle/tf_odometry`,
   `vehicle_odometry`, `simulation_odometry_alias`, `<robot>/odom`, or
   `map -> odom` compatibility path.
7. There is exactly one `map -> <robot>/base_footprint` authority per robot.
8. Existing scheduling, command gating, terminal hold, logging, replay, and
   solver behavior remain unchanged apart from the frame migration.

Run the repository's authoritative `./scripts/build_ros2.sh --release` after
the focused unit and launch tests. Record the Mentor Pi interface commit above
as the integration dependency in the resulting `ng_planner_v2` change.
