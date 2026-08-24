# Central-planner handoff

Mentor Pi is a command-only vehicle runtime. Global pose, mocap conversion,
trajectory messages, and outer trajectory tracking are owned by the central
planner repository.

## Vehicle input

Each vehicle accepts:

```text
/<robot>/vehicle/reference
geometry_msgs/msg/TwistStamped
```

The controller consumes Mecanum `vx/vy/wz` or Ackermann rear-axle `vx/wz`.
Every reference refreshes a 100 ms deadline from local receipt time; zero,
stale, current, or unsynchronized sender timestamps do not govern freshness.

## State ownership

The central application consumes
`/vrpn_mocap/<robot>/pose`, validates that it is the geometry-center pose in
`map`, and publishes:

- `/<robot>/vehicle/pose`;
- planner-private map-frame odometry; and
- the sole `map -> <robot>/base_footprint` transform.

Mentor controller odometry remains private under
`/<robot>/vehicle/_controller_*` and never owns a map transform.

## Simulation

Mentor simulation exposes the same command topic and private controller
odometry. The central adapter applies the initial map origin and Ackermann
rear-axle-to-geometry-center offset before publishing the public state.
