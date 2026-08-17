# Vehicle State and Control Constraints

This document defines planner constraints for the current JGA27 Mentor Pi
vehicle configuration. All quantities use metres, radians, and seconds.
Forward and reverse limits are symmetric, and no operational safety derating
is included.

The acceleration limits and Ackermann steering-rate limit are provisional
controller/setpoint ceilings. They have not been qualified as physical
actuator limits by guarded HIL.

## Mentor Pi parameter values

The constraints below use geometry parameters rather than embedding Mentor Pi
dimensions in their equations. The current vehicle values are:

```text
wheel_radius                  = 0.0325 m
wheelbase                     = 0.135 m
wheel_track                   = 0.140 m
rear_axle_to_geometry_center  = 0.0675 m
mecanum_projection_sum        = 0.140 m
```

`wheelbase`, `wheel_track`, and `rear_axle_to_geometry_center` apply to the
Ackermann vehicle. `mecanum_projection_sum` is the sum of the Mecanum robot
center projections on its body X and Y axes.

## Source actuator constraints

### `DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`

```text
|wheel_angular_speed| <= 37.699112 rad/s
```

This is the enforced JGA27 output-shaft speed limit. The motor output shaft is
rigidly coupled to its driven wheel.

### `DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT`

```text
|wheel_angular_acceleration| <= 188.495559 rad/s^2
```

This is the provisional angular-acceleration ceiling derived from the current
motor-controller model and output limit.

## Ackermann

State and control vectors:

```text
state   = (x, y, yaw, v, steering)
control = (acc, steering_rate)
```

`x` and `y` locate the vehicle geometry center. `v` is the longitudinal speed
of the rear-axle midpoint.

Dynamics:

```text
x_dot        = v * cos(yaw)
               - (rear_axle_to_geometry_center / wheelbase)
                 * v * tan(steering) * sin(yaw)
y_dot        = v * sin(yaw)
               + (rear_axle_to_geometry_center / wheelbase)
                 * v * tan(steering) * cos(yaw)
yaw_dot      = v * tan(steering) / wheelbase
v_dot        = acc
steering_dot = steering_rate
```

`x` and `y` have no actuator position bound, and `yaw` has no actuator heading
bound.

### `ACKERMANN_STEERING_ANGLE_LIMIT`

```text
-0.6 <= steering <= 0.6 rad
```

This limits the logical steering position sent to the steering servo.

### `ACKERMANN_STEERING_RATE_LIMIT`

```text
|steering_rate| <= 60 rad/s
```

This is the provisional steering setpoint-rate ceiling derived from the
configured `0.020 s` PWM interpolation.

### `ACKERMANN_REAR_LEFT_MOTOR_SPEED_LIMIT`

```text
|v * (1 - wheel_track * tan(steering) / (2 * wheelbase)) / wheel_radius|
    <= 37.699112 rad/s
```

This keeps the driven rear-left wheel within
`DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`.

### `ACKERMANN_REAR_RIGHT_MOTOR_SPEED_LIMIT`

```text
|v * (1 + wheel_track * tan(steering) / (2 * wheelbase)) / wheel_radius|
    <= 37.699112 rad/s
```

This keeps the driven rear-right wheel within
`DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`.

### `ACKERMANN_REAR_LEFT_MOTOR_ACCELERATION_LIMIT`

```text
|(
  acc * (1 - wheel_track * tan(steering) / (2 * wheelbase))
  - wheel_track * v * steering_rate
    / (2 * wheelbase * cos(steering)^2)
) / wheel_radius| <= 188.495559 rad/s^2
```

This keeps the driven rear-left wheel within
`DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT` while longitudinal acceleration and
steering rate change together.

### `ACKERMANN_REAR_RIGHT_MOTOR_ACCELERATION_LIMIT`

```text
|(
  acc * (1 + wheel_track * tan(steering) / (2 * wheelbase))
  + wheel_track * v * steering_rate
    / (2 * wheelbase * cos(steering)^2)
) / wheel_radius| <= 188.495559 rad/s^2
```

This keeps the driven rear-right wheel within
`DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT` while longitudinal acceleration and
steering rate change together.

## Mecanum

State and control vectors:

```text
state   = (x, y, yaw, vx, vy, v_yaw)
control = (ax, ay, a_yaw)
```

`vx` and `vy` are body-frame linear velocities.

Dynamics:

```text
x_dot     = vx * cos(yaw) - vy * sin(yaw)
y_dot     = vx * sin(yaw) + vy * cos(yaw)
yaw_dot   = v_yaw
vx_dot    = ax
vy_dot    = ay
v_yaw_dot = a_yaw
```

`x` and `y` have no actuator position bound, and `yaw` has no actuator heading
bound.

### `MECANUM_FRONT_LEFT_MOTOR_SPEED_LIMIT`

```text
|(vx - vy - mecanum_projection_sum * v_yaw) / wheel_radius|
    <= 37.699112 rad/s
```

This keeps the driven front-left wheel within
`DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`.

### `MECANUM_FRONT_RIGHT_MOTOR_SPEED_LIMIT`

```text
|(vx + vy + mecanum_projection_sum * v_yaw) / wheel_radius|
    <= 37.699112 rad/s
```

This keeps the driven front-right wheel within
`DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`.

### `MECANUM_REAR_LEFT_MOTOR_SPEED_LIMIT`

```text
|(vx + vy - mecanum_projection_sum * v_yaw) / wheel_radius|
    <= 37.699112 rad/s
```

This keeps the driven rear-left wheel within
`DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`.

### `MECANUM_REAR_RIGHT_MOTOR_SPEED_LIMIT`

```text
|(vx - vy + mecanum_projection_sum * v_yaw) / wheel_radius|
    <= 37.699112 rad/s
```

This keeps the driven rear-right wheel within
`DRIVEN_WHEEL_ANGULAR_SPEED_LIMIT`.

### `MECANUM_FRONT_LEFT_MOTOR_ACCELERATION_LIMIT`

```text
|(ax - ay - mecanum_projection_sum * a_yaw) / wheel_radius|
    <= 188.495559 rad/s^2
```

This keeps the driven front-left wheel within
`DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT`.

### `MECANUM_FRONT_RIGHT_MOTOR_ACCELERATION_LIMIT`

```text
|(ax + ay + mecanum_projection_sum * a_yaw) / wheel_radius|
    <= 188.495559 rad/s^2
```

This keeps the driven front-right wheel within
`DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT`.

### `MECANUM_REAR_LEFT_MOTOR_ACCELERATION_LIMIT`

```text
|(ax + ay - mecanum_projection_sum * a_yaw) / wheel_radius|
    <= 188.495559 rad/s^2
```

This keeps the driven rear-left wheel within
`DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT`.

### `MECANUM_REAR_RIGHT_MOTOR_ACCELERATION_LIMIT`

```text
|(ax - ay + mecanum_projection_sum * a_yaw) / wheel_radius|
    <= 188.495559 rad/s^2
```

This keeps the driven rear-right wheel within
`DRIVEN_WHEEL_ANGULAR_ACCELERATION_LIMIT`.

## Qualification status

- Driven-wheel speed and steering-position bounds are enforced software limits
  for the current JGA27 vehicle configuration.
- The acceleration ceiling is derived from the current motor-controller model
  and output ceiling; it is not a measured physical acceleration limit.
- The steering-rate ceiling is derived from the configured `0.020 s` PWM
  setpoint interpolation; it is not a measured physical servo rate.
- Physical characterization must replace the provisional acceleration and
  steering-rate values before treating them as qualified actuator limits.
