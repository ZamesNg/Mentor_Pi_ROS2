# RRCLite v2 micro-ROS runtime

This directory contains the MCU-owned ROS runtime. It is deliberately isolated
from hardware workers: generated ROS messages are converted into portable
domain structs, validated, and copied through `RuntimeHooks`. No worker sees an
`rcl`, `rmw`, or generated-message pointer.

## Implemented graph

The runtime creates node `/mentor_pi/controller` with exactly:

- seven publishers: `motors/state`, `pwm_servos/state`, `imu`,
  `buttons/events`, `battery/state`, `heartbeat`, and `diagnostics`;
- seven subscriptions, registered with the executor in the contract order:
  motor, PWM servo, bus servo, LED, buzzer, RGB, and OLED command;
- six manually pumped services: motor model, PWM offsets, bus get/configure/
  stop, and battery threshold.

Every endpoint uses explicit volatile keep-last QoS. Motion topics are best
effort depth one. Discrete topics and all services are reliable depth one; the
button publisher is reliable depth eight. Reliable publishers and services
have a 10 ms XRCE session timeout.

## Runtime properties

- `SAFE_BOOT -> WAIT_AGENT -> CREATE_ENTITIES -> ACTIVE -> TEARDOWN -> BACKOFF`
  is a fixed state machine with 100/200/400/800/1600/2000 ms capped backoff.
- Entity creation and teardown are cursor-driven. Every `RunOnce()` slice in
  either phase starts at most one middleware boundary, so the task heartbeat is
  advanced between calls. The fixed graph uses 47 ordered creation boundaries;
  a fully constructed normal teardown uses 24 boundaries and finalizes its ROS
  entities in exact reverse construction order.
- Each creation boundary is limited to 40 ms, initial time sync to 20 ms, and
  the whole creation phase to 2 s. Each remote finalizer is limited to 10 ms and
  the remote teardown phase to 500 ms. When the next 10 ms finalizer no longer
  fits, teardown spends one slice setting the context destroy timeout to zero,
  then completes bounded local cleanup without remote waits. Finalizer failures
  are recorded and never prevent later reverse-order cleanup.
- The custom framing transport is the existing CH9102F/USART1 adapter. Any
  USART framing/noise/overrun/parity error, generic DMA error, RX-ring overrun,
  TX DMA failure, or TX timeout disarms motors and tears down the session.
- The executor owns only the seven subscriptions. Services are taken and
  responded to nonblockingly by `MicroRosTask`; motor/PWM/battery have one slot
  each and all bus services share one slot. Stop is inspected first. A timed-out
  slot remains as a bounded tombstone until its generation-tagged worker
  completion is drained, so late work cannot satisfy or permanently wedge a
  later request.
- ACTIVE work rotates strictly through service, reliable-telemetry, and
  maintenance classes. A shared per-slice permit allows at most one service
  response, reliable publication, Agent ping, or time-sync attempt; an idle
  class does not donate its turn. Service slots and publishers use persistent
  round-robin cursors, so traffic in one endpoint cannot drain a FIFO or starve
  Agent maintenance.
- Callbacks perform conversion, validation, and bounded handoff only. They
  never wait for a peripheral and never retain middleware memory.
- Telemetry rates are 50/20/50 Hz for motor/PWM/IMU, 1 Hz battery and
  diagnostics, 2 Hz heartbeat, and at most 20 button events per second. Each
  slice publishes at most one due best-effort sample and a selected reliable
  class publishes at most one due reliable sample. The
  runtime retains each owner task's newest complete snapshot between updates;
  an empty SPSC mailbox never creates a synthetic zero-state sample.
- The two OLED strings use preassigned 24-byte buffers. All other custom data
  is fixed size.
- A 48 KiB resettable arena is open only during entity creation, sealed before
  `ACTIVE`, opened for deallocation-only teardown, and reset as one unit. The
  executor is spun once with a zero timeout before sealing because Jazzy rclc
  lazily creates its wait set on first spin. A post-seal allocation latches a
  fatal invariant, performs best-effort teardown, and stops the MicroRosTask
  heartbeat so the safety supervisor forces an IWDG reset into safe boot.
- Time sync is attempted for 20 ms at creation, retried for 10 ms every five
  seconds until successful, and repeated at least every 60 seconds. Control
  timing always uses the monotonic hook. Negative epoch corrections are
  deferred and each publisher's timestamp is nondecreasing.

## Integration

Before starting the six static tasks, the firmware composition root must build
a complete `RuntimeHooks`, call `ConfigureMicroRosRuntime()`, and register
`MentorPiMicroRosTaskMain` as the `MicroRosTask` entry. Hook dispatch functions
must be nonblocking copies into owner-task storage; service poll functions must
match both fields of `ServiceToken` before returning a completion.

The generated Jazzy library must retain the limits in
`config/microros_colcon.meta`: 1 node, 7 publishers, 7 subscriptions, 6
services, no clients, history 8, 512-byte MTU, 40 ms create timeout, and 10 ms
destroy timeout.

For portable cursor/order/deadline/failure tests and a generated-header compile
check:

```sh
cmake -S firmware/mentor_pi_mcu/app/microros \
  -B build/mentor_pi_microros -DBUILD_TESTING=ON
cmake --build build/mentor_pi_microros
ctest --test-dir build/mentor_pi_microros --output-on-failure
```

The portable tests exercise the lifecycle cursors and a fake boundary driver;
they do not link `MicroRosRuntime` or Jazzy middleware. The compile check builds
the real runtime translation units against generated headers but does not link
production firmware. The final Arm link and on-target behavior still require
the hardware composition root, FreeRTOS/HAL objects, `libmicroros.a`, linker
map, and HIL.
