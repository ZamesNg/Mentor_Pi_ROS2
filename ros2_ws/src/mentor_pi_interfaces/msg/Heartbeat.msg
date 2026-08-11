uint8 BOOTING=0
uint8 READY=1
uint8 DEGRADED=2
uint8 FAULT=3

uint16 TIME_SYNCHRONIZED=1
uint16 MOTOR_WATCHDOG_ACTIVE=2
uint16 LOW_BATTERY=4
uint16 IMU_HEALTHY=8
uint16 BUS_SERVO_BUSY=16

builtin_interfaces/Time stamp
uint32 sequence
uint32 uptime_ms
uint32 agent_session_id
uint8 state
uint16 flags
