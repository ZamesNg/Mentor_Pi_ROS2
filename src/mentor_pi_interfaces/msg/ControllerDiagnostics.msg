# Subscription indices (arrays of length 7)
uint8 SUB_MOTORS=0
uint8 SUB_PWM_SERVOS=1
uint8 SUB_BUS_SERVOS=2
uint8 SUB_LEDS=3
uint8 SUB_BUZZER=4
uint8 SUB_RGB=5
uint8 SUB_OLED=6
uint8 SUB_COUNT=7

# Task indices (array of length 6)
uint8 TASK_SAFETY_SUPERVISOR=0
uint8 TASK_MOTOR_CONTROL=1
uint8 TASK_MICRO_ROS=2
uint8 TASK_BUS_SERVO=3
uint8 TASK_SENSOR=4
uint8 TASK_PERIPHERAL=5
uint8 TASK_COUNT=6
uint8 TASK_NONE=255

# RAM indices (arrays of length 2)
uint8 RAM_DMA_SRAM=0
uint8 RAM_CCM=1
uint8 RAM_CLASS_COUNT=2

# Peripheral indices (arrays of length 8)
uint8 PERIPH_BUS_SERVO=0
uint8 PERIPH_IMU=1
uint8 PERIPH_OLED=2
uint8 PERIPH_BATTERY_ADC=3
uint8 PERIPH_PWM_SERVO=4
uint8 PERIPH_LEDS=5
uint8 PERIPH_BUZZER=6
uint8 PERIPH_RGB=7
uint8 PERIPH_COUNT=8

# USART1 error indices (array of length 4)
uint8 USART1_FRAMING=0
uint8 USART1_NOISE=1
uint8 USART1_OVERRUN=2
uint8 USART1_PARITY=3
uint8 USART1_ERROR_COUNT=4

# Session states
uint8 SESSION_SAFE_BOOT=0
uint8 SESSION_WAIT_AGENT=1
uint8 SESSION_CREATE_ENTITIES=2
uint8 SESSION_ACTIVE=3
uint8 SESSION_TEARDOWN=4
uint8 SESSION_BACKOFF=5

# Safe-teardown reasons
uint8 TEARDOWN_NONE=0
uint8 TEARDOWN_AGENT_LOST=1
uint8 TEARDOWN_USART1_ERROR=2
uint8 TEARDOWN_RX_OVERRUN=3
uint8 TEARDOWN_TX_TIMEOUT=4
uint8 TEARDOWN_ENTITY_ERROR=5
uint8 TEARDOWN_MEMORY_VIOLATION=6
uint8 TEARDOWN_TASK_STALL=7

# Reset reasons
uint8 RESET_POWER_ON=0
uint8 RESET_PIN=1
uint8 RESET_SOFTWARE=2
uint8 RESET_INDEPENDENT_WATCHDOG=3
uint8 RESET_WINDOW_WATCHDOG=4
uint8 RESET_BROWNOUT=5
uint8 RESET_LOW_POWER=6
uint8 RESET_UNKNOWN=255

# Error-source values
uint8 SOURCE_NONE=0
uint8 SOURCE_TRANSPORT=1
uint8 SOURCE_MOTORS=2
uint8 SOURCE_PWM_SERVOS=3
uint8 SOURCE_BUS_SERVOS=4
uint8 SOURCE_LEDS=5
uint8 SOURCE_BUZZER=6
uint8 SOURCE_RGB=7
uint8 SOURCE_OLED=8
uint8 SOURCE_IMU=9
uint8 SOURCE_BATTERY=10
uint8 SOURCE_EXECUTOR=11
uint8 SOURCE_MEMORY=12

builtin_interfaces/Time stamp
uint64 transport_rx_bytes
uint64 transport_tx_bytes

uint32 uptime_ms
uint32 session_generation
uint32 agent_reconnects

uint32 command_messages
uint32 command_rejections
uint32[7] mailbox_overwrites
uint32 button_event_drops
uint32 publication_errors

uint32 service_requests
uint32 service_completions
uint32 service_busy_rejections
uint32 service_timeouts
uint32 service_partial_results
uint32 late_response_drops

uint32[4] motor_lease_expiries
uint32[4] motor_command_rejections
uint32 motor_watchdog_trips
uint32 motor_command_consumptions
uint32 motor_command_age_over_20_ms
uint32 motor_command_max_age_us
uint32 executor_overruns

uint32[8] peripheral_errors
uint32[8] peripheral_timeouts
uint32[4] usart1_errors
uint32 usart1_rx_dma_high_water_bytes
uint32 transport_rx_overruns
uint32 transport_tx_timeouts
uint32 maximum_transport_wait_us

uint32[6] task_missed_releases
uint32[6] task_max_execution_us
uint32[6] task_stack_high_water_bytes
uint32[6] task_heartbeat_age_ms
uint32[2] free_ram_bytes
uint32[2] minimum_free_ram_bytes
uint32 flash_used_bytes
uint32 flash_total_bytes
uint32 post_seal_allocation_attempts
uint32 last_error_uptime_ms

uint16 last_error_detail
uint8 session_state
uint8 last_teardown_reason
uint8 last_reset_reason
uint8 last_watchdog_task
uint8 last_error_code
uint8 last_error_source
