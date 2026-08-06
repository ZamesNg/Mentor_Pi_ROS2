# The official micro-ROS static-library builder extracts these target flags.
RRCLITE_MICROROS_CFLAGS := \
  -mcpu=cortex-m4 \
  -mthumb \
  -mfpu=fpv4-sp-d16 \
  -mfloat-abi=hard \
  -Os \
  --specs=nano.specs \
  -DSTM32F407xx \
  -DUSE_HAL_DRIVER \
  -DCLOCK_MONOTONIC=0
