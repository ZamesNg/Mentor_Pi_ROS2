# RRCLite retained-hardware drivers

This directory contains allocation-free C++17 drivers for the retained
RRCLite V1.0 hardware. The drivers intentionally depend on narrow abstract HAL
interfaces rather than STM32 handles. The STM32 platform layer owns pin,
timer, DMA, interrupt, and half-duplex direction configuration.

The native test build is independent of STM32Cube:

```sh
cmake -S firmware/mentor_pi_mcu/drivers -B /tmp/rrclite-driver-build
cmake --build /tmp/rrclite-driver-build
ctest --test-dir /tmp/rrclite-driver-build --output-on-failure
```

Hardware bindings must preserve these assumptions:

- motor indices 0..3 map to M1..M4; encoders 0/1 are 32-bit and 2/3 are
  16-bit;
- PWM-servo pins use a 20 ms frame and 500..2500 us high pulses;
- UART5 is a 115200 8N1 single-wire Hiwonder bus and never blocks a worker;
- RGB uses SPI bytes `0xc0`/`0xf8`, GRB order, and a 24-byte reset suffix;
- QMI8658 uses software I2C and must not publish until its board-axis signed
  permutation has been measured and marked verified;
- SSD1306 uses I2C1 address `0x3c` (the legacy STM32 HAL value `0x78` was the
  shifted address);
- ADC DMA order is VREFINT then PB0/ADC1 channel 8. The baseline divider is
  100 kohm + 10 kohm, or 11:1.
