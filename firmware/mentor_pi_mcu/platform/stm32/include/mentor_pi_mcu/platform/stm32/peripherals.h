// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_PLATFORM_STM32_PERIPHERALS_H_
#define MENTOR_PI_MCU_PLATFORM_STM32_PERIPHERALS_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/platform/stm32/status.h"

namespace mentor_pi_mcu::platform::stm32 {

inline constexpr std::size_t kMotorCount = 4U;
inline constexpr std::size_t kPwmServoCount = 4U;
inline constexpr std::size_t kLedCount = 3U;
inline constexpr std::size_t kButtonCount = 2U;
inline constexpr std::size_t kRgbDmaCapacityBytes = 128U;
inline constexpr std::size_t kBusUartBufferCapacityBytes = 128U;

// Motor duty is signed permille [-1000, 1000]. Zero disables both bridge PWM
// outputs. A sign change disables both channels before enabling the new side.
// Nonzero duty is rejected until ArmMotorOutput() records fresh ACTIVE-session
// authority. Lease expiry and session teardown must call DisarmMotorOutput().
Status ArmMotorOutput(std::size_t motor_index);
void DisarmMotorOutput(std::size_t motor_index);
Status SetMotorDutyPermille(std::size_t motor_index,
                            std::int16_t duty_permille);
// TIM5/TIM2 are 32-bit; TIM4/TIM3 are 16-bit and zero-extend in this raw
// snapshot. Target glue retains the per-channel width for modular deltas.
std::array<std::uint32_t, kMotorCount> ReadEncoderCounters();

// Idempotent register-only safety primitive. It takes no lock, waits for
// nothing, clears all eight compares, disables every bridge output, and clears
// software arm state. It is safe in task or interrupt context.
void EmergencyStopMotors();
std::uint8_t MotorArmedMask();

// TIM13 uses a 1 MHz time base. The ISR copies the shadow at a common frame
// boundary and only toggles the four GPIO pins/schedules the next edge.
Status SetPwmServoPulseShadow(
    const std::array<std::uint16_t, kPwmServoCount>& pulse_width_us);
Status StartPwmServoFrameGenerator();
void StopPwmServoFrameGenerator();
std::uint32_t PwmServoFrameSequence();

Status SetLed(std::size_t led_index, bool on);
bool ReadButtonPressed(std::size_t button_index);

// TIM12 produces the PA6 square wave. Pattern timing remains owned by
// PeripheralTask; zero stops the timer and drives PA6 low.
Status SetBuzzerFrequency(std::uint16_t frequency_hz);

// Data is copied into a fixed DMA-safe bounce buffer. Completion is reported by
// IsRgbTransferComplete(); no transfer is queued behind an active one.
Status StartRgbTransfer(const std::uint8_t* data, std::size_t length);
Status StartRgbPixels(const std::array<std::uint8_t, 2>& red,
                      const std::array<std::uint8_t, 2>& green,
                      const std::array<std::uint8_t, 2>& blue);
bool IsRgbTransferComplete();
Status RgbTransferStatus();
void CancelRgbTransfer();

// Starts one two-rank scan: VREFINT then PB0/ADC1 channel 8. The result remains
// in a fixed DMA buffer until the next start.
Status StartBatteryAdcConversion();
Status TakeBatteryAdcResult(std::array<std::uint16_t, 2>* samples);
std::uint16_t FactoryVrefintCalibration();

// OLED I2C uses the HAL's shifted 8-bit address form. Timeouts must be
// 1..10 ms.
Status OledI2cTransmit(std::uint16_t device_address, const std::uint8_t* data,
                       std::size_t length, std::uint32_t timeout_ms);
Status ResetOledI2c();

// QMI8658 software-I2C is fixed to PB10/PB11. Transactions are short,
// synchronous, and bounded by timeout_ms (1..10 ms).
Status ImuI2cWrite(std::uint8_t device_address, std::uint8_t register_address,
                   const std::uint8_t* data, std::size_t length,
                   std::uint32_t timeout_ms);
Status ImuI2cRead(std::uint8_t device_address, std::uint8_t register_address,
                  std::uint8_t* data, std::size_t length,
                  std::uint32_t timeout_ms);

// UART5 is interrupt-driven, single-wire, 115200 8N1. The platform owns fixed
// TX/RX buffers. For reads, the TX-complete ISR changes direction and arms the
// exact-length receive immediately so the first reply byte cannot be lost to
// task scheduling latency. response_length == 0 selects transmit-only.
Status StartBusUartExchange(const std::uint8_t* data, std::size_t length,
                            std::size_t response_length);
Status TakeBusUartCompletion(std::uint8_t* destination,
                             std::size_t destination_capacity,
                             std::size_t* received_length);
void ResetBusUart();

// ISR delegates used only by the vector/callback shims.
void HandleMotorReleaseFromIsr();
void HandlePwmServoTimerFromIsr();
void HandleBuzzerTimerFromIsr();
void HandleImuDataReadyFromIsr();
void HandleRgbCompleteFromIsr();
void HandleRgbErrorFromIsr();
void HandleAdcCompleteFromIsr();
void HandleAdcErrorFromIsr();
void HandleBusUartTxCompleteFromIsr();
void HandleBusUartRxCompleteFromIsr();
void HandleBusUartErrorFromIsr();

}  // namespace mentor_pi_mcu::platform::stm32

#endif  // MENTOR_PI_MCU_PLATFORM_STM32_PERIPHERALS_H_
