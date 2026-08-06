// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_CONTROLLER_IMU_CHARACTERIZATION_H_
#define MENTOR_PI_MCU_APP_CONTROLLER_IMU_CHARACTERIZATION_H_

#include <cstdint>

#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/qmi8658.h"

namespace mentor_pi_mcu::app::controller {

// Debugger-only sensor-frame snapshot. The single SensorTask writer makes the
// sequence odd while updating and even when every field is coherent. This is
// deliberately separate from ROS telemetry and grants no actuator authority.
struct ImuCharacterizationSnapshot {
  std::uint32_t sequence{0U};
  std::uint32_t timestamp_ms{0U};
  std::uint16_t detail{0U};
  std::uint8_t address{0U};
  std::uint8_t revision{0U};
  std::uint8_t result_code{0U};
  std::uint8_t valid{0U};
  // Keep plain arrays so the debugger snapshot has a simple C-compatible ABI.
  float acceleration_mps2[3]{};     // NOLINT(modernize-avoid-c-arrays)
  float angular_velocity_rps[3]{};  // NOLINT(modernize-avoid-c-arrays)
};

void UpdateImuCharacterizationSnapshot(
    std::uint32_t timestamp_ms, std::uint8_t address, std::uint8_t revision,
    mentor_pi::mcu::Result result,
    const mentor_pi::mcu::drivers::ImuSample* sample);

}  // namespace mentor_pi_mcu::app::controller

// The C-linkage name is stable and intentionally easy to inspect from GDB.
extern "C" volatile mentor_pi_mcu::app::controller::ImuCharacterizationSnapshot
    rrclite_imu_characterization_snapshot;

#endif  // MENTOR_PI_MCU_APP_CONTROLLER_IMU_CHARACTERIZATION_H_
