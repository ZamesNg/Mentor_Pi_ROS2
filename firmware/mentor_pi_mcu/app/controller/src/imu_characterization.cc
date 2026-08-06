// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/controller/imu_characterization.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

extern "C" {
volatile mentor_pi_mcu::app::controller::ImuCharacterizationSnapshot
    rrclite_imu_characterization_snapshot{};
}

namespace mentor_pi_mcu::app::controller {

static_assert(std::is_standard_layout_v<ImuCharacterizationSnapshot>);
static_assert(std::is_trivially_copyable_v<ImuCharacterizationSnapshot>);
static_assert(sizeof(float) == 4U);

void UpdateImuCharacterizationSnapshot(
    std::uint32_t timestamp_ms, std::uint8_t address, std::uint8_t revision,
    mentor_pi::mcu::Result result,
    const mentor_pi::mcu::drivers::ImuSample* sample) {
  const std::uint32_t writing_sequence =
      (rrclite_imu_characterization_snapshot.sequence + 1U) | 1U;
  rrclite_imu_characterization_snapshot.sequence = writing_sequence;
  std::atomic_signal_fence(std::memory_order_seq_cst);

  rrclite_imu_characterization_snapshot.timestamp_ms = timestamp_ms;
  rrclite_imu_characterization_snapshot.address = address;
  rrclite_imu_characterization_snapshot.revision = revision;
  rrclite_imu_characterization_snapshot.result_code =
      static_cast<std::uint8_t>(result.code);
  rrclite_imu_characterization_snapshot.detail = result.detail;
  const bool valid = result.ok() && sample != nullptr;
  rrclite_imu_characterization_snapshot.valid =
      static_cast<std::uint8_t>(valid);
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    rrclite_imu_characterization_snapshot.acceleration_mps2[axis] =
        valid ? sample->acceleration_mps2[axis] : 0.0F;
    rrclite_imu_characterization_snapshot.angular_velocity_rps[axis] =
        valid ? sample->angular_velocity_rps[axis] : 0.0F;
  }

  std::atomic_signal_fence(std::memory_order_seq_cst);
  rrclite_imu_characterization_snapshot.sequence = writing_sequence + 1U;
}

}  // namespace mentor_pi_mcu::app::controller
