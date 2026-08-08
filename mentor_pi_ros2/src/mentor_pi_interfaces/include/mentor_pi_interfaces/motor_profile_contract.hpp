// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_INTERFACES__MOTOR_PROFILE_CONTRACT_HPP_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_INTERFACES__MOTOR_PROFILE_CONTRACT_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi_interfaces {

struct MotorProfileContract {
  std::uint8_t model;
  std::uint32_t ticks_per_revolution;
  float max_rps;
};

// These values are part of the SetMotorModel response contract. Firmware and
// the host supervisor both consume this table so an acknowledged profile
// cannot silently drift between the two runtime endpoints.
inline constexpr std::array<MotorProfileContract, 4> kMotorProfileContracts{{
    {0U, 3960U, 1.5F},
    {1U, 1980U, 3.0F},
    {2U, 1040U, 6.0F},
    {3U, 5764U, 1.1F},
}};

constexpr const MotorProfileContract* FindMotorProfileContract(
    std::uint8_t model) {
  for (const MotorProfileContract& profile : kMotorProfileContracts) {
    if (profile.model == model) {
      return &profile;
    }
  }
  return nullptr;
}

}  // namespace mentor_pi_interfaces

#endif  // MENTOR_PI_INTERFACES__MOTOR_PROFILE_CONTRACT_HPP_
