// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_SRC_MOTOR_ADRC_SERVICE_ADAPTER_H_
#define MENTOR_PI_MCU_APP_MICROROS_SRC_MOTOR_ADRC_SERVICE_ADAPTER_H_

#include <algorithm>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/domain/validation.h"

namespace mentor_pi_mcu::app::microros {

// Keep the generated C request at the middleware boundary. This template is
// instantiated by services.cc with the checked SDK type and by focused native
// tests with that same generated type, without coupling domain code to ROS.
template <typename WireRequest>
mentor_pi::mcu::Result DecodeMotorAdrcRequest(
    const WireRequest& request, mentor_pi::mcu::SetMotorAdrcCommand* command) {
  if (command == nullptr) {
    return {mentor_pi::mcu::ResultCode::kInvalidArgument, 0U};
  }
  *command = {};
  command->update_mask = request.update_mask;
  std::copy_n(request.known_velocity_decay_rate_s_inverse,
              command->known_velocity_decay_rate_s_inverse.size(),
              command->known_velocity_decay_rate_s_inverse.begin());
  std::copy_n(request.input_gain_rps_per_second_per_permille,
              command->input_gain_rps_per_second_per_permille.size(),
              command->input_gain_rps_per_second_per_permille.begin());
  std::copy_n(request.controller_bandwidth_rad_s,
              command->controller_bandwidth_rad_s.size(),
              command->controller_bandwidth_rad_s.begin());
  std::copy_n(request.controller_fal_exponent,
              command->controller_fal_exponent.size(),
              command->controller_fal_exponent.begin());
  std::copy_n(request.controller_fal_threshold_rps,
              command->controller_fal_threshold_rps.size(),
              command->controller_fal_threshold_rps.begin());
  std::copy_n(request.observer_bandwidth_rad_s,
              command->observer_bandwidth_rad_s.size(),
              command->observer_bandwidth_rad_s.begin());
  std::copy_n(request.observer_velocity_fal_exponent,
              command->observer_velocity_fal_exponent.size(),
              command->observer_velocity_fal_exponent.begin());
  std::copy_n(request.observer_disturbance_fal_exponent,
              command->observer_disturbance_fal_exponent.size(),
              command->observer_disturbance_fal_exponent.begin());
  std::copy_n(request.observer_fal_threshold_rps,
              command->observer_fal_threshold_rps.size(),
              command->observer_fal_threshold_rps.begin());
  std::copy_n(request.disturbance_leakage_s_inverse,
              command->disturbance_leakage_s_inverse.size(),
              command->disturbance_leakage_s_inverse.begin());
  std::copy_n(request.disturbance_estimate_limit_rps_per_second,
              command->disturbance_estimate_limit_rps_per_second.size(),
              command->disturbance_estimate_limit_rps_per_second.begin());
  std::copy_n(request.velocity_filter_new_weight,
              command->velocity_filter_new_weight.size(),
              command->velocity_filter_new_weight.begin());
  std::copy_n(request.positive_minimum_drive_permille,
              command->positive_minimum_drive_permille.size(),
              command->positive_minimum_drive_permille.begin());
  std::copy_n(request.negative_minimum_drive_permille,
              command->negative_minimum_drive_permille.size(),
              command->negative_minimum_drive_permille.begin());
  return mentor_pi::mcu::ValidateSetMotorAdrcCommand(*command);
}

template <typename WireResponse>
void EncodeMotorAdrcResponse(mentor_pi::mcu::Result result,
                             std::uint8_t applied_mask,
                             WireResponse* response) {
  if (response == nullptr) {
    return;
  }
  *response = {};
  response->result.code = static_cast<std::uint8_t>(result.code);
  response->result.detail = result.detail;
  response->applied_mask = result.ok() ? applied_mask : 0U;
}

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_SRC_MOTOR_ADRC_SERVICE_ADAPTER_H_
