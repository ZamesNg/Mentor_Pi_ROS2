// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/controller/status_rgb.h"

#include <cstdint>

namespace mentor_pi_mcu::app::controller {

bool MicroRosHeartbeatController::Update(
    std::uint32_t successful_heartbeat_count) {
  const std::uint32_t heartbeat_delta =
      successful_heartbeat_count - previous_heartbeat_count_;
  if ((heartbeat_delta & 1U) != 0U) {
    on_ = !on_;
  }
  previous_heartbeat_count_ = successful_heartbeat_count;
  return on_;
}

bool StatusRgbController::TransportSampleDue(std::uint32_t now_ms) const {
  return !transport_sampled_ ||
         now_ms - last_transport_sample_ms_ >= kTransportSamplePeriodMs;
}

void StatusRgbController::ObserveTransport(std::uint32_t now_ms,
                                           const TransportActivity& activity) {
  if (transport_sampled_ && activity.open) {
    if (activity.rx_wire_bytes != previous_rx_wire_bytes_) {
      rx_pulse_started_ms_ = now_ms;
      rx_pulse_active_ = true;
    }
    if (activity.tx_wire_bytes != previous_tx_wire_bytes_) {
      tx_pulse_started_ms_ = now_ms;
      tx_pulse_active_ = true;
    }
  }
  if (!activity.open) {
    rx_pulse_active_ = false;
    tx_pulse_active_ = false;
  }
  previous_rx_wire_bytes_ = activity.rx_wire_bytes;
  previous_tx_wire_bytes_ = activity.tx_wire_bytes;
  last_transport_sample_ms_ = now_ms;
  transport_sampled_ = true;
}

StatusRgbColor StatusRgbController::Update(std::uint32_t now_ms) {
  if (rx_pulse_active_ &&
      now_ms - rx_pulse_started_ms_ >= kTransportPulseDurationMs) {
    rx_pulse_active_ = false;
  }
  if (tx_pulse_active_ &&
      now_ms - tx_pulse_started_ms_ >= kTransportPulseDurationMs) {
    tx_pulse_active_ = false;
  }

  constexpr std::uint8_t kOff = 0U;
  return {kOff, tx_pulse_active_ ? kBrightness : kOff,
          rx_pulse_active_ ? kBrightness : kOff};
}

}  // namespace mentor_pi_mcu::app::controller
