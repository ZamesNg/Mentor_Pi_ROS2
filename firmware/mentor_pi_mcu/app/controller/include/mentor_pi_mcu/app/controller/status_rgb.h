// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_CONTROLLER_STATUS_RGB_H_
#define MENTOR_PI_MCU_APP_CONTROLLER_STATUS_RGB_H_

#include <cstdint>

#include "mentor_pi_mcu/app/controller/platform_hooks.h"

namespace mentor_pi_mcu::app::controller {

struct StatusRgbColor {
  std::uint8_t red{0U};
  std::uint8_t green{0U};
  std::uint8_t blue{0U};
};

// Toggles the RGB1 red channel after each successful micro-ROS heartbeat.
class MicroRosHeartbeatController {
 public:
  bool Update(std::uint32_t successful_heartbeat_count);

 private:
  std::uint32_t previous_heartbeat_count_{0U};
  bool on_{false};
};

// Allocation-free state machine for the firmware-owned first RGB pixel.
// Transport counters are deliberately sampled at 10 Hz rather than from an
// interrupt or on every PeripheralTask release.
class StatusRgbController {
 public:
  static constexpr std::uint8_t kBrightness = 32U;
  static constexpr std::uint32_t kTransportSamplePeriodMs = 100U;
  static constexpr std::uint32_t kTransportPulseDurationMs = 50U;

  bool TransportSampleDue(std::uint32_t now_ms) const;
  void ObserveTransport(std::uint32_t now_ms,
                        const TransportActivity& activity);
  StatusRgbColor Update(std::uint32_t now_ms);

 private:
  std::uint64_t previous_rx_wire_bytes_{0U};
  std::uint64_t previous_tx_wire_bytes_{0U};
  std::uint32_t last_transport_sample_ms_{0U};
  std::uint32_t rx_pulse_started_ms_{0U};
  std::uint32_t tx_pulse_started_ms_{0U};
  bool transport_sampled_{false};
  bool rx_pulse_active_{false};
  bool tx_pulse_active_{false};
};

}  // namespace mentor_pi_mcu::app::controller

#endif  // MENTOR_PI_MCU_APP_CONTROLLER_STATUS_RGB_H_
