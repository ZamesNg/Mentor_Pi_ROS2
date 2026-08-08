// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_ENDPOINT_CONTRACT_H_
#define MENTOR_PI_MCU_APP_MICROROS_ENDPOINT_CONTRACT_H_

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi_mcu::app::microros {

enum class Reliability : std::uint8_t {
  kBestEffort,
  kReliable,
};

struct TopicEndpoint {
  const char* relative_name;
  Reliability reliability;
  std::size_t depth;
};

struct ServiceEndpoint {
  const char* relative_name;
};

inline constexpr std::array<TopicEndpoint, 7> kPublisherEndpoints{{
    {"motors/state", Reliability::kBestEffort, 1U},
    {"pwm_servos/state", Reliability::kBestEffort, 1U},
    {"imu", Reliability::kBestEffort, 1U},
    {"buttons/events", Reliability::kReliable, 8U},
    {"battery/state", Reliability::kReliable, 1U},
    {"heartbeat", Reliability::kReliable, 1U},
    {"diagnostics", Reliability::kReliable, 1U},
}};

inline constexpr std::array<TopicEndpoint, 7> kSubscriptionEndpoints{{
    {"motors/command", Reliability::kBestEffort, 1U},
    {"pwm_servos/command", Reliability::kBestEffort, 1U},
    {"bus_servos/command", Reliability::kBestEffort, 1U},
    {"leds/command", Reliability::kReliable, 1U},
    {"buzzer/command", Reliability::kReliable, 1U},
    {"rgb/command", Reliability::kReliable, 1U},
    {"oled/command", Reliability::kReliable, 1U},
}};

inline constexpr std::array<ServiceEndpoint, 7> kServiceEndpoints{{
    {"motors/set_model"},
    {"motors/set_pid"},
    {"pwm_servos/set_offsets"},
    {"bus_servos/get_state"},
    {"bus_servos/configure"},
    {"bus_servos/stop"},
    {"battery/set_low_threshold"},
}};

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_ENDPOINT_CONTRACT_H_
