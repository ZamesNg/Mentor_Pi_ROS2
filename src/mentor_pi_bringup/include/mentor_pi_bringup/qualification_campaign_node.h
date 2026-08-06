// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_NODE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_NODE_H_

#include <atomic>
#include <cstdint>
#include <memory>

#include "mentor_pi_bringup/qualification_campaign_core.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace mentor_pi_bringup {

inline constexpr char kGuardedFixtureAcknowledgement[] =
    "PERIPHERALS_DISCONNECTED_OR_GUARDED";

struct QualificationCampaignOutcome {
  std::atomic<bool> complete{false};
  std::atomic<bool> passed{false};
};

struct QualificationCampaignInstance {
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<QualificationCampaignOutcome> outcome;
};

// Not exposed as ROS parameters. This isolates ROS-node fault tests from
// emulator wall-clock jitter without weakening the production scheduler.
struct QualificationCampaignTestOverrides {
  CampaignProfile profile{};
  std::int64_t minimum_telemetry_gap_ns = 0;
};

QualificationCampaignInstance MakeQualificationCampaignNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

QualificationCampaignInstance MakeQualificationCampaignNodeForTest(
    const rclcpp::NodeOptions& options,
    QualificationCampaignTestOverrides overrides);

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_NODE_H_
