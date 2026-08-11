// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_EVIDENCE_H_
// NOLINTNEXTLINE: Required by the ROS 2 header-guard convention.
#define MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_EVIDENCE_H_

#include <cstdint>
#include <string>

#include "mentor_pi_bringup/qualification_campaign_core.h"

namespace mentor_pi_bringup {

// The identity the caller expects a published evidence bundle to attest.
// Every CampaignProfile field is compared exactly, including rational rates.
struct CampaignEvidenceExpectation {
  CampaignMode mode = CampaignMode::kLoad500;
  CampaignProfile profile{};
  std::int64_t configured_duration_ns = 0;
};

// Verifies the closed set of evidence files, their SHA-256 digests, and their
// exact campaign identity. The verifier does not treat a valid bundle as proof
// of release qualification; it only establishes bundle integrity and identity.
bool VerifyCampaignEvidence(const std::string& directory,
                            const CampaignEvidenceExpectation& expectation,
                            std::string* error);

}  // namespace mentor_pi_bringup

#endif  // MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_EVIDENCE_H_
