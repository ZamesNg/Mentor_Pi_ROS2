// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#ifndef MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_EVIDENCE_INTERNAL_H_
#define MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_EVIDENCE_INTERNAL_H_

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>

#include "mentor_pi_bringup/qualification_campaign_core.h"

namespace mentor_pi_bringup::campaign_evidence_internal {

std::string Sha256Hex(std::string_view contents);
bool Sha256File(const std::filesystem::path& path, std::string* digest,
                std::uint64_t* size, std::string* error);
bool AtomicRenameDirectoryNoReplace(const std::filesystem::path& source,
                                    const std::filesystem::path& destination,
                                    std::string* error);
std::string CanonicalCampaignIdentity(CampaignMode mode,
                                      const CampaignProfile& profile,
                                      std::int64_t configured_duration_ns);
std::string CanonicalSummaryIdentityJson(CampaignMode mode,
                                         const CampaignProfile& profile,
                                         std::int64_t configured_duration_ns);
bool IsValidCampaignIdentity(CampaignMode mode, const CampaignProfile& profile,
                             std::int64_t configured_duration_ns,
                             std::string* error);

}  // namespace mentor_pi_bringup::campaign_evidence_internal

#endif  // MENTOR_PI_BRINGUP__QUALIFICATION_CAMPAIGN_EVIDENCE_INTERNAL_H_
