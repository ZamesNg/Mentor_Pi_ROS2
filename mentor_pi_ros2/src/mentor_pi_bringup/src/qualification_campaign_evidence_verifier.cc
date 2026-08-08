// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "mentor_pi_bringup/qualification_campaign_evidence.h"
#include "mentor_pi_bringup/qualification_campaign_evidence_internal.h"

namespace mentor_pi_bringup {
namespace campaign_evidence_internal {
namespace {

constexpr std::array<std::uint32_t, 64U> kRoundConstants{
    UINT32_C(0x428a2f98), UINT32_C(0x71374491), UINT32_C(0xb5c0fbcf),
    UINT32_C(0xe9b5dba5), UINT32_C(0x3956c25b), UINT32_C(0x59f111f1),
    UINT32_C(0x923f82a4), UINT32_C(0xab1c5ed5), UINT32_C(0xd807aa98),
    UINT32_C(0x12835b01), UINT32_C(0x243185be), UINT32_C(0x550c7dc3),
    UINT32_C(0x72be5d74), UINT32_C(0x80deb1fe), UINT32_C(0x9bdc06a7),
    UINT32_C(0xc19bf174), UINT32_C(0xe49b69c1), UINT32_C(0xefbe4786),
    UINT32_C(0x0fc19dc6), UINT32_C(0x240ca1cc), UINT32_C(0x2de92c6f),
    UINT32_C(0x4a7484aa), UINT32_C(0x5cb0a9dc), UINT32_C(0x76f988da),
    UINT32_C(0x983e5152), UINT32_C(0xa831c66d), UINT32_C(0xb00327c8),
    UINT32_C(0xbf597fc7), UINT32_C(0xc6e00bf3), UINT32_C(0xd5a79147),
    UINT32_C(0x06ca6351), UINT32_C(0x14292967), UINT32_C(0x27b70a85),
    UINT32_C(0x2e1b2138), UINT32_C(0x4d2c6dfc), UINT32_C(0x53380d13),
    UINT32_C(0x650a7354), UINT32_C(0x766a0abb), UINT32_C(0x81c2c92e),
    UINT32_C(0x92722c85), UINT32_C(0xa2bfe8a1), UINT32_C(0xa81a664b),
    UINT32_C(0xc24b8b70), UINT32_C(0xc76c51a3), UINT32_C(0xd192e819),
    UINT32_C(0xd6990624), UINT32_C(0xf40e3585), UINT32_C(0x106aa070),
    UINT32_C(0x19a4c116), UINT32_C(0x1e376c08), UINT32_C(0x2748774c),
    UINT32_C(0x34b0bcb5), UINT32_C(0x391c0cb3), UINT32_C(0x4ed8aa4a),
    UINT32_C(0x5b9cca4f), UINT32_C(0x682e6ff3), UINT32_C(0x748f82ee),
    UINT32_C(0x78a5636f), UINT32_C(0x84c87814), UINT32_C(0x8cc70208),
    UINT32_C(0x90befffa), UINT32_C(0xa4506ceb), UINT32_C(0xbef9a3f7),
    UINT32_C(0xc67178f2)};

constexpr std::array<const char*, kCampaignCommandCount> kCommandNames{
    "motor", "pwm_servo", "bus_servo", "led", "buzzer", "rgb", "oled"};

std::uint32_t RotateRight(std::uint32_t value, unsigned int shift) {
  return (value >> shift) | (value << (32U - shift));
}

class Sha256 final {
 public:
  void Update(const std::uint8_t* data, std::size_t size) {
    total_bytes_ += static_cast<std::uint64_t>(size);
    std::size_t offset = 0U;
    while (offset < size) {
      const std::size_t available = block_.size() - block_size_;
      const std::size_t count = std::min(available, size - offset);
      std::copy_n(data + offset, count,
                  block_.begin() + static_cast<std::ptrdiff_t>(block_size_));
      block_size_ += count;
      offset += count;
      if (block_size_ == block_.size()) {
        Transform(block_.data());
        block_size_ = 0U;
      }
    }
  }

  std::array<std::uint8_t, 32U> Finish() {
    const std::uint64_t bit_count = total_bytes_ * UINT64_C(8);
    block_[block_size_++] = UINT8_C(0x80);
    if (block_size_ > 56U) {
      std::fill(block_.begin() + static_cast<std::ptrdiff_t>(block_size_),
                block_.end(), UINT8_C(0));
      Transform(block_.data());
      block_size_ = 0U;
    }
    std::fill(block_.begin() + static_cast<std::ptrdiff_t>(block_size_),
              block_.begin() + 56, UINT8_C(0));
    for (std::size_t index = 0U; index < 8U; ++index) {
      const unsigned int shift = static_cast<unsigned int>((7U - index) * 8U);
      block_[56U + index] =
          static_cast<std::uint8_t>((bit_count >> shift) & UINT64_C(0xff));
    }
    Transform(block_.data());

    std::array<std::uint8_t, 32U> digest{};
    for (std::size_t word = 0U; word < state_.size(); ++word) {
      for (std::size_t byte = 0U; byte < 4U; ++byte) {
        const unsigned int shift = static_cast<unsigned int>((3U - byte) * 8U);
        digest[(word * 4U) + byte] =
            static_cast<std::uint8_t>((state_[word] >> shift) & UINT32_C(0xff));
      }
    }
    return digest;
  }

 private:
  void Transform(const std::uint8_t* block) {
    std::array<std::uint32_t, 64U> schedule{};
    for (std::size_t index = 0U; index < 16U; ++index) {
      const std::size_t offset = index * 4U;
      schedule[index] =
          (static_cast<std::uint32_t>(block[offset]) << 24U) |
          (static_cast<std::uint32_t>(block[offset + 1U]) << 16U) |
          (static_cast<std::uint32_t>(block[offset + 2U]) << 8U) |
          static_cast<std::uint32_t>(block[offset + 3U]);
    }
    for (std::size_t index = 16U; index < schedule.size(); ++index) {
      const std::uint32_t first = schedule[index - 15U];
      const std::uint32_t second = schedule[index - 2U];
      const std::uint32_t sigma_zero =
          RotateRight(first, 7U) ^ RotateRight(first, 18U) ^ (first >> 3U);
      const std::uint32_t sigma_one =
          RotateRight(second, 17U) ^ RotateRight(second, 19U) ^ (second >> 10U);
      schedule[index] =
          schedule[index - 16U] + sigma_zero + schedule[index - 7U] + sigma_one;
    }

    std::uint32_t a = state_[0U];
    std::uint32_t b = state_[1U];
    std::uint32_t c = state_[2U];
    std::uint32_t d = state_[3U];
    std::uint32_t e = state_[4U];
    std::uint32_t f = state_[5U];
    std::uint32_t g = state_[6U];
    std::uint32_t h = state_[7U];
    for (std::size_t index = 0U; index < schedule.size(); ++index) {
      const std::uint32_t sum_one =
          RotateRight(e, 6U) ^ RotateRight(e, 11U) ^ RotateRight(e, 25U);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t temporary_one =
          h + sum_one + choice + kRoundConstants[index] + schedule[index];
      const std::uint32_t sum_zero =
          RotateRight(a, 2U) ^ RotateRight(a, 13U) ^ RotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary_two = sum_zero + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary_one;
      d = c;
      c = b;
      b = a;
      a = temporary_one + temporary_two;
    }
    state_[0U] += a;
    state_[1U] += b;
    state_[2U] += c;
    state_[3U] += d;
    state_[4U] += e;
    state_[5U] += f;
    state_[6U] += g;
    state_[7U] += h;
  }

  std::array<std::uint32_t, 8U> state_{
      UINT32_C(0x6a09e667), UINT32_C(0xbb67ae85), UINT32_C(0x3c6ef372),
      UINT32_C(0xa54ff53a), UINT32_C(0x510e527f), UINT32_C(0x9b05688c),
      UINT32_C(0x1f83d9ab), UINT32_C(0x5be0cd19)};
  std::array<std::uint8_t, 64U> block_{};
  std::size_t block_size_ = 0U;
  std::uint64_t total_bytes_ = 0U;
};

std::string DigestToHex(const std::array<std::uint8_t, 32U>& digest) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const std::uint8_t byte : digest) {
    output << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return output.str();
}

bool IsKnownMode(CampaignMode mode) {
  return std::string_view(CampaignModeName(mode)) != "unknown";
}

}  // namespace

std::string Sha256Hex(std::string_view contents) {
  Sha256 digest;
  digest.Update(reinterpret_cast<const std::uint8_t*>(contents.data()),
                contents.size());
  return DigestToHex(digest.Finish());
}

bool Sha256File(const std::filesystem::path& path, std::string* digest,
                std::uint64_t* size, std::string* error) {
  if (digest == nullptr || size == nullptr || error == nullptr) {
    return false;
  }
  std::error_code filesystem_error;
  const std::filesystem::file_status file_status =
      std::filesystem::symlink_status(path, filesystem_error);
  if (filesystem_error || std::filesystem::is_symlink(file_status) ||
      !std::filesystem::is_regular_file(file_status)) {
    *error = "cannot hash non-regular file: " + path.string();
    return false;
  }
  const std::uintmax_t file_size =
      std::filesystem::file_size(path, filesystem_error);
  if (filesystem_error ||
      file_size > static_cast<std::uintmax_t>(
                      std::numeric_limits<std::uint64_t>::max())) {
    *error = "cannot determine evidence file size: " + path.string();
    return false;
  }
  std::ifstream input(path, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    *error = "cannot open evidence file for hashing: " + path.string();
    return false;
  }
  Sha256 hasher;
  std::array<char, 16384U> buffer{};
  while (input.good()) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const std::streamsize count = input.gcount();
    if (count > 0) {
      hasher.Update(reinterpret_cast<const std::uint8_t*>(buffer.data()),
                    static_cast<std::size_t>(count));
    }
  }
  if (!input.eof()) {
    *error = "cannot finish hashing evidence file: " + path.string();
    return false;
  }
  *digest = DigestToHex(hasher.Finish());
  *size = static_cast<std::uint64_t>(file_size);
  return true;
}

std::string CanonicalCampaignIdentity(CampaignMode mode,
                                      const CampaignProfile& profile,
                                      std::int64_t configured_duration_ns) {
  std::ostringstream output;
  output << "campaign_mode=" << CampaignModeName(mode) << '\n'
         << "configured_duration_ns=" << configured_duration_ns << '\n'
         << "profile_mode=" << CampaignModeName(profile.mode) << '\n'
         << "profile_canonical_duration_ns=" << profile.canonical_duration_ns
         << '\n'
         << "profile_service_round_period_ns="
         << profile.service_round_period_ns << '\n'
         << "profile_expected_cycles=" << profile.expected_cycles << '\n'
         << "profile_continuous_session_required="
         << (profile.continuous_session_required ? 1 : 0) << '\n';
  for (std::size_t index = 0U; index < profile.command_rates.size(); ++index) {
    output << "profile_command_rate=" << kCommandNames[index] << ','
           << profile.command_rates[index].numerator << ','
           << profile.command_rates[index].denominator_seconds << '\n';
  }
  return output.str();
}

std::string CanonicalSummaryIdentityJson(CampaignMode mode,
                                         const CampaignProfile& profile,
                                         std::int64_t configured_duration_ns) {
  std::ostringstream output;
  const std::string identity =
      CanonicalCampaignIdentity(mode, profile, configured_duration_ns);
  output << "  \"campaign_identity\": {\n"
         << R"(    "mode": ")" << CampaignModeName(mode) << "\",\n"
         << "    \"configured_duration_ns\": " << configured_duration_ns
         << ",\n"
         << R"(    "profile_mode": ")" << CampaignModeName(profile.mode)
         << "\",\n"
         << "    \"profile_canonical_duration_ns\": "
         << profile.canonical_duration_ns << ",\n"
         << "    \"profile_service_round_period_ns\": "
         << profile.service_round_period_ns << ",\n"
         << "    \"profile_expected_cycles\": " << profile.expected_cycles
         << ",\n"
         << "    \"profile_continuous_session_required\": "
         << (profile.continuous_session_required ? "true" : "false")
         << ",\n    \"profile_command_rates\": [\n";
  for (std::size_t index = 0U; index < profile.command_rates.size(); ++index) {
    output << R"(      {"name": ")" << kCommandNames[index]
           << R"(", "numerator": )" << profile.command_rates[index].numerator
           << ", \"denominator_seconds\": "
           << profile.command_rates[index].denominator_seconds << "}"
           << (index + 1U == profile.command_rates.size() ? "\n" : ",\n");
  }
  output << "    ],\n    \"identity_sha256\": \"" << Sha256Hex(identity)
         << "\"\n  },\n";
  return output.str();
}

bool IsValidCampaignIdentity(CampaignMode mode, const CampaignProfile& profile,
                             std::int64_t configured_duration_ns,
                             std::string* error) {
  if (!IsKnownMode(mode) || !IsKnownMode(profile.mode) ||
      mode != profile.mode || configured_duration_ns < 0 ||
      profile.canonical_duration_ns < 0 ||
      profile.service_round_period_ns <= 0) {
    if (error != nullptr) {
      *error = "campaign mode/profile/duration identity is invalid";
    }
    return false;
  }
  for (const CampaignProfile::Rate& rate : profile.command_rates) {
    if (rate.denominator_seconds == 0U) {
      if (error != nullptr) {
        *error = "campaign profile has a zero rate denominator";
      }
      return false;
    }
  }
  if (profile.continuous_session_required && configured_duration_ns == 0) {
    if (error != nullptr) {
      *error = "continuous campaign duration must be positive";
    }
    return false;
  }
  return true;
}

}  // namespace campaign_evidence_internal
namespace {

using campaign_evidence_internal::CanonicalCampaignIdentity;
using campaign_evidence_internal::IsValidCampaignIdentity;
using campaign_evidence_internal::Sha256File;
using campaign_evidence_internal::Sha256Hex;

constexpr std::array<const char*, kCampaignCommandCount> kCommandNames{
    "motor", "pwm_servo", "bus_servo", "led", "buzzer", "rgb", "oled"};
constexpr std::array<const char*, 4U> kPayloadNames{
    "summary.json", "metrics.csv", "session-transitions.csv", "junit.xml"};
constexpr std::string_view kManifestName = "manifest.sha256";
constexpr std::string_view kManifestHeader =
    "RRCLITE-QUALIFICATION-EVIDENCE-MANIFEST 1";
constexpr std::uintmax_t kMaximumManifestBytes = 65536U;
constexpr std::uintmax_t kMaximumPayloadBytes =
    UINTMAX_C(16) * UINTMAX_C(1024) * UINTMAX_C(1024);
constexpr std::size_t kMaximumJsonDepth = 32U;
constexpr std::size_t kMaximumJsonNodes = 100000U;
constexpr std::size_t kMaximumJsonStringBytes = 1048576U;
constexpr std::size_t kMaximumJsonNumberBytes = 128U;

struct PayloadEntry {
  std::uint64_t size = 0U;
  std::string digest;
};

struct ParsedManifest {
  CampaignMode mode = CampaignMode::kLoad500;
  CampaignProfile profile{};
  std::int64_t configured_duration_ns = 0;
  std::string identity_digest;
  std::map<std::string, PayloadEntry> payloads;
};

bool IsReadOnly(std::filesystem::perms permissions) {
  constexpr std::filesystem::perms kWritePermissions =
      std::filesystem::perms::owner_write |
      std::filesystem::perms::group_write |
      std::filesystem::perms::others_write;
  return (permissions & kWritePermissions) == std::filesystem::perms::none;
}

bool IsCanonicalChildName(const std::string& name) {
  if (name.empty() || name == "." || name == ".." ||
      name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos) {
    return false;
  }
  const std::filesystem::path path(name);
  return !path.is_absolute() && !path.has_root_name() &&
         !path.has_root_directory() && path.parent_path().empty() &&
         path.filename() == path && path.lexically_normal() == path;
}

bool IsLowerSha256(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char character) {
    return (character >= '0' && character <= '9') ||
           (character >= 'a' && character <= 'f');
  });
}

template <typename Integer>
bool ParseInteger(const std::string& value, Integer* result) {
  if (result == nullptr || value.empty() || value.front() == '+') {
    return false;
  }
  const std::size_t first_digit = value.front() == '-' ? 1U : 0U;
  if (first_digit == value.size() ||
      (value[first_digit] == '0' && value.size() - first_digit > 1U) ||
      value == "-0") {
    return false;
  }
  Integer parsed{};
  const char* const begin = value.data();
  const char* const end = begin + value.size();
  const auto conversion = std::from_chars(begin, end, parsed, 10);
  if (conversion.ec != std::errc{} || conversion.ptr != end) {
    return false;
  }
  *result = parsed;
  return true;
}

bool ReadBoundedFile(const std::filesystem::path& path,
                     std::uintmax_t maximum_size, std::string* contents,
                     std::string* error) {
  std::error_code filesystem_error;
  const std::uintmax_t size =
      std::filesystem::file_size(path, filesystem_error);
  if (filesystem_error || size > maximum_size) {
    *error =
        "evidence file is absent or exceeds its size bound: " + path.string();
    return false;
  }
  std::ifstream input(path, std::ios::in | std::ios::binary);
  if (!input.is_open()) {
    *error = "cannot open evidence file: " + path.string();
    return false;
  }
  contents->assign(std::istreambuf_iterator<char>(input),
                   std::istreambuf_iterator<char>());
  if (!input.eof() && input.fail()) {
    *error = "cannot read evidence file: " + path.string();
    return false;
  }
  if (contents->size() != static_cast<std::size_t>(size)) {
    *error = "evidence file changed while it was read: " + path.string();
    return false;
  }
  return true;
}

bool ReadTokenLine(std::istringstream* input, std::vector<std::string>* tokens,
                   std::size_t line_number, std::string* error) {
  std::string line;
  if (!std::getline(*input, line)) {
    return false;
  }
  if (line.empty() || line.size() > 1024U || line.front() == ' ' ||
      line.back() == ' ' || line.back() == '\r' ||
      line.find('\t') != std::string::npos ||
      line.find("  ") != std::string::npos) {
    *error = "malformed manifest line " + std::to_string(line_number);
    return false;
  }
  tokens->clear();
  std::istringstream line_input(line);
  std::string token;
  while (line_input >> token) {
    tokens->push_back(token);
  }
  if (tokens->empty()) {
    *error = "malformed manifest line " + std::to_string(line_number);
    return false;
  }
  return true;
}

bool ParseManifest(const std::string& contents, ParsedManifest* manifest,
                   std::string* error) {
  if (contents.empty() || contents.back() != '\n') {
    *error = "manifest must end with one canonical line terminator";
    return false;
  }
  std::istringstream input(contents);
  std::string header;
  if (!std::getline(input, header) || header != kManifestHeader) {
    *error = "manifest header or schema version is invalid";
    return false;
  }

  std::optional<CampaignMode> mode;
  std::optional<std::int64_t> configured_duration_ns;
  std::optional<CampaignMode> profile_mode;
  std::optional<std::int64_t> canonical_duration_ns;
  std::optional<std::int64_t> service_round_period_ns;
  std::optional<std::uint32_t> expected_cycles;
  std::optional<bool> continuous_session_required;
  std::array<std::optional<CampaignProfile::Rate>, kCampaignCommandCount>
      rates{};
  std::optional<std::string> identity_digest;
  std::map<std::string, PayloadEntry> payloads;
  std::size_t line_number = 1U;
  std::vector<std::string> tokens;
  while (input.peek() != std::char_traits<char>::eof()) {
    ++line_number;
    if (!ReadTokenLine(&input, &tokens, line_number, error)) {
      return false;
    }
    const std::string& key = tokens[0U];
    if (key == "campaign_mode") {
      if (tokens.size() != 2U || mode.has_value()) {
        *error = "malformed or duplicate campaign_mode";
        return false;
      }
      mode = ParseCampaignMode(tokens[1U]);
      if (!mode.has_value()) {
        *error = "manifest campaign_mode is unknown";
        return false;
      }
    } else if (key == "configured_duration_ns") {
      std::int64_t value = 0;
      if (tokens.size() != 2U || configured_duration_ns.has_value() ||
          !ParseInteger(tokens[1U], &value)) {
        *error = "malformed or duplicate configured_duration_ns";
        return false;
      }
      configured_duration_ns = value;
    } else if (key == "profile_mode") {
      if (tokens.size() != 2U || profile_mode.has_value()) {
        *error = "malformed or duplicate profile_mode";
        return false;
      }
      profile_mode = ParseCampaignMode(tokens[1U]);
      if (!profile_mode.has_value()) {
        *error = "manifest profile_mode is unknown";
        return false;
      }
    } else if (key == "profile_canonical_duration_ns") {
      std::int64_t value = 0;
      if (tokens.size() != 2U || canonical_duration_ns.has_value() ||
          !ParseInteger(tokens[1U], &value)) {
        *error = "malformed or duplicate profile_canonical_duration_ns";
        return false;
      }
      canonical_duration_ns = value;
    } else if (key == "profile_service_round_period_ns") {
      std::int64_t value = 0;
      if (tokens.size() != 2U || service_round_period_ns.has_value() ||
          !ParseInteger(tokens[1U], &value)) {
        *error = "malformed or duplicate profile_service_round_period_ns";
        return false;
      }
      service_round_period_ns = value;
    } else if (key == "profile_expected_cycles") {
      std::uint32_t value = 0U;
      if (tokens.size() != 2U || expected_cycles.has_value() ||
          !ParseInteger(tokens[1U], &value)) {
        *error = "malformed or duplicate profile_expected_cycles";
        return false;
      }
      expected_cycles = value;
    } else if (key == "profile_continuous_session_required") {
      if (tokens.size() != 2U || continuous_session_required.has_value() ||
          (tokens[1U] != "0" && tokens[1U] != "1")) {
        *error = "malformed or duplicate profile_continuous_session_required";
        return false;
      }
      continuous_session_required = tokens[1U] == "1";
    } else if (key == "profile_command_rate") {
      if (tokens.size() != 4U || !IsCanonicalChildName(tokens[1U])) {
        *error = "malformed profile_command_rate";
        return false;
      }
      const auto* const name =
          std::find(kCommandNames.begin(), kCommandNames.end(), tokens[1U]);
      if (name == kCommandNames.end()) {
        *error = "unknown profile command name";
        return false;
      }
      const std::size_t index =
          static_cast<std::size_t>(std::distance(kCommandNames.begin(), name));
      CampaignProfile::Rate rate;
      if (rates[index].has_value() ||
          !ParseInteger(tokens[2U], &rate.numerator) ||
          !ParseInteger(tokens[3U], &rate.denominator_seconds)) {
        *error = "malformed or duplicate profile command rate";
        return false;
      }
      rates[index] = rate;
    } else if (key == "identity_sha256") {
      if (tokens.size() != 2U || identity_digest.has_value() ||
          !IsLowerSha256(tokens[1U])) {
        *error = "malformed or duplicate identity_sha256";
        return false;
      }
      identity_digest = tokens[1U];
    } else if (key == "payload") {
      if (tokens.size() != 4U || !IsCanonicalChildName(tokens[1U])) {
        *error = "malformed or non-canonical payload path";
        return false;
      }
      if (std::find(kPayloadNames.begin(), kPayloadNames.end(), tokens[1U]) ==
          kPayloadNames.end()) {
        *error = "unknown payload path: " + tokens[1U];
        return false;
      }
      PayloadEntry entry;
      if (!ParseInteger(tokens[2U], &entry.size) ||
          !IsLowerSha256(tokens[3U]) ||
          !payloads.emplace(tokens[1U], PayloadEntry{entry.size, tokens[3U]})
               .second) {
        *error = "malformed or duplicate payload entry";
        return false;
      }
    } else {
      *error = "unknown manifest field: " + key;
      return false;
    }
  }

  if (!mode.has_value() || !configured_duration_ns.has_value() ||
      !profile_mode.has_value() || !canonical_duration_ns.has_value() ||
      !service_round_period_ns.has_value() || !expected_cycles.has_value() ||
      !continuous_session_required.has_value() ||
      !identity_digest.has_value() || payloads.size() != kPayloadNames.size()) {
    *error = "manifest is missing required identity or payload fields";
    return false;
  }
  CampaignProfile profile;
  profile.mode = *profile_mode;
  profile.canonical_duration_ns = *canonical_duration_ns;
  profile.service_round_period_ns = *service_round_period_ns;
  profile.expected_cycles = *expected_cycles;
  profile.continuous_session_required = *continuous_session_required;
  for (std::size_t index = 0U; index < rates.size(); ++index) {
    if (!rates[index].has_value()) {
      *error = "manifest is missing a profile command rate";
      return false;
    }
    profile.command_rates[index] =
        rates[index].value_or(CampaignProfile::Rate{});
  }
  if (!IsValidCampaignIdentity(*mode, profile, *configured_duration_ns,
                               error)) {
    return false;
  }
  manifest->mode = *mode;
  manifest->profile = profile;
  manifest->configured_duration_ns = *configured_duration_ns;
  manifest->identity_digest = *identity_digest;
  manifest->payloads = std::move(payloads);
  return true;
}

bool ProfilesEqual(const CampaignProfile& left, const CampaignProfile& right) {
  if (left.mode != right.mode ||
      left.canonical_duration_ns != right.canonical_duration_ns ||
      left.service_round_period_ns != right.service_round_period_ns ||
      left.expected_cycles != right.expected_cycles ||
      left.continuous_session_required != right.continuous_session_required) {
    return false;
  }
  for (std::size_t index = 0U; index < left.command_rates.size(); ++index) {
    if (left.command_rates[index].numerator !=
            right.command_rates[index].numerator ||
        left.command_rates[index].denominator_seconds !=
            right.command_rates[index].denominator_seconds) {
      return false;
    }
  }
  return true;
}

class SummaryJsonParser final {
 public:
  SummaryJsonParser(std::string_view input,
                    const CampaignEvidenceExpectation& expectation,
                    std::string* error)
      : input_(input), expectation_(expectation), error_(error) {}

  bool Parse() {
    SkipWhitespace();
    if (!AccountNode(0U) || !Consume('{', "summary root must be an object")) {
      return false;
    }

    std::set<std::string> fields;
    SkipWhitespace();
    if (ConsumeIf('}')) {
      return Fail("summary root object is empty");
    }
    while (true) {
      SkipWhitespace();
      std::string field;
      if (!ParseString(&field) || !fields.insert(field).second) {
        return Fail("summary root contains a malformed or duplicate key");
      }
      SkipWhitespace();
      if (!Consume(':', "summary root key is missing ':'")) {
        return false;
      }
      if (field == "schema_version") {
        std::uint32_t version = 0U;
        if (!ParseIntegerValue(&version, 1U) || version != 1U) {
          return Fail("summary schema_version must be integer 1");
        }
      } else if (field == "campaign_identity") {
        if (!ParseCampaignIdentity(1U)) {
          return false;
        }
      } else if (field == "metadata" || field == "result" ||
                 field == "session" || field == "motor_age" ||
                 field == "transport" || field == "commands" ||
                 field == "services" || field == "telemetry") {
        if (!ParseValueOfType('{', 1U, "summary field must be an object")) {
          return false;
        }
      } else if (field == "failure_names" ||
                 field == "unobservable_release_metrics") {
        if (!ParseValueOfType('[', 1U, "summary field must be an array")) {
          return false;
        }
      } else {
        return Fail("summary root contains an unknown field");
      }

      SkipWhitespace();
      if (ConsumeIf('}')) {
        break;
      }
      if (!Consume(',', "summary root members are not comma-separated")) {
        return false;
      }
    }
    SkipWhitespace();
    if (position_ != input_.size()) {
      return Fail("summary has trailing non-whitespace content");
    }

    const std::set<std::string> expected_fields{
        "schema_version", "metadata",      "campaign_identity",
        "result",         "failure_names", "session",
        "motor_age",      "transport",     "commands",
        "services",       "telemetry",     "unobservable_release_metrics"};
    if (fields != expected_fields) {
      return Fail("summary root is missing a required field");
    }
    return true;
  }

 private:
  bool Fail(const std::string& message) {
    if (error_ != nullptr && error_->empty()) {
      *error_ = message + " at byte " + std::to_string(position_);
    }
    return false;
  }

  bool AccountNode(std::size_t depth) {
    if (depth > kMaximumJsonDepth) {
      return Fail("summary JSON nesting exceeds its bound");
    }
    if (node_count_ >= kMaximumJsonNodes) {
      return Fail("summary JSON node count exceeds its bound");
    }
    ++node_count_;
    return true;
  }

  void SkipWhitespace() {
    while (position_ < input_.size()) {
      const char character = input_[position_];
      if (character != ' ' && character != '\t' && character != '\n' &&
          character != '\r') {
        break;
      }
      ++position_;
    }
  }

  bool ConsumeIf(char expected) {
    if (position_ < input_.size() && input_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  bool Consume(char expected, const char* message) {
    if (!ConsumeIf(expected)) {
      return Fail(message);
    }
    return true;
  }

  static int HexDigit(char character) {
    if (character >= '0' && character <= '9') {
      return character - '0';
    }
    if (character >= 'a' && character <= 'f') {
      return character - 'a' + 10;
    }
    if (character >= 'A' && character <= 'F') {
      return character - 'A' + 10;
    }
    return -1;
  }

  bool ParseHexCodeUnit(std::uint32_t* code_unit) {
    if (position_ + 4U > input_.size()) {
      return Fail("summary JSON has a truncated Unicode escape");
    }
    std::uint32_t value = 0U;
    for (std::size_t index = 0U; index < 4U; ++index) {
      const int digit = HexDigit(input_[position_ + index]);
      if (digit < 0) {
        return Fail("summary JSON has an invalid Unicode escape");
      }
      value = (value << 4U) | static_cast<std::uint32_t>(digit);
    }
    position_ += 4U;
    *code_unit = value;
    return true;
  }

  bool AppendCodePoint(std::uint32_t code_point, std::string* output,
                       std::size_t* decoded_size) {
    std::array<char, 4U> encoded{};
    std::size_t encoded_size = 0U;
    if (code_point <= UINT32_C(0x7f)) {
      encoded[0U] = static_cast<char>(code_point);
      encoded_size = 1U;
    } else if (code_point <= UINT32_C(0x7ff)) {
      encoded[0U] = static_cast<char>(UINT32_C(0xc0) | (code_point >> 6U));
      encoded[1U] =
          static_cast<char>(UINT32_C(0x80) | (code_point & UINT32_C(0x3f)));
      encoded_size = 2U;
    } else if (code_point <= UINT32_C(0xffff)) {
      encoded[0U] = static_cast<char>(UINT32_C(0xe0) | (code_point >> 12U));
      encoded[1U] = static_cast<char>(UINT32_C(0x80) |
                                      ((code_point >> 6U) & UINT32_C(0x3f)));
      encoded[2U] =
          static_cast<char>(UINT32_C(0x80) | (code_point & UINT32_C(0x3f)));
      encoded_size = 3U;
    } else if (code_point <= UINT32_C(0x10ffff)) {
      encoded[0U] = static_cast<char>(UINT32_C(0xf0) | (code_point >> 18U));
      encoded[1U] = static_cast<char>(UINT32_C(0x80) |
                                      ((code_point >> 12U) & UINT32_C(0x3f)));
      encoded[2U] = static_cast<char>(UINT32_C(0x80) |
                                      ((code_point >> 6U) & UINT32_C(0x3f)));
      encoded[3U] =
          static_cast<char>(UINT32_C(0x80) | (code_point & UINT32_C(0x3f)));
      encoded_size = 4U;
    } else {
      return Fail("summary JSON Unicode code point is out of range");
    }
    if (*decoded_size > kMaximumJsonStringBytes - encoded_size) {
      return Fail("summary JSON string exceeds its bound");
    }
    *decoded_size += encoded_size;
    if (output != nullptr) {
      output->append(encoded.data(), encoded_size);
    }
    return true;
  }

  bool ParseUtf8(std::string* output, std::size_t* decoded_size) {
    const std::size_t start = position_;
    const auto first = static_cast<unsigned char>(input_[position_]);
    std::size_t length = 0U;
    if (first >= 0xc2U && first <= 0xdfU) {
      length = 2U;
    } else if (first >= 0xe0U && first <= 0xefU) {
      length = 3U;
    } else if (first >= 0xf0U && first <= 0xf4U) {
      length = 4U;
    } else {
      return Fail("summary JSON string contains invalid UTF-8");
    }
    if (position_ + length > input_.size()) {
      return Fail("summary JSON string contains truncated UTF-8");
    }
    for (std::size_t index = 1U; index < length; ++index) {
      const auto continuation =
          static_cast<unsigned char>(input_[position_ + index]);
      if (continuation < 0x80U || continuation > 0xbfU) {
        return Fail("summary JSON string contains invalid UTF-8 continuation");
      }
    }
    const auto second = static_cast<unsigned char>(input_[position_ + 1U]);
    if ((first == 0xe0U && second < 0xa0U) ||
        (first == 0xedU && second > 0x9fU) ||
        (first == 0xf0U && second < 0x90U) ||
        (first == 0xf4U && second > 0x8fU)) {
      return Fail("summary JSON string contains non-scalar UTF-8");
    }
    if (*decoded_size > kMaximumJsonStringBytes - length) {
      return Fail("summary JSON string exceeds its bound");
    }
    *decoded_size += length;
    position_ += length;
    if (output != nullptr) {
      output->append(input_.substr(start, length));
    }
    return true;
  }

  bool ParseString(std::string* output) {
    if (!Consume('"', "summary JSON value must be a string")) {
      return false;
    }
    if (output != nullptr) {
      output->clear();
    }
    std::size_t decoded_size = 0U;
    while (position_ < input_.size()) {
      const auto character = static_cast<unsigned char>(input_[position_]);
      if (character == static_cast<unsigned char>('"')) {
        ++position_;
        return true;
      }
      if (character < 0x20U) {
        return Fail("summary JSON string contains an unescaped control byte");
      }
      if (character >= 0x80U) {
        if (!ParseUtf8(output, &decoded_size)) {
          return false;
        }
        continue;
      }
      ++position_;
      if (character != static_cast<unsigned char>('\\')) {
        if (!AppendCodePoint(character, output, &decoded_size)) {
          return false;
        }
        continue;
      }
      if (position_ >= input_.size()) {
        return Fail("summary JSON has a truncated escape");
      }
      const char escape = input_[position_++];
      switch (escape) {
        case '"':
        case '\\':
        case '/':
          if (!AppendCodePoint(static_cast<unsigned char>(escape), output,
                               &decoded_size)) {
            return false;
          }
          break;
        case 'b':
          if (!AppendCodePoint(0x08U, output, &decoded_size)) {
            return false;
          }
          break;
        case 'f':
          if (!AppendCodePoint(0x0cU, output, &decoded_size)) {
            return false;
          }
          break;
        case 'n':
          if (!AppendCodePoint(0x0aU, output, &decoded_size)) {
            return false;
          }
          break;
        case 'r':
          if (!AppendCodePoint(0x0dU, output, &decoded_size)) {
            return false;
          }
          break;
        case 't':
          if (!AppendCodePoint(0x09U, output, &decoded_size)) {
            return false;
          }
          break;
        case 'u': {
          std::uint32_t code_point = 0U;
          if (!ParseHexCodeUnit(&code_point)) {
            return false;
          }
          if (code_point >= UINT32_C(0xd800) &&
              code_point <= UINT32_C(0xdbff)) {
            if (position_ + 2U > input_.size() || input_[position_] != '\\' ||
                input_[position_ + 1U] != 'u') {
              return Fail("summary JSON has an unpaired high surrogate");
            }
            position_ += 2U;
            std::uint32_t low_surrogate = 0U;
            if (!ParseHexCodeUnit(&low_surrogate) ||
                low_surrogate < UINT32_C(0xdc00) ||
                low_surrogate > UINT32_C(0xdfff)) {
              return Fail("summary JSON has an invalid low surrogate");
            }
            code_point = UINT32_C(0x10000) +
                         ((code_point - UINT32_C(0xd800)) << 10U) +
                         (low_surrogate - UINT32_C(0xdc00));
          } else if (code_point >= UINT32_C(0xdc00) &&
                     code_point <= UINT32_C(0xdfff)) {
            return Fail("summary JSON has an unpaired low surrogate");
          }
          if (!AppendCodePoint(code_point, output, &decoded_size)) {
            return false;
          }
          break;
        }
        default:
          return Fail("summary JSON has an invalid string escape");
      }
    }
    return Fail("summary JSON has an unterminated string");
  }

  bool ParseNumberToken(std::string_view* token) {
    const std::size_t start = position_;
    if (ConsumeIf('-') && position_ == input_.size()) {
      return Fail("summary JSON has a truncated number");
    }
    if (ConsumeIf('0')) {
      if (position_ < input_.size() && input_[position_] >= '0' &&
          input_[position_] <= '9') {
        return Fail("summary JSON number has a leading zero");
      }
    } else {
      if (position_ >= input_.size() || input_[position_] < '1' ||
          input_[position_] > '9') {
        return Fail("summary JSON has an invalid number");
      }
      while (position_ < input_.size() && input_[position_] >= '0' &&
             input_[position_] <= '9') {
        ++position_;
      }
    }
    if (ConsumeIf('.')) {
      if (position_ >= input_.size() || input_[position_] < '0' ||
          input_[position_] > '9') {
        return Fail("summary JSON number has an invalid fraction");
      }
      while (position_ < input_.size() && input_[position_] >= '0' &&
             input_[position_] <= '9') {
        ++position_;
      }
    }
    if (position_ < input_.size() &&
        (input_[position_] == 'e' || input_[position_] == 'E')) {
      ++position_;
      if (position_ < input_.size() &&
          (input_[position_] == '+' || input_[position_] == '-')) {
        ++position_;
      }
      if (position_ >= input_.size() || input_[position_] < '0' ||
          input_[position_] > '9') {
        return Fail("summary JSON number has an invalid exponent");
      }
      while (position_ < input_.size() && input_[position_] >= '0' &&
             input_[position_] <= '9') {
        ++position_;
      }
    }
    if (position_ - start > kMaximumJsonNumberBytes) {
      return Fail("summary JSON number exceeds its bound");
    }
    *token = input_.substr(start, position_ - start);
    return true;
  }

  template <typename Integer>
  bool ParseIntegerValue(Integer* value, std::size_t depth) {
    SkipWhitespace();
    if (!AccountNode(depth)) {
      return false;
    }
    std::string_view token;
    if (!ParseNumberToken(&token) ||
        token.find_first_of(".eE") != std::string_view::npos ||
        !ParseInteger(std::string(token), value)) {
      return Fail("summary JSON value must be a canonical bounded integer");
    }
    return true;
  }

  bool ParseStringValue(std::string* value, std::size_t depth) {
    SkipWhitespace();
    return AccountNode(depth) && ParseString(value);
  }

  bool ParseBooleanValue(bool* value, std::size_t depth) {
    SkipWhitespace();
    if (!AccountNode(depth)) {
      return false;
    }
    if (input_.substr(position_, 4U) == "true") {
      position_ += 4U;
      *value = true;
      return true;
    }
    if (input_.substr(position_, 5U) == "false") {
      position_ += 5U;
      *value = false;
      return true;
    }
    return Fail("summary JSON value must be boolean");
  }

  bool SkipObject(std::size_t depth) {
    if (!Consume('{', "summary JSON object is malformed")) {
      return false;
    }
    std::set<std::string> keys;
    SkipWhitespace();
    if (ConsumeIf('}')) {
      return true;
    }
    while (true) {
      SkipWhitespace();
      std::string key;
      if (!ParseString(&key) || !keys.insert(key).second) {
        return Fail("summary JSON object contains a duplicate key");
      }
      SkipWhitespace();
      if (!Consume(':', "summary JSON object key is missing ':'") ||
          !SkipValue(depth + 1U)) {
        return false;
      }
      SkipWhitespace();
      if (ConsumeIf('}')) {
        return true;
      }
      if (!Consume(',',
                   "summary JSON object members are not comma-separated")) {
        return false;
      }
    }
  }

  bool SkipArray(std::size_t depth) {
    if (!Consume('[', "summary JSON array is malformed")) {
      return false;
    }
    SkipWhitespace();
    if (ConsumeIf(']')) {
      return true;
    }
    while (true) {
      if (!SkipValue(depth + 1U)) {
        return false;
      }
      SkipWhitespace();
      if (ConsumeIf(']')) {
        return true;
      }
      if (!Consume(',', "summary JSON array members are not comma-separated")) {
        return false;
      }
    }
  }

  bool SkipValue(std::size_t depth) {
    SkipWhitespace();
    if (!AccountNode(depth) || position_ >= input_.size()) {
      return Fail("summary JSON value is missing");
    }
    const char character = input_[position_];
    if (character == '{') {
      return SkipObject(depth);
    }
    if (character == '[') {
      return SkipArray(depth);
    }
    if (character == '"') {
      return ParseString(nullptr);
    }
    if (character == '-' || (character >= '0' && character <= '9')) {
      std::string_view token;
      return ParseNumberToken(&token);
    }
    if (input_.substr(position_, 4U) == "true" ||
        input_.substr(position_, 4U) == "null") {
      position_ += 4U;
      return true;
    }
    if (input_.substr(position_, 5U) == "false") {
      position_ += 5U;
      return true;
    }
    return Fail("summary JSON value has an invalid type or literal");
  }

  bool ParseValueOfType(char expected, std::size_t depth, const char* message) {
    SkipWhitespace();
    if (position_ >= input_.size() || input_[position_] != expected) {
      return Fail(message);
    }
    return SkipValue(depth);
  }

  bool ParseRate(std::size_t depth, std::size_t expected_index,
                 CampaignProfile::Rate* rate) {
    SkipWhitespace();
    if (!AccountNode(depth) ||
        !Consume('{', "campaign identity rate must be an object")) {
      return false;
    }
    std::set<std::string> fields;
    std::optional<std::string> name;
    std::optional<std::uint32_t> numerator;
    std::optional<std::uint32_t> denominator;
    SkipWhitespace();
    if (ConsumeIf('}')) {
      return Fail("campaign identity rate object is empty");
    }
    while (true) {
      SkipWhitespace();
      std::string field;
      if (!ParseString(&field) || !fields.insert(field).second) {
        return Fail("campaign identity rate has a duplicate key");
      }
      SkipWhitespace();
      if (!Consume(':', "campaign identity rate key is missing ':'")) {
        return false;
      }
      if (field == "name") {
        std::string value;
        if (!ParseStringValue(&value, depth + 1U)) {
          return false;
        }
        name = std::move(value);
      } else if (field == "numerator") {
        std::uint32_t value = 0U;
        if (!ParseIntegerValue(&value, depth + 1U)) {
          return false;
        }
        numerator = value;
      } else if (field == "denominator_seconds") {
        std::uint32_t value = 0U;
        if (!ParseIntegerValue(&value, depth + 1U)) {
          return false;
        }
        denominator = value;
      } else {
        return Fail("campaign identity rate has an unknown key");
      }
      SkipWhitespace();
      if (ConsumeIf('}')) {
        break;
      }
      if (!Consume(',',
                   "campaign identity rate members are not comma-separated")) {
        return false;
      }
    }
    if (fields.size() != 3U || !name.has_value() || !numerator.has_value() ||
        !denominator.has_value() || *name != kCommandNames[expected_index]) {
      return Fail(
          "campaign identity rate is missing, duplicated, or out of order");
    }
    rate->numerator = *numerator;
    rate->denominator_seconds = *denominator;
    return true;
  }

  bool ParseRates(
      std::size_t depth,
      std::array<CampaignProfile::Rate, kCampaignCommandCount>* rates) {
    SkipWhitespace();
    if (!AccountNode(depth) ||
        !Consume('[', "campaign identity command rates must be an array")) {
      return false;
    }
    for (std::size_t index = 0U; index < rates->size(); ++index) {
      SkipWhitespace();
      if (index != 0U &&
          !Consume(',', "campaign identity rates are not comma-separated")) {
        return false;
      }
      if (!ParseRate(depth + 1U, index, &(*rates)[index])) {
        return false;
      }
    }
    SkipWhitespace();
    return Consume(']', "campaign identity rates have the wrong length");
  }

  bool ParseCampaignIdentity(std::size_t depth) {
    SkipWhitespace();
    if (!AccountNode(depth) ||
        !Consume('{', "campaign_identity must be an object")) {
      return false;
    }
    std::set<std::string> fields;
    std::optional<std::string> mode_name;
    std::optional<std::int64_t> configured_duration_ns;
    std::optional<std::string> profile_mode_name;
    std::optional<std::int64_t> canonical_duration_ns;
    std::optional<std::int64_t> service_round_period_ns;
    std::optional<std::uint32_t> expected_cycles;
    std::optional<bool> continuous_session_required;
    std::optional<std::array<CampaignProfile::Rate, kCampaignCommandCount>>
        command_rates;
    std::optional<std::string> identity_digest;
    SkipWhitespace();
    if (ConsumeIf('}')) {
      return Fail("campaign_identity object is empty");
    }
    while (true) {
      SkipWhitespace();
      std::string field;
      if (!ParseString(&field) || !fields.insert(field).second) {
        return Fail("campaign_identity contains a duplicate key");
      }
      SkipWhitespace();
      if (!Consume(':', "campaign_identity key is missing ':'")) {
        return false;
      }
      if (field == "mode" || field == "profile_mode" ||
          field == "identity_sha256") {
        std::string value;
        if (!ParseStringValue(&value, depth + 1U)) {
          return false;
        }
        if (field == "mode") {
          mode_name = std::move(value);
        } else if (field == "profile_mode") {
          profile_mode_name = std::move(value);
        } else {
          identity_digest = std::move(value);
        }
      } else if (field == "configured_duration_ns" ||
                 field == "profile_canonical_duration_ns" ||
                 field == "profile_service_round_period_ns") {
        std::int64_t value = 0;
        if (!ParseIntegerValue(&value, depth + 1U)) {
          return false;
        }
        if (field == "configured_duration_ns") {
          configured_duration_ns = value;
        } else if (field == "profile_canonical_duration_ns") {
          canonical_duration_ns = value;
        } else {
          service_round_period_ns = value;
        }
      } else if (field == "profile_expected_cycles") {
        std::uint32_t value = 0U;
        if (!ParseIntegerValue(&value, depth + 1U)) {
          return false;
        }
        expected_cycles = value;
      } else if (field == "profile_continuous_session_required") {
        bool value = false;
        if (!ParseBooleanValue(&value, depth + 1U)) {
          return false;
        }
        continuous_session_required = value;
      } else if (field == "profile_command_rates") {
        std::array<CampaignProfile::Rate, kCampaignCommandCount> rates{};
        if (!ParseRates(depth + 1U, &rates)) {
          return false;
        }
        command_rates = rates;
      } else {
        return Fail("campaign_identity contains an unknown key");
      }

      SkipWhitespace();
      if (ConsumeIf('}')) {
        break;
      }
      if (!Consume(',', "campaign_identity members are not comma-separated")) {
        return false;
      }
    }

    if (fields.size() != 9U || !mode_name.has_value() ||
        !configured_duration_ns.has_value() || !profile_mode_name.has_value() ||
        !canonical_duration_ns.has_value() ||
        !service_round_period_ns.has_value() || !expected_cycles.has_value() ||
        !continuous_session_required.has_value() ||
        !command_rates.has_value() || !identity_digest.has_value() ||
        !IsLowerSha256(*identity_digest)) {
      return Fail("campaign_identity is missing a required typed field");
    }
    const std::optional<CampaignMode> mode = ParseCampaignMode(*mode_name);
    const std::optional<CampaignMode> profile_mode =
        ParseCampaignMode(*profile_mode_name);
    if (!mode.has_value() || !profile_mode.has_value()) {
      return Fail("campaign_identity contains an unknown mode");
    }
    CampaignProfile profile;
    profile.mode = *profile_mode;
    profile.canonical_duration_ns = *canonical_duration_ns;
    profile.service_round_period_ns = *service_round_period_ns;
    profile.expected_cycles = *expected_cycles;
    profile.continuous_session_required = *continuous_session_required;
    profile.command_rates = *command_rates;
    if (!IsValidCampaignIdentity(*mode, profile, *configured_duration_ns,
                                 error_)) {
      return false;
    }
    if (*mode != expectation_.mode ||
        !ProfilesEqual(profile, expectation_.profile) ||
        *configured_duration_ns != expectation_.configured_duration_ns) {
      return Fail("summary campaign_identity does not match expected");
    }
    const std::string identity =
        CanonicalCampaignIdentity(*mode, profile, *configured_duration_ns);
    if (*identity_digest != Sha256Hex(identity)) {
      return Fail("summary campaign_identity digest is invalid");
    }
    return true;
  }

  std::string_view input_;
  const CampaignEvidenceExpectation& expectation_;
  std::string* error_ = nullptr;
  std::size_t position_ = 0U;
  std::size_t node_count_ = 0U;
};

bool ParseAndValidateSummaryJson(const std::string& contents,
                                 const CampaignEvidenceExpectation& expectation,
                                 std::string* error) {
  SummaryJsonParser parser(contents, expectation, error);
  return parser.Parse();
}

}  // namespace

bool VerifyCampaignEvidence(const std::string& directory,
                            const CampaignEvidenceExpectation& expectation,
                            std::string* error) {
  if (error == nullptr) {
    return false;
  }
  error->clear();
  if (!IsValidCampaignIdentity(expectation.mode, expectation.profile,
                               expectation.configured_duration_ns, error)) {
    return false;
  }
  const std::filesystem::path evidence_directory(directory);
  if (!evidence_directory.is_absolute() ||
      evidence_directory.lexically_normal() != evidence_directory) {
    *error = "evidence directory must be an absolute normalized path";
    return false;
  }
  std::error_code filesystem_error;
  const std::filesystem::file_status directory_status =
      std::filesystem::symlink_status(evidence_directory, filesystem_error);
  if (filesystem_error || std::filesystem::is_symlink(directory_status) ||
      !std::filesystem::is_directory(directory_status)) {
    *error = "evidence path must be a non-symlink directory";
    return false;
  }
  if (!IsReadOnly(directory_status.permissions())) {
    *error = "evidence directory is not read-only";
    return false;
  }

  std::set<std::string> observed_files;
  std::filesystem::directory_iterator iterator(evidence_directory,
                                               filesystem_error);
  if (filesystem_error) {
    *error =
        "cannot enumerate evidence directory: " + filesystem_error.message();
    return false;
  }
  const std::filesystem::directory_iterator end;
  while (iterator != end) {
    const std::filesystem::directory_entry entry = *iterator;
    const std::string name = entry.path().filename().string();
    if (!IsCanonicalChildName(name) || !observed_files.insert(name).second) {
      *error = "evidence contains a malformed or duplicate child name";
      return false;
    }
    const std::filesystem::file_status status =
        entry.symlink_status(filesystem_error);
    if (filesystem_error || std::filesystem::is_symlink(status) ||
        !std::filesystem::is_regular_file(status)) {
      *error = "evidence child is not a regular non-symlink file: " + name;
      return false;
    }
    if (!IsReadOnly(status.permissions())) {
      *error = "evidence file is not read-only: " + name;
      return false;
    }
    iterator.increment(filesystem_error);
    if (filesystem_error) {
      *error = "cannot finish enumerating evidence directory: " +
               filesystem_error.message();
      return false;
    }
  }
  std::set<std::string> expected_files{std::string(kManifestName)};
  expected_files.insert(kPayloadNames.begin(), kPayloadNames.end());
  if (observed_files != expected_files) {
    *error = "evidence directory has missing or extra files";
    return false;
  }

  std::string manifest_contents;
  if (!ReadBoundedFile(evidence_directory / kManifestName,
                       kMaximumManifestBytes, &manifest_contents, error)) {
    return false;
  }
  ParsedManifest manifest;
  if (!ParseManifest(manifest_contents, &manifest, error)) {
    return false;
  }
  if (manifest.mode != expectation.mode ||
      !ProfilesEqual(manifest.profile, expectation.profile) ||
      manifest.configured_duration_ns != expectation.configured_duration_ns) {
    *error = "manifest campaign mode/profile/duration does not match expected";
    return false;
  }
  const std::string expected_identity =
      CanonicalCampaignIdentity(expectation.mode, expectation.profile,
                                expectation.configured_duration_ns);
  if (manifest.identity_digest != Sha256Hex(expected_identity)) {
    *error = "manifest campaign identity digest is invalid";
    return false;
  }

  for (const char* payload_name : kPayloadNames) {
    const PayloadEntry& expected_payload = manifest.payloads.at(payload_name);
    if (expected_payload.size > kMaximumPayloadBytes) {
      *error = "manifest payload exceeds its size bound: " +
               std::string(payload_name);
      return false;
    }
    std::string observed_digest;
    std::uint64_t observed_size = 0U;
    if (!Sha256File(evidence_directory / payload_name, &observed_digest,
                    &observed_size, error)) {
      return false;
    }
    if (observed_size != expected_payload.size ||
        observed_digest != expected_payload.digest) {
      *error = "payload size or SHA-256 mismatch: " + std::string(payload_name);
      return false;
    }
  }

  std::string summary_contents;
  if (!ReadBoundedFile(evidence_directory / "summary.json",
                       kMaximumPayloadBytes, &summary_contents, error)) {
    return false;
  }
  const PayloadEntry& summary_payload = manifest.payloads.at("summary.json");
  if (summary_contents.size() != summary_payload.size ||
      Sha256Hex(summary_contents) != summary_payload.digest) {
    *error = "summary changed between integrity and structural validation";
    return false;
  }
  return ParseAndValidateSummaryJson(summary_contents, expectation, error);
}

}  // namespace mentor_pi_bringup
