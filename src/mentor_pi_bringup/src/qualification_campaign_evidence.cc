// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include "mentor_pi_bringup/qualification_campaign_evidence.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "mentor_pi_bringup/qualification_campaign_core.h"

#if defined(__APPLE__)
#include <unistd.h>

#include <cstdio>
#elif defined(__linux__)
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include "mentor_pi_bringup/qualification_campaign_evidence_internal.h"

namespace mentor_pi_bringup {
namespace {

constexpr std::array<const char*, kCampaignCommandCount> kCommandNames{
    "motor", "pwm_servo", "bus_servo", "led", "buzzer", "rgb", "oled"};
constexpr std::array<const char*, kCampaignServiceCount> kServiceNames{
    "motor_model",   "pwm_offsets", "bus_state",
    "bus_configure", "bus_stop",    "battery_threshold"};
constexpr std::array<const char*, kCampaignTelemetryCount> kTelemetryNames{
    "heartbeat", "diagnostics", "motor_state",  "pwm_servo_state",
    "imu",       "battery",     "button_events"};
constexpr std::array<const char*, 24U> kFailureNames{
    "INVALID_CONFIGURATION",
    "EVIDENCE_NOT_STARTED",
    "DURATION_INCOMPLETE",
    "SCHEDULE_MISSED",
    "COMMAND_COUNT_MISMATCH",
    "NONZERO_MOTOR_ATTEMPT",
    "INVALID_TELEMETRY",
    "TELEMETRY_RATE",
    "SESSION_CHANGED",
    "SESSION_INVALID",
    "SESSION_CYCLE_MISSING",
    "ENDPOINT_LOST",
    "ENDPOINT_RECOVERY_TIMEOUT",
    "UNEXPECTED_RESET",
    "RESET_CYCLE_MISSING",
    "DIAGNOSTIC_REGRESSION",
    "DIAGNOSTIC_ERROR",
    "TRANSPORT_RATE",
    "MOTOR_AGE_BASELINE",
    "MOTOR_AGE_EVIDENCE_MISSING",
    "MOTOR_AGE_P99",
    "MOTOR_AGE_MAXIMUM",
    "SERVICE_FAILURE",
    "SERVICE_COVERAGE"};
constexpr std::array<const char*, 4U> kPayloadNames{
    "summary.json", "metrics.csv", "session-transitions.csv", "junit.xml"};
constexpr std::string_view kManifestName = "manifest.sha256";

std::string JsonEscape(const std::string& value) {
  std::ostringstream escaped;
  for (const char raw_character : value) {
    const auto character = static_cast<unsigned char>(raw_character);
    switch (character) {
      case '"':
        escaped << "\\\"";
        break;
      case '\\':
        escaped << "\\\\";
        break;
      case '\b':
        escaped << "\\b";
        break;
      case '\f':
        escaped << "\\f";
        break;
      case '\n':
        escaped << "\\n";
        break;
      case '\r':
        escaped << "\\r";
        break;
      case '\t':
        escaped << "\\t";
        break;
      default:
        if (character < 0x20U) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<unsigned int>(character) << std::dec;
        } else {
          escaped << static_cast<char>(character);
        }
        break;
    }
  }
  return escaped.str();
}

std::string XmlEscape(const std::string& value) {
  std::string escaped;
  for (const char character : value) {
    switch (character) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      case '"':
        escaped += "&quot;";
        break;
      case '\'':
        escaped += "&apos;";
        break;
      default:
        escaped += character;
        break;
    }
  }
  return escaped;
}

bool WriteFile(const std::filesystem::path& destination,
               const std::string& contents, std::string* error) {
  std::ofstream output(destination,
                       std::ios::out | std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
    *error = "cannot open " + destination.string();
    return false;
  }
  output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  output.close();
  if (!output) {
    *error = "cannot finish " + destination.string();
    return false;
  }
  return true;
}

void CleanupStagingDirectory(const std::filesystem::path& staging_directory,
                             std::string* error) {
  std::error_code permission_error;
  std::filesystem::permissions(
      staging_directory,
      std::filesystem::perms::owner_read | std::filesystem::perms::owner_write |
          std::filesystem::perms::owner_exec,
      std::filesystem::perm_options::add, permission_error);
  std::error_code cleanup_error;
  std::filesystem::remove_all(staging_directory, cleanup_error);
  if (cleanup_error && error != nullptr) {
    if (!error->empty()) {
      *error += "; ";
    }
    *error += "exact staging cleanup failed: " + cleanup_error.message();
  }
}

bool CreateSiblingStagingDirectory(
    const std::filesystem::path& output_directory,
    std::filesystem::path* staging_directory, std::string* error) {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto clock_value =
      std::chrono::steady_clock::now().time_since_epoch().count();
#if defined(__APPLE__) || defined(__linux__)
  const auto process_id = static_cast<std::uint64_t>(getpid());
#else
  const std::uint64_t process_id = 0U;
#endif
  for (std::uint32_t attempt = 0U; attempt < 64U; ++attempt) {
    const std::uint64_t unique =
        sequence.fetch_add(1U, std::memory_order_relaxed);
    std::ostringstream name;
    name << ".rrclite-evidence-staging-" << process_id << '-' << clock_value
         << '-' << unique << '-' << attempt;
    const std::filesystem::path candidate =
        output_directory.parent_path() / name.str();
    std::error_code create_error;
    if (std::filesystem::create_directory(candidate, create_error)) {
      *staging_directory = candidate;
      return true;
    }
    if (create_error && create_error != std::errc::file_exists) {
      *error =
          "cannot create sibling staging directory: " + create_error.message();
      return false;
    }
  }
  *error = "cannot allocate a unique sibling staging directory";
  return false;
}

bool AtomicRenameNoReplace(const std::filesystem::path& source,
                           const std::filesystem::path& destination,
                           std::string* error) {
#if defined(__APPLE__)
  if (renamex_np(source.c_str(), destination.c_str(), RENAME_EXCL) == 0) {
    return true;
  }
#elif defined(__linux__) && defined(SYS_renameat2)
  constexpr unsigned int kRenameNoReplace = 1U;
  if (syscall(SYS_renameat2, AT_FDCWD, source.c_str(), AT_FDCWD,
              destination.c_str(), kRenameNoReplace) == 0) {
    return true;
  }
#else
  *error =
      "atomic no-replace directory publication is unsupported on this host";
  return false;
#endif
  *error = "cannot atomically publish evidence without replacement: " +
           std::string(std::strerror(errno));
  return false;
}

bool ApplyReadOnlyModes(const std::filesystem::path& staging_directory,
                        std::string* error) {
  constexpr std::filesystem::perms kFilePermissions =
      std::filesystem::perms::owner_read | std::filesystem::perms::group_read |
      std::filesystem::perms::others_read;
  constexpr std::filesystem::perms kDirectoryPermissions =
      kFilePermissions | std::filesystem::perms::owner_exec |
      std::filesystem::perms::group_exec | std::filesystem::perms::others_exec;
  std::error_code permission_error;
  for (const char* payload_name : kPayloadNames) {
    std::filesystem::permissions(
        staging_directory / payload_name, kFilePermissions,
        std::filesystem::perm_options::replace, permission_error);
    if (permission_error) {
      *error = "cannot make evidence payload read-only: " +
               permission_error.message();
      return false;
    }
  }
  std::filesystem::permissions(
      staging_directory / kManifestName, kFilePermissions,
      std::filesystem::perm_options::replace, permission_error);
  if (permission_error) {
    *error = "cannot make evidence manifest read-only: " +
             permission_error.message();
    return false;
  }
  std::filesystem::permissions(staging_directory, kDirectoryPermissions,
                               std::filesystem::perm_options::replace,
                               permission_error);
  if (permission_error) {
    *error = "cannot make evidence directory read-only: " +
             permission_error.message();
    return false;
  }
  return true;
}

bool IsSafePathComponent(const std::string& value) {
  if (value.empty() || value == "." || value == ".." ||
      value.find("..") != std::string::npos) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char raw_character) {
    const auto character = static_cast<unsigned char>(raw_character);
    return std::isalnum(character) != 0 || character == '-' ||
           character == '_' || character == '.';
  });
}

bool HasFiniteSummaryNumbers(const CampaignSummary& summary) {
  if (!std::isfinite(summary.maximum_transport_interval_bytes_per_second)) {
    return false;
  }
  return std::all_of(summary.telemetry_rates_hz.begin(),
                     summary.telemetry_rates_hz.end(),
                     [](double rate) { return std::isfinite(rate); });
}

void AppendJsonString(std::ostringstream* output, const char* name,
                      const std::string& value, bool comma = true) {
  *output << "    \"" << name << "\": \"" << JsonEscape(value) << "\"";
  if (comma) {
    *output << ',';
  }
  *output << '\n';
}

void AppendMetricCsv(std::ostringstream* output, const std::string& name,
                     CampaignMetricStatus status, const std::string& value,
                     const std::string& unit, const std::string& threshold,
                     const std::string& note) {
  *output << name << ',' << CampaignMetricStatusName(status) << ',' << value
          << ',' << unit << ',' << threshold << ',' << '"' << JsonEscape(note)
          << '"' << '\n';
}

void AppendJUnitCase(std::ostringstream* output, const std::string& name,
                     CampaignMetricStatus status, const std::string& detail) {
  *output << R"(    <testcase classname="rrclite.qualification" name=")"
          << XmlEscape(name) << "\">\n";
  if (status == CampaignMetricStatus::kFail) {
    *output << "      <failure message=\"" << XmlEscape(detail) << "\"/>\n";
  } else if (status == CampaignMetricStatus::kNotObserved ||
             status == CampaignMetricStatus::kNotApplicable) {
    *output << "      <skipped message=\"" << XmlEscape(detail) << "\"/>\n";
  }
  *output << "    </testcase>\n";
}

}  // namespace

namespace campaign_evidence_internal {

bool AtomicRenameDirectoryNoReplace(const std::filesystem::path& source,
                                    const std::filesystem::path& destination,
                                    std::string* error) {
  if (error == nullptr) {
    return false;
  }
  return AtomicRenameNoReplace(source, destination, error);
}

}  // namespace campaign_evidence_internal

const char* CampaignMetricStatusName(CampaignMetricStatus status) {
  switch (status) {
    case CampaignMetricStatus::kPass:
      return "PASS";
    case CampaignMetricStatus::kFail:
      return "FAIL";
    case CampaignMetricStatus::kNotObserved:
      return "NOT_OBSERVED";
    case CampaignMetricStatus::kNotApplicable:
      return "NOT_APPLICABLE";
  }
  return "UNKNOWN";
}

bool IsValidCampaignEvidenceToken(const std::string& value,
                                  std::size_t maximum_length) {
  if (value.empty() || value.size() > maximum_length) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char raw_character) {
    const auto character = static_cast<unsigned char>(raw_character);
    return std::isalnum(character) != 0 || character == '-' ||
           character == '_' || character == '.' || character == '+' ||
           character == ':' || character == '/';
  });
}

bool IsValidSha256(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char raw_character) {
    const auto character = static_cast<unsigned char>(raw_character);
    return std::isxdigit(character) != 0;
  });
}

bool WriteCampaignEvidence(const std::string& directory,
                           const CampaignEvidenceMetadata& metadata,
                           const CampaignSummary& summary, std::string* error) {
  if (error == nullptr) {
    return false;
  }
  error->clear();
  if (!IsSafePathComponent(metadata.run_id) || metadata.run_id.size() > 96U ||
      !IsValidCampaignEvidenceToken(metadata.source_revision, 128U) ||
      !IsValidSha256(metadata.firmware_sha256) ||
      !IsValidCampaignEvidenceToken(metadata.host_revision, 128U) ||
      !IsValidCampaignEvidenceToken(metadata.ros_distribution, 32U) ||
      !IsValidCampaignEvidenceToken(metadata.campaign_mode, 32U) ||
      !IsValidCampaignEvidenceToken(metadata.board_serial, 96U) ||
      !IsValidCampaignEvidenceToken(metadata.fixture_revision, 96U) ||
      !IsValidCampaignEvidenceToken(metadata.start_time_utc, 32U) ||
      !IsValidCampaignEvidenceToken(metadata.finish_time_utc, 32U)) {
    *error = "required evidence metadata is missing or malformed";
    return false;
  }
  if (metadata.campaign_mode != CampaignModeName(summary.mode)) {
    *error = "metadata campaign_mode does not match the Summary mode";
    return false;
  }
  if (!campaign_evidence_internal::IsValidCampaignIdentity(
          summary.mode, summary.profile, summary.configured_duration_ns,
          error)) {
    return false;
  }
  if (summary.stored_transition_count > summary.transitions.size()) {
    *error = "stored transition count exceeds its fixed bound";
    return false;
  }
  if (!HasFiniteSummaryNumbers(summary)) {
    *error = "numeric evidence contains NaN or infinity";
    return false;
  }
  const std::filesystem::path output_directory(directory);
  if (!output_directory.is_absolute() ||
      output_directory.lexically_normal() != output_directory) {
    *error = "evidence directory must be absolute and normalized";
    return false;
  }
  std::error_code filesystem_error;
  const std::filesystem::file_status output_status =
      std::filesystem::symlink_status(output_directory, filesystem_error);
  if (!filesystem_error &&
      output_status.type() != std::filesystem::file_type::not_found) {
    *error =
        "refusing existing evidence directory: " + output_directory.string();
    return false;
  }
  if (filesystem_error &&
      filesystem_error != std::errc::no_such_file_or_directory) {
    *error =
        "cannot inspect evidence destination: " + filesystem_error.message();
    return false;
  }
  const std::filesystem::path parent = output_directory.parent_path();
  filesystem_error.clear();
  const std::filesystem::file_status parent_status =
      std::filesystem::symlink_status(parent, filesystem_error);
  if (filesystem_error || !std::filesystem::is_directory(parent_status) ||
      std::filesystem::is_symlink(parent_status)) {
    *error = "evidence parent must be an existing non-symlink directory";
    return false;
  }
  std::filesystem::path staging_directory;
  if (!CreateSiblingStagingDirectory(output_directory, &staging_directory,
                                     error)) {
    return false;
  }
  std::ostringstream json;
  json << "{\n  \"schema_version\": 1,\n  \"metadata\": {\n";
  AppendJsonString(&json, "run_id", metadata.run_id);
  AppendJsonString(&json, "source_revision", metadata.source_revision);
  AppendJsonString(&json, "firmware_sha256", metadata.firmware_sha256);
  AppendJsonString(&json, "host_revision", metadata.host_revision);
  AppendJsonString(&json, "ros_distribution", metadata.ros_distribution);
  AppendJsonString(&json, "board_serial", metadata.board_serial);
  AppendJsonString(&json, "fixture_revision", metadata.fixture_revision);
  AppendJsonString(&json, "campaign_mode", metadata.campaign_mode);
  AppendJsonString(&json, "start_time_utc", metadata.start_time_utc);
  AppendJsonString(&json, "finish_time_utc", metadata.finish_time_utc, false);
  json << "  },\n";
  json << campaign_evidence_internal::CanonicalSummaryIdentityJson(
      summary.mode, summary.profile, summary.configured_duration_ns);
  json << "  \"result\": {\n"
       << R"(    "execution": ")"
       << (summary.execution_passed ? "PASS" : "FAIL") << "\",\n"
       << "    \"release_qualification\": \"INCOMPLETE\",\n"
       << "    \"canonical_profile\": "
       << (summary.canonical_profile ? "true" : "false") << ",\n"
       << R"(    "failure_mask": "0x)" << std::hex << std::setw(16)
       << std::setfill('0') << summary.failure_mask << std::dec << "\"\n"
       << "  },\n  \"failure_names\": [";
  bool first_failure = true;
  for (std::size_t index = 0U; index < kFailureNames.size(); ++index) {
    if ((summary.failure_mask & (UINT64_C(1) << index)) == 0U) {
      continue;
    }
    json << (first_failure ? "\n" : ",\n") << "    \"" << kFailureNames[index]
         << "\"";
    first_failure = false;
  }
  if (!first_failure) {
    json << '\n';
  }
  if ((summary.failure_mask >> kFailureNames.size()) != 0U) {
    json << (first_failure ? "\n" : ",\n") << "    \"UNKNOWN_FAILURE_BIT\"\n";
    first_failure = false;
  }
  static_cast<void>(first_failure);
  json << "  ],\n  \"session\": {\n"
       << "    \"initial_id\": " << summary.initial_session_id << ",\n"
       << "    \"final_id\": " << summary.final_session_id << ",\n"
       << "    \"observed_cycles\": " << summary.observed_cycles << ",\n"
       << "    \"observed_mcu_resets\": " << summary.observed_mcu_resets
       << "\n  },\n  \"motor_age\": {\n"
       << "    \"consumptions\": " << summary.motor_command_consumptions
       << ",\n    \"over_20_ms\": " << summary.motor_command_age_over_20_ms
       << ",\n    \"maximum_us\": " << summary.motor_command_max_age_us
       << "\n  },\n  \"transport\": {\n"
       << "    \"maximum_diagnostics_interval_bytes_per_second\": "
       << std::fixed << std::setprecision(3)
       << summary.maximum_transport_interval_bytes_per_second
       << ",\n    \"independent_capture\": \"NOT_OBSERVED\"\n  },\n"
       << "  \"commands\": {\n";
  for (std::size_t index = 0U; index < kCommandNames.size(); ++index) {
    json << "    \"" << kCommandNames[index]
         << "\": " << summary.command_publications[index]
         << (index + 1U == kCommandNames.size() ? "\n" : ",\n");
  }
  json << "  },\n  \"services\": {\n";
  for (std::size_t index = 0U; index < kServiceNames.size(); ++index) {
    json << "    \"" << kServiceNames[index] << R"(": {"requests": )"
         << summary.service_requests[index]
         << ", \"successes\": " << summary.service_successes[index]
         << ", \"failures\": " << summary.service_failures[index]
         << ", \"skips\": " << summary.service_skips[index] << "}"
         << (index + 1U == kServiceNames.size() ? "\n" : ",\n");
  }
  json << "  },\n  \"telemetry\": {\n";
  for (std::size_t index = 0U; index < kTelemetryNames.size(); ++index) {
    json << "    \"" << kTelemetryNames[index] << R"(": {"samples": )"
         << summary.telemetry_samples[index]
         << ", \"rate_hz\": " << summary.telemetry_rates_hz[index] << "}"
         << (index + 1U == kTelemetryNames.size() ? "\n" : ",\n");
  }
  json << "  },\n  \"unobservable_release_metrics\": [\n"
       << "    \"independent_serial_capture\",\n"
       << "    \"every_complete_one_second_wire_window\",\n"
       << "    \"physical_motor_response_and_lease_timing\",\n"
       << "    \"physical_availability_to_endpoint_recovery\",\n"
       << "    \"measured_task_stack_headroom\",\n"
       << "    \"external_steady_state_allocation_trace\",\n"
       << "    \"functional_peripheral_HIL\"\n  ]\n}\n";

  std::ostringstream metrics;
  metrics << "metric,status,value,unit,threshold,note\n";
  AppendMetricCsv(&metrics, "execution",
                  summary.execution_passed ? CampaignMetricStatus::kPass
                                           : CampaignMetricStatus::kFail,
                  summary.execution_passed ? "1" : "0", "boolean", "1",
                  "host-observable campaign checks only");
  AppendMetricCsv(&metrics, "release_qualification",
                  CampaignMetricStatus::kNotObserved, "0", "boolean", "1",
                  "physical HIL and independent captures are required");
  AppendMetricCsv(
      &metrics, "internal_wire_traffic", summary.internal_wire_traffic,
      std::to_string(summary.maximum_transport_interval_bytes_per_second),
      "bytes_per_second", "<70000",
      "derived from MCU diagnostics intervals, not independent capture");
  AppendMetricCsv(&metrics, "independent_wire_capture",
                  summary.independent_wire_capture, "", "bytes_per_second",
                  "<70000 every complete second", "external instrument needed");
  AppendMetricCsv(&metrics, "complete_one_second_wire_windows",
                  summary.complete_one_second_wire_windows, "",
                  "bytes_per_second", "<70000 each complete second",
                  "diagnostics intervals cannot reconstruct aligned windows");
  AppendMetricCsv(&metrics, "connected_telemetry_gaps",
                  summary.connected_telemetry_gaps, "", "milliseconds",
                  "<=2 declared periods",
                  "evaluated only while the host observes a connected graph");
  AppendMetricCsv(&metrics, "motor_age_p99", summary.motor_age_p99,
                  std::to_string(summary.motor_command_age_over_20_ms),
                  "strict_over_20_ms_samples", "<=1% of consumptions",
                  "nearest-rank threshold inferred from MCU counters");
  AppendMetricCsv(&metrics, "motor_age_maximum", summary.motor_age_maximum,
                  std::to_string(summary.motor_command_max_age_us),
                  "microseconds", "<100000", "MCU diagnostic high-water");
  AppendMetricCsv(&metrics, "physical_motor_response",
                  summary.physical_motor_response, "", "", "",
                  "requires guarded motor HIL");
  AppendMetricCsv(&metrics, "physical_endpoint_recovery",
                  summary.physical_endpoint_recovery, "", "milliseconds",
                  "<=5000 from physical availability",
                  "host cannot observe physical availability directly");
  AppendMetricCsv(&metrics, "usb_outage_rotation", summary.usb_outage_rotation,
                  "", "seconds", "1,2,5 rotation",
                  "heartbeat gaps do not establish physical disconnect timing");
  AppendMetricCsv(&metrics, "stack_headroom", summary.stack_headroom, "",
                  "percent", ">=25", "requires target stack measurements");
  AppendMetricCsv(
      &metrics, "allocation_trace", summary.allocation_trace, "", "allocations",
      "0", "diagnostic attempt counter is checked; external trace absent");
  AppendMetricCsv(&metrics, "button_stimulus", summary.button_stimulus,
                  std::to_string(summary.telemetry_samples[6U]), "events",
                  ">=1Hz", "requires operator or fixture button actuation");
  AppendMetricCsv(&metrics, "imu_function", summary.imu_function,
                  std::to_string(summary.telemetry_samples[4U]), "samples",
                  "50Hz valid", "requires characterized IMU transform");

  std::ostringstream transitions;
  transitions << "cycle,previous_session_id,new_session_id,heartbeat_gap_ms,"
                 "connected_interval_ms,"
                 "endpoint_recovery_ms,endpoint_recovered,"
                 "heartbeat_outage_observed,mcu_uptime_regression_observed,"
                 "reset_reason\n";
  for (std::size_t index = 0U; index < summary.stored_transition_count;
       ++index) {
    const CampaignTransition& transition = summary.transitions[index];
    transitions << transition.cycle << ',' << transition.previous_session_id
                << ',' << transition.new_session_id << ','
                << transition.heartbeat_gap_ms << ','
                << transition.connected_interval_ms << ','
                << transition.endpoint_recovery_ms << ','
                << (transition.endpoint_recovered ? 1 : 0) << ','
                << (transition.heartbeat_outage_observed ? 1 : 0) << ','
                << (transition.mcu_uptime_regression_observed ? 1 : 0) << ','
                << static_cast<unsigned int>(transition.reset_reason) << '\n';
  }

  const CampaignMetricStatus execution_status =
      summary.execution_passed ? CampaignMetricStatus::kPass
                               : CampaignMetricStatus::kFail;
  std::ostringstream junit;
  junit << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        << "<testsuites>\n  <testsuite name=\"rrclite-qualification-"
        << XmlEscape(metadata.run_id) << "\">\n";
  AppendJUnitCase(&junit, "host_observable_execution", execution_status,
                  "one or more host-observable checks failed");
  AppendJUnitCase(&junit, "internal_wire_traffic",
                  summary.internal_wire_traffic,
                  "MCU diagnostics traffic interval was unavailable or failed");
  AppendJUnitCase(&junit, "motor_age_p99", summary.motor_age_p99,
                  "motor consumption-age evidence was unavailable or failed");
  AppendJUnitCase(&junit, "motor_age_maximum", summary.motor_age_maximum,
                  "motor maximum-age evidence was unavailable or failed");
  AppendJUnitCase(&junit, "independent_wire_capture",
                  summary.independent_wire_capture,
                  "requires an independent serial instrument");
  AppendJUnitCase(&junit, "complete_one_second_wire_windows",
                  summary.complete_one_second_wire_windows,
                  "requires aligned counters or an independent capture");
  AppendJUnitCase(&junit, "connected_telemetry_gaps",
                  summary.connected_telemetry_gaps,
                  "connected-window stream gaps were unavailable or failed");
  AppendJUnitCase(&junit, "physical_motor_response",
                  summary.physical_motor_response,
                  "requires guarded motor HIL");
  AppendJUnitCase(&junit, "physical_endpoint_recovery",
                  summary.physical_endpoint_recovery,
                  "physical availability is outside host observability");
  AppendJUnitCase(&junit, "usb_outage_rotation", summary.usb_outage_rotation,
                  "requires operator log or physical USB instrumentation");
  AppendJUnitCase(&junit, "stack_headroom", summary.stack_headroom,
                  "requires measured target task stacks");
  AppendJUnitCase(&junit, "allocation_trace", summary.allocation_trace,
                  "requires an external allocation trace");
  AppendJUnitCase(&junit, "button_stimulus", summary.button_stimulus,
                  "requires operator or fixture actuation");
  AppendJUnitCase(&junit, "imu_function", summary.imu_function,
                  "requires the characterized and reviewed IMU transform");
  AppendJUnitCase(&junit, "release_qualification",
                  CampaignMetricStatus::kNotObserved,
                  "this harness never declares D5 or release qualification");
  junit << "  </testsuite>\n</testsuites>\n";

  const std::array<std::pair<const char*, std::string>, 4U> payloads{
      std::pair<const char*, std::string>{"summary.json", json.str()},
      std::pair<const char*, std::string>{"metrics.csv", metrics.str()},
      std::pair<const char*, std::string>{"session-transitions.csv",
                                          transitions.str()},
      std::pair<const char*, std::string>{"junit.xml", junit.str()}};
  bool succeeded = true;
  for (const auto& payload : payloads) {
    if (!WriteFile(staging_directory / payload.first, payload.second, error)) {
      succeeded = false;
      break;
    }
  }

  std::array<std::string, kPayloadNames.size()> payload_digests{};
  std::array<std::uint64_t, kPayloadNames.size()> payload_sizes{};
  if (succeeded) {
    for (std::size_t index = 0U; index < kPayloadNames.size(); ++index) {
      if (!campaign_evidence_internal::Sha256File(
              staging_directory / kPayloadNames[index], &payload_digests[index],
              &payload_sizes[index], error)) {
        succeeded = false;
        break;
      }
    }
  }

  if (succeeded) {
    const std::string identity =
        campaign_evidence_internal::CanonicalCampaignIdentity(
            summary.mode, summary.profile, summary.configured_duration_ns);
    std::ostringstream manifest;
    manifest << "RRCLITE-QUALIFICATION-EVIDENCE-MANIFEST 1\n"
             << "campaign_mode " << CampaignModeName(summary.mode) << '\n'
             << "configured_duration_ns " << summary.configured_duration_ns
             << '\n'
             << "profile_mode " << CampaignModeName(summary.profile.mode)
             << '\n'
             << "profile_canonical_duration_ns "
             << summary.profile.canonical_duration_ns << '\n'
             << "profile_service_round_period_ns "
             << summary.profile.service_round_period_ns << '\n'
             << "profile_expected_cycles " << summary.profile.expected_cycles
             << '\n'
             << "profile_continuous_session_required "
             << (summary.profile.continuous_session_required ? 1 : 0) << '\n';
    for (std::size_t index = 0U; index < summary.profile.command_rates.size();
         ++index) {
      manifest << "profile_command_rate " << kCommandNames[index] << ' '
               << summary.profile.command_rates[index].numerator << ' '
               << summary.profile.command_rates[index].denominator_seconds
               << '\n';
    }
    manifest << "identity_sha256 "
             << campaign_evidence_internal::Sha256Hex(identity) << '\n';
    for (std::size_t index = 0U; index < kPayloadNames.size(); ++index) {
      manifest << "payload " << kPayloadNames[index] << ' '
               << payload_sizes[index] << ' ' << payload_digests[index] << '\n';
    }
    succeeded =
        WriteFile(staging_directory / kManifestName, manifest.str(), error);
  }
  if (succeeded) {
    succeeded = ApplyReadOnlyModes(staging_directory, error);
  }
  if (succeeded) {
    CampaignEvidenceExpectation expectation;
    expectation.mode = summary.mode;
    expectation.profile = summary.profile;
    expectation.configured_duration_ns = summary.configured_duration_ns;
    succeeded =
        VerifyCampaignEvidence(staging_directory.string(), expectation, error);
  }
  if (succeeded) {
    succeeded = campaign_evidence_internal::AtomicRenameDirectoryNoReplace(
        staging_directory, output_directory, error);
  }
  if (!succeeded) {
    CleanupStagingDirectory(staging_directory, error);
  }
  return succeeded;
}

}  // namespace mentor_pi_bringup
