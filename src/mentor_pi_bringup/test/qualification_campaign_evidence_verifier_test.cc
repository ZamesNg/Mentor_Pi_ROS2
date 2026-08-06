// Copyright 2026 Mentor Pi Maintainers
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <locale>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>

#include "mentor_pi_bringup/qualification_campaign_evidence.h"
#include "mentor_pi_bringup/qualification_campaign_evidence_internal.h"

namespace {

using mentor_pi_bringup::CampaignEvidenceExpectation;
using mentor_pi_bringup::CampaignEvidenceMetadata;
using mentor_pi_bringup::CampaignMode;
using mentor_pi_bringup::CampaignProfileForMode;
using mentor_pi_bringup::CampaignSummary;
using mentor_pi_bringup::VerifyCampaignEvidence;
using mentor_pi_bringup::WriteCampaignEvidence;
using mentor_pi_bringup::campaign_evidence_internal::
    AtomicRenameDirectoryNoReplace;
using mentor_pi_bringup::campaign_evidence_internal::
    CanonicalSummaryIdentityJson;
using mentor_pi_bringup::campaign_evidence_internal::Sha256Hex;

int g_failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++g_failures;
  }
}

std::filesystem::path UniqueTemporaryRoot(const std::string& label) {
  static std::atomic<std::uint64_t> sequence{0U};
  const auto time = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() /
      ("rrclite-evidence-test-" + label + '-' + std::to_string(time) + '-' +
       std::to_string(sequence.fetch_add(1U, std::memory_order_relaxed)));
  std::error_code error;
  std::filesystem::create_directory(path, error);
  Expect(!error, "temporary test root is created: " + error.message());
  return path;
}

void MakeDirectoryWritable(const std::filesystem::path& directory) {
  std::error_code error;
  std::filesystem::permissions(directory,
                               std::filesystem::perms::owner_read |
                                   std::filesystem::perms::owner_write |
                                   std::filesystem::perms::owner_exec,
                               std::filesystem::perm_options::add, error);
  Expect(!error, "test can make directory writable: " + error.message());
}

void MakeFileWritable(const std::filesystem::path& path) {
  std::error_code error;
  std::filesystem::permissions(path, std::filesystem::perms::owner_write,
                               std::filesystem::perm_options::add, error);
  Expect(!error, "test can make file writable: " + error.message());
}

void RestoreReadOnlyModes(const std::filesystem::path& bundle) {
  constexpr std::filesystem::perms kFilePermissions =
      std::filesystem::perms::owner_read | std::filesystem::perms::group_read |
      std::filesystem::perms::others_read;
  constexpr std::filesystem::perms kDirectoryPermissions =
      kFilePermissions | std::filesystem::perms::owner_exec |
      std::filesystem::perms::group_exec | std::filesystem::perms::others_exec;
  std::error_code error;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(bundle)) {
    if (!entry.is_symlink()) {
      std::filesystem::permissions(entry.path(), kFilePermissions,
                                   std::filesystem::perm_options::replace,
                                   error);
      Expect(!error, "test restores file read-only mode: " + error.message());
      error.clear();
    }
  }
  std::filesystem::permissions(bundle, kDirectoryPermissions,
                               std::filesystem::perm_options::replace, error);
  Expect(!error, "test restores directory read-only mode: " + error.message());
}

void RemoveTree(const std::filesystem::path& root) {
  if (!std::filesystem::exists(root) && !std::filesystem::is_symlink(root)) {
    return;
  }
  std::error_code error;
  for (std::filesystem::recursive_directory_iterator iterator(
           root, std::filesystem::directory_options::skip_permission_denied,
           error),
       end;
       !error && iterator != end; ++iterator) {
    const std::filesystem::file_status status = iterator->symlink_status(error);
    if (error) {
      break;
    }
    if (std::filesystem::is_directory(status) &&
        !std::filesystem::is_symlink(status)) {
      MakeDirectoryWritable(iterator->path());
    }
  }
  MakeDirectoryWritable(root);
  error.clear();
  std::filesystem::remove_all(root, error);
  Expect(!error, "temporary test tree is removed: " + error.message());
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::in | std::ios::binary);
  std::ostringstream contents;
  contents << input.rdbuf();
  return contents.str();
}

void WriteFile(const std::filesystem::path& path, const std::string& contents) {
  std::ofstream output(path,
                       std::ios::out | std::ios::binary | std::ios::trunc);
  output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  output.close();
  Expect(static_cast<bool>(output), "test mutation writes successfully");
}

bool ReplaceOnce(std::string* contents, const std::string& from,
                 const std::string& to) {
  const std::size_t position = contents->find(from);
  if (position == std::string::npos ||
      contents->find(from, position + from.size()) != std::string::npos) {
    return false;
  }
  contents->replace(position, from.size(), to);
  return true;
}

struct Fixture {
  std::filesystem::path root;
  std::filesystem::path bundle;
  CampaignEvidenceMetadata metadata;
  CampaignSummary summary;
  CampaignEvidenceExpectation expectation;
};

Fixture MakeFixture(const std::string& label) {
  Fixture fixture;
  fixture.root = UniqueTemporaryRoot(label);
  fixture.bundle = fixture.root / "evidence";
  fixture.summary.mode = CampaignMode::kLoad500;
  fixture.summary.profile = CampaignProfileForMode(CampaignMode::kLoad500);
  fixture.summary.configured_duration_ns = INT64_C(2000000000);
  fixture.summary.canonical_profile = false;
  fixture.summary.execution_passed = true;
  fixture.metadata.run_id = "evidence-test-run";
  fixture.metadata.source_revision = "source-r4";
  fixture.metadata.firmware_sha256 =
      "963f2834a08b9e86dbe736e58cfee83a2378983cdcaf2f1bc1a3ea0257136e8f";
  fixture.metadata.host_revision = "host-r4";
  fixture.metadata.ros_distribution = "humble";
  fixture.metadata.board_serial = "RRCLITE-TEST-BOARD";
  fixture.metadata.fixture_revision = "fixture-r2";
  fixture.metadata.campaign_mode = "load500";
  fixture.metadata.start_time_utc = "2026-08-06T00:00:00Z";
  fixture.metadata.finish_time_utc = "2026-08-06T00:00:02Z";
  fixture.expectation.mode = fixture.summary.mode;
  fixture.expectation.profile = fixture.summary.profile;
  fixture.expectation.configured_duration_ns =
      fixture.summary.configured_duration_ns;
  return fixture;
}

void Publish(Fixture* fixture) {
  std::string error;
  Expect(WriteCampaignEvidence(fixture->bundle.string(), fixture->metadata,
                               fixture->summary, &error),
         "valid evidence publishes: " + error);
}

void RewriteManifest(Fixture* fixture, const std::string& contents) {
  MakeDirectoryWritable(fixture->bundle);
  MakeFileWritable(fixture->bundle / "manifest.sha256");
  WriteFile(fixture->bundle / "manifest.sha256", contents);
  RestoreReadOnlyModes(fixture->bundle);
}

bool RewriteSummaryAndManifest(Fixture* fixture,
                               const std::string& summary_contents) {
  std::string manifest = ReadFile(fixture->bundle / "manifest.sha256");
  const std::string prefix = "payload summary.json ";
  const std::size_t entry = manifest.find(prefix);
  const std::size_t line_end = manifest.find('\n', entry);
  if (entry == std::string::npos || line_end == std::string::npos ||
      manifest.find(prefix, entry + prefix.size()) != std::string::npos) {
    return false;
  }
  const std::string replacement = prefix +
                                  std::to_string(summary_contents.size()) +
                                  ' ' + Sha256Hex(summary_contents);
  manifest.replace(entry, line_end - entry, replacement);

  MakeDirectoryWritable(fixture->bundle);
  MakeFileWritable(fixture->bundle / "summary.json");
  MakeFileWritable(fixture->bundle / "manifest.sha256");
  WriteFile(fixture->bundle / "summary.json", summary_contents);
  WriteFile(fixture->bundle / "manifest.sha256", manifest);
  RestoreReadOnlyModes(fixture->bundle);
  return true;
}

bool SwapRateEntries(std::string* contents, const std::string& first_name,
                     const std::string& second_name) {
  const std::string first_token = R"("name": ")" + first_name + '"';
  const std::string second_token = R"("name": ")" + second_name + '"';
  const std::size_t first_token_position = contents->find(first_token);
  const std::size_t second_token_position = contents->find(second_token);
  if (first_token_position == std::string::npos ||
      second_token_position == std::string::npos ||
      first_token_position >= second_token_position) {
    return false;
  }
  const std::size_t first_start = contents->rfind('\n', first_token_position);
  const std::size_t first_end = contents->find('\n', first_token_position);
  const std::size_t second_start = contents->rfind('\n', second_token_position);
  const std::size_t second_end = contents->find('\n', second_token_position);
  if (first_start == std::string::npos || first_end == std::string::npos ||
      second_start == std::string::npos || second_end == std::string::npos) {
    return false;
  }
  const std::string first_line =
      contents->substr(first_start + 1U, first_end - first_start);
  const std::string second_line =
      contents->substr(second_start + 1U, second_end - second_start);
  contents->replace(second_start + 1U, second_end - second_start, first_line);
  contents->replace(first_start + 1U, first_end - first_start, second_line);
  return true;
}

void ExpectSummaryMutationRejected(const std::string& label,
                                   const std::string& contents) {
  Fixture fixture = MakeFixture(label);
  Publish(&fixture);
  Expect(RewriteSummaryAndManifest(&fixture, contents),
         "test rewrites summary and recomputes its manifest entry");
  std::string error;
  Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                 &error),
         "strict summary parser rejects " + label);
  RemoveTree(fixture.root);
}

class CommaDecimalPoint final : public std::numpunct<char> {
 protected:
  char do_decimal_point() const override { return ','; }
};

void TestSha256KnownAnswers() {
  Expect(Sha256Hex("") ==
             "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855",
         "SHA-256 empty-string known answer");
  Expect(Sha256Hex("abc") ==
             "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
         "SHA-256 abc known answer");
  const std::string million_a(1000000U, 'a');
  Expect(Sha256Hex(million_a) ==
             "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0",
         "SHA-256 million-a multi-block known answer");
}

void TestValidBundleAndExpectedIdentity() {
  Fixture fixture = MakeFixture("valid");
  fixture.summary.stored_transition_count = 1U;
  fixture.summary.transitions[0U].cycle = 1U;
  fixture.summary.transitions[0U].reset_reason = 3U;
  Publish(&fixture);
  std::string error;
  Expect(VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                &error),
         "published bundle verifies: " + error);
  const std::string transitions =
      ReadFile(fixture.bundle / "session-transitions.csv");
  Expect(transitions.find("mcu_uptime_regression_observed,reset_reason\n") !=
                 std::string::npos &&
             transitions.size() >= 3U &&
             transitions.substr(transitions.size() - 3U) == ",3\n",
         "transition evidence records the accepted reset reason");

  CampaignEvidenceExpectation wrong_mode;
  wrong_mode.mode = CampaignMode::kSoak;
  wrong_mode.profile = CampaignProfileForMode(CampaignMode::kSoak);
  wrong_mode.configured_duration_ns = fixture.summary.configured_duration_ns;
  error.clear();
  Expect(!VerifyCampaignEvidence(fixture.bundle.string(), wrong_mode, &error),
         "verifier rejects an expected-mode mismatch");

  CampaignEvidenceExpectation wrong_profile = fixture.expectation;
  ++wrong_profile.profile.command_rates[0U].numerator;
  error.clear();
  Expect(
      !VerifyCampaignEvidence(fixture.bundle.string(), wrong_profile, &error),
      "verifier rejects an expected exact-rational-profile mismatch");

  CampaignEvidenceExpectation wrong_duration = fixture.expectation;
  ++wrong_duration.configured_duration_ns;
  error.clear();
  Expect(
      !VerifyCampaignEvidence(fixture.bundle.string(), wrong_duration, &error),
      "verifier rejects an expected-duration mismatch");
  RemoveTree(fixture.root);
}

void TestWriterRejectsIdentityMismatchWithoutStagingLeak() {
  Fixture fixture = MakeFixture("writer-identity");
  fixture.metadata.campaign_mode = "soak";
  std::string error;
  Expect(!WriteCampaignEvidence(fixture.bundle.string(), fixture.metadata,
                                fixture.summary, &error),
         "writer rejects metadata campaign_mode inconsistent with Summary");
  Expect(!std::filesystem::exists(fixture.bundle),
         "identity rejection does not publish a destination");
  bool staging_seen = false;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(fixture.root)) {
    staging_seen = staging_seen || entry.path().filename().string().find(
                                       ".rrclite-evidence-staging-") == 0U;
  }
  Expect(!staging_seen, "identity rejection leaves no sibling staging tree");
  RemoveTree(fixture.root);
}

void TestWriterSelfVerificationFailureCleansStaging() {
  Fixture fixture = MakeFixture("writer-self-verification");
  fixture.summary.maximum_transport_interval_bytes_per_second = 12.5;
  const std::locale original_locale;
  std::locale::global(
      std::locale(std::locale::classic(), new CommaDecimalPoint));
  std::string error;
  const bool published = WriteCampaignEvidence(
      fixture.bundle.string(), fixture.metadata, fixture.summary, &error);
  std::locale::global(original_locale);
  Expect(!published,
         "writer refuses staging whose locale-mutated JSON fails verification");
  Expect(!std::filesystem::exists(fixture.bundle),
         "failed self-verification never publishes a destination");
  bool staging_seen = false;
  for (const std::filesystem::directory_entry& entry :
       std::filesystem::directory_iterator(fixture.root)) {
    staging_seen = staging_seen || entry.path().filename().string().find(
                                       ".rrclite-evidence-staging-") == 0U;
  }
  Expect(!staging_seen,
         "failed self-verification removes the exact sibling staging tree");
  RemoveTree(fixture.root);
}

void TestSummaryJsonStructuralAttacks() {
  Fixture seed = MakeFixture("summary-attack-seed");
  Publish(&seed);
  const std::string valid_summary = ReadFile(seed.bundle / "summary.json");
  const std::string canonical_identity = CanonicalSummaryIdentityJson(
      seed.expectation.mode, seed.expectation.profile,
      seed.expectation.configured_duration_ns);
  RemoveTree(seed.root);

  ExpectSummaryMutationRejected("malformed-prefix",
                                "garbage\n" + valid_summary);
  ExpectSummaryMutationRejected("malformed-suffix", valid_summary + "{}\n");

  std::string appended_identity = valid_summary;
  Expect(ReplaceOnce(&appended_identity, "  \"campaign_identity\": {",
                     "  \"untrusted_identity\": {"),
         "test renames the real identity object");
  appended_identity += canonical_identity;
  ExpectSummaryMutationRejected("appended-canonical-identity",
                                appended_identity);

  std::string duplicate_identity = valid_summary;
  const std::size_t root_close = duplicate_identity.rfind("\n}\n");
  Expect(root_close != std::string::npos,
         "test locates the summary root closing brace");
  if (root_close != std::string::npos) {
    duplicate_identity.insert(
        root_close, ",\n  \"campaign_identity\": {\"mode\": \"soak\"}");
  }
  ExpectSummaryMutationRejected("duplicate-top-level-identity",
                                duplicate_identity);

  std::string duplicate_identity_key = valid_summary;
  Expect(ReplaceOnce(&duplicate_identity_key, "    \"mode\": \"load500\",\n",
                     "    \"mode\": \"load500\",\n"
                     "    \"mode\": \"load500\",\n"),
         "test duplicates a campaign identity key");
  ExpectSummaryMutationRejected("duplicate-identity-key",
                                duplicate_identity_key);

  std::string wrong_integer_type = valid_summary;
  Expect(ReplaceOnce(&wrong_integer_type,
                     "    \"configured_duration_ns\": 2000000000,\n",
                     "    \"configured_duration_ns\": \"2000000000\",\n"),
         "test changes an identity integer to a string");
  ExpectSummaryMutationRejected("wrong-identity-field-type",
                                wrong_integer_type);

  std::string reordered_rates = valid_summary;
  Expect(SwapRateEntries(&reordered_rates, "motor", "pwm_servo"),
         "test swaps two named rate-array entries");
  ExpectSummaryMutationRejected("reordered-command-rates", reordered_rates);
}

void TestPayloadTamperAndClosedFileSet() {
  {
    Fixture fixture = MakeFixture("payload-tamper");
    Publish(&fixture);
    MakeDirectoryWritable(fixture.bundle);
    MakeFileWritable(fixture.bundle / "metrics.csv");
    WriteFile(fixture.bundle / "metrics.csv",
              ReadFile(fixture.bundle / "metrics.csv") + "tampered\n");
    RestoreReadOnlyModes(fixture.bundle);
    std::string error;
    Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                   &error),
           "verifier rejects a tampered payload");
    RemoveTree(fixture.root);
  }
  {
    Fixture fixture = MakeFixture("extra-file");
    Publish(&fixture);
    MakeDirectoryWritable(fixture.bundle);
    WriteFile(fixture.bundle / "extra.txt", "unexpected\n");
    RestoreReadOnlyModes(fixture.bundle);
    std::string error;
    Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                   &error),
           "verifier rejects an extra file");
    RemoveTree(fixture.root);
  }
  {
    Fixture fixture = MakeFixture("missing-file");
    Publish(&fixture);
    MakeDirectoryWritable(fixture.bundle);
    std::error_code remove_error;
    std::filesystem::remove(fixture.bundle / "junit.xml", remove_error);
    Expect(!remove_error, "test removes one required payload");
    RestoreReadOnlyModes(fixture.bundle);
    std::string error;
    Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                   &error),
           "verifier rejects a missing file");
    RemoveTree(fixture.root);
  }
  {
    Fixture fixture = MakeFixture("symlink-file");
    Publish(&fixture);
    WriteFile(fixture.root / "outside-metrics", "outside\n");
    MakeDirectoryWritable(fixture.bundle);
    std::error_code link_error;
    std::filesystem::remove(fixture.bundle / "metrics.csv", link_error);
    Expect(!link_error, "test removes payload before symlink substitution");
    std::filesystem::create_symlink(fixture.root / "outside-metrics",
                                    fixture.bundle / "metrics.csv", link_error);
    Expect(!link_error, "test substitutes a required payload symlink");
    RestoreReadOnlyModes(fixture.bundle);
    std::string error;
    Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                   &error),
           "verifier rejects a payload symlink");
    RemoveTree(fixture.root);
  }
}

void TestManifestGrammarAttacks() {
  const std::array<std::pair<std::string, std::string>, 5U> replacements{
      std::pair<std::string, std::string>{"campaign_mode load500\n",
                                          "campaign_mode load500\n"
                                          "campaign_mode load500\n"},
      std::pair<std::string, std::string>{"payload summary.json ",
                                          "payload ../summary.json "},
      std::pair<std::string, std::string>{"payload summary.json ",
                                          "payload /tmp/summary.json "},
      std::pair<std::string, std::string>{"payload summary.json ",
                                          "payload nested/summary.json "},
      std::pair<std::string, std::string>{"identity_sha256 ",
                                          "unknown_identity_field "}};
  std::size_t case_index = 0U;
  for (const auto& replacement : replacements) {
    Fixture fixture =
        MakeFixture("manifest-attack-" + std::to_string(case_index));
    Publish(&fixture);
    std::string manifest = ReadFile(fixture.bundle / "manifest.sha256");
    Expect(ReplaceOnce(&manifest, replacement.first, replacement.second),
           "test locates one manifest token to mutate");
    RewriteManifest(&fixture, manifest);
    std::string error;
    Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                   &error),
           "verifier rejects malformed/duplicate/non-canonical manifest case " +
               std::to_string(case_index));
    RemoveTree(fixture.root);
    ++case_index;
  }

  Fixture duplicate_payload = MakeFixture("duplicate-payload");
  Publish(&duplicate_payload);
  std::string manifest = ReadFile(duplicate_payload.bundle / "manifest.sha256");
  const std::size_t payload_position = manifest.find("payload summary.json ");
  const std::size_t payload_end = manifest.find('\n', payload_position);
  Expect(
      payload_position != std::string::npos && payload_end != std::string::npos,
      "test locates payload entry for duplication");
  manifest +=
      manifest.substr(payload_position, payload_end - payload_position + 1U);
  RewriteManifest(&duplicate_payload, manifest);
  std::string error;
  Expect(!VerifyCampaignEvidence(duplicate_payload.bundle.string(),
                                 duplicate_payload.expectation, &error),
         "verifier rejects a duplicate payload manifest entry");
  RemoveTree(duplicate_payload.root);
}

void TestDirectorySymlinkAndWritableModes() {
  Fixture fixture = MakeFixture("directory-symlink");
  Publish(&fixture);
  const std::filesystem::path alias = fixture.root / "alias";
  std::error_code link_error;
  std::filesystem::create_directory_symlink(fixture.bundle, alias, link_error);
  Expect(!link_error, "test creates a directory symlink");
  std::string error;
  Expect(!VerifyCampaignEvidence(alias.string(), fixture.expectation, &error),
         "verifier rejects an evidence-directory symlink");

  MakeDirectoryWritable(fixture.bundle);
  error.clear();
  Expect(!VerifyCampaignEvidence(fixture.bundle.string(), fixture.expectation,
                                 &error),
         "verifier rejects a writable evidence directory");
  RestoreReadOnlyModes(fixture.bundle);
  RemoveTree(fixture.root);
}

void TestAtomicNoReplaceAndDestinationRace() {
  const std::filesystem::path root = UniqueTemporaryRoot("atomic-no-replace");
  const std::filesystem::path source = root / "staging";
  const std::filesystem::path destination = root / "evidence";
  std::filesystem::create_directory(source);
  WriteFile(source / "source-marker", "source\n");

  std::atomic<bool> create_destination{false};
  std::atomic<bool> destination_created{false};
  std::thread competing_creator([&]() {
    while (!create_destination.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    std::error_code create_error;
    std::filesystem::create_directory(destination, create_error);
    if (!create_error) {
      WriteFile(destination / "destination-marker", "destination\n");
      destination_created.store(true, std::memory_order_release);
    }
  });
  create_destination.store(true, std::memory_order_release);
  while (!destination_created.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  std::string error;
  Expect(!AtomicRenameDirectoryNoReplace(source, destination, &error),
         "atomic publication rejects a concurrently appearing destination");
  competing_creator.join();
  Expect(std::filesystem::exists(source / "source-marker"),
         "failed no-replace publication preserves the exact source staging");
  Expect(std::filesystem::exists(destination / "destination-marker"),
         "failed no-replace publication never replaces the destination");
  RemoveTree(root);
}

void TestWriterCleansExactStagingAfterPublicationRace() {
  bool race_observed = false;
  for (std::uint32_t attempt = 0U; attempt < 32U && !race_observed; ++attempt) {
    Fixture fixture = MakeFixture("writer-race-" + std::to_string(attempt));
    std::atomic<bool> watcher_ready{false};
    std::atomic<bool> stop_watcher{false};
    std::atomic<bool> destination_created{false};
    std::thread competing_creator([&]() {
      watcher_ready.store(true, std::memory_order_release);
      while (!stop_watcher.load(std::memory_order_acquire)) {
        std::error_code iteration_error;
        std::filesystem::directory_iterator iterator(fixture.root,
                                                     iteration_error);
        const std::filesystem::directory_iterator end;
        while (!iteration_error && iterator != end) {
          const std::string name = iterator->path().filename().string();
          if (name.find(".rrclite-evidence-staging-") == 0U) {
            std::error_code create_error;
            if (std::filesystem::create_directory(fixture.bundle,
                                                  create_error)) {
              destination_created.store(true, std::memory_order_release);
            }
            return;
          }
          iterator.increment(iteration_error);
        }
        std::this_thread::yield();
      }
    });
    while (!watcher_ready.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    std::string error;
    const bool published = WriteCampaignEvidence(
        fixture.bundle.string(), fixture.metadata, fixture.summary, &error);
    stop_watcher.store(true, std::memory_order_release);
    competing_creator.join();
    race_observed = destination_created.load(std::memory_order_acquire);
    if (race_observed) {
      Expect(!published,
             "writer rejects a destination that appears during publication");
      bool staging_seen = false;
      for (const std::filesystem::directory_entry& entry :
           std::filesystem::directory_iterator(fixture.root)) {
        staging_seen = staging_seen || entry.path().filename().string().find(
                                           ".rrclite-evidence-staging-") == 0U;
      }
      Expect(!staging_seen,
             "failed publication removes only its exact sibling staging tree");
    }
    RemoveTree(fixture.root);
  }
  Expect(race_observed,
         "concurrent test observes a destination appearing after staging");
}

}  // namespace

int main() {
  try {
    TestSha256KnownAnswers();
    TestValidBundleAndExpectedIdentity();
    TestWriterRejectsIdentityMismatchWithoutStagingLeak();
    TestWriterSelfVerificationFailureCleansStaging();
    TestSummaryJsonStructuralAttacks();
    TestPayloadTamperAndClosedFileSet();
    TestManifestGrammarAttacks();
    TestDirectorySymlinkAndWritableModes();
    TestAtomicNoReplaceAndDestinationRace();
    TestWriterCleansExactStagingAfterPublicationRace();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: unexpected exception: " << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "FAIL: unexpected non-standard exception\n";
    return 1;
  }
  if (g_failures == 0) {
    std::cout << "qualification evidence verifier tests passed\n";
  }
  return g_failures == 0 ? 0 : 1;
}
