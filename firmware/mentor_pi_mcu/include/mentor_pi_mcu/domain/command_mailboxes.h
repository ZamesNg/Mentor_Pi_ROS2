#ifndef MENTOR_PI_MCU_DOMAIN_COMMAND_MAILBOXES_H_
#define MENTOR_PI_MCU_DOMAIN_COMMAND_MAILBOXES_H_

#include <array>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/fixed_containers.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

struct MotorCommandSnapshot {
  std::array<float, kMotorCount> target_rps{};
  std::array<std::uint32_t, kMotorCount> accepted_at_us{};
  std::array<std::uint32_t, kMotorCount> field_generation{};
  std::uint32_t generation{0};
};

struct PwmCommandSnapshot {
  std::array<std::uint16_t, kPwmServoCount> pulse_width_us{1500U, 1500U, 1500U,
                                                           1500U};
  std::array<std::uint16_t, kPwmServoCount> duration_ms{};
  std::array<std::uint32_t, kPwmServoCount> field_generation{};
  std::uint32_t generation{0};
};

struct CommandAdmission {
  Result result{};
  bool overwrote_unread{false};
  std::uint32_t generation{0};
};

struct LedCommandSnapshot {
  std::array<LedCommand, kLedCount> commands{LedCommand{1U, 0U, 0U, 0U},
                                             LedCommand{2U, 0U, 0U, 0U},
                                             LedCommand{3U, 0U, 0U, 0U}};
  std::array<std::uint32_t, kLedCount> field_generation{};
  std::uint32_t generation{0};
};

struct BusMotionSnapshot {
  BusServoCommand command{};
  std::uint32_t generation{0};
};

struct BuzzerCommandSnapshot {
  BuzzerCommand command{};
  std::uint32_t generation{0};
};

struct RgbCommandSnapshot {
  RgbState state{};
  std::array<std::uint32_t, kRgbPixelCount> field_generation{};
  std::uint32_t generation{0};
};

struct OledCommandSnapshot {
  OledState state{};
  std::array<std::uint32_t, kOledHostLineCount> field_generation{};
  std::uint32_t generation{0};
};

class MotorCommandMailbox {
 public:
  CommandAdmission Publish(const MotorCommand& command, float max_rps,
                           std::uint32_t accepted_at_us);
  bool ConsumeLatest(MotorCommandSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  void ResetMergedFields();
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  MotorCommandSnapshot shadow_{};
  LatestMailbox<MotorCommandSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

class PwmCommandMailbox {
 public:
  CommandAdmission Publish(const PwmServoCommand& command);
  bool ConsumeLatest(PwmCommandSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  void ResetMergedFields();
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  PwmCommandSnapshot shadow_{};
  LatestMailbox<PwmCommandSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

class BusMotionMailbox {
 public:
  CommandAdmission Publish(const BusServoCommand& command);
  bool ConsumeLatest(BusMotionSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  BusMotionSnapshot shadow_{};
  LatestMailbox<BusMotionSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

class LedCommandMailbox {
 public:
  CommandAdmission Publish(const LedCommand& command);
  bool ConsumeLatest(LedCommandSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  void ResetMergedFields();
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  LedCommandSnapshot shadow_{};
  LatestMailbox<LedCommandSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

class BuzzerCommandMailbox {
 public:
  CommandAdmission Publish(const BuzzerCommand& command);
  bool ConsumeLatest(BuzzerCommandSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  BuzzerCommandSnapshot shadow_{};
  LatestMailbox<BuzzerCommandSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

class RgbCommandMailbox {
 public:
  CommandAdmission Publish(const RgbCommand& command);
  bool ConsumeLatest(RgbCommandSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  void ResetMergedFields();
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  RgbCommandSnapshot shadow_{};
  LatestMailbox<RgbCommandSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

class OledCommandMailbox {
 public:
  CommandAdmission Publish(const OledCommand& command);
  bool ConsumeLatest(OledCommandSnapshot* snapshot) {
    return mailbox_.ConsumeLatest(snapshot);
  }
  void ResetMergedFields();
  std::uint32_t overwrite_count() const { return overwrite_count_.value(); }

 private:
  OledCommandSnapshot shadow_{};
  LatestMailbox<OledCommandSnapshot> mailbox_{};
  SaturatingCounter<std::uint32_t> overwrite_count_{};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_COMMAND_MAILBOXES_H_
