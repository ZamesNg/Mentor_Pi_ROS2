#ifndef MENTOR_PI_MCU_DOMAIN_BUS_SERVO_H_
#define MENTOR_PI_MCU_DOMAIN_BUS_SERVO_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/fixed_containers.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

constexpr std::uint8_t kBusServoFrameHeader = 0x55U;
constexpr std::size_t kBusServoMaximumArguments = 8U;
constexpr std::size_t kBusServoMaximumFrameBytes =
    kBusServoMaximumArguments + 6U;

enum class BusServoOpcode : std::uint8_t {
  kMoveTimeWrite = 1,
  kMoveTimeRead = 2,
  kMoveStop = 12,
  kIdWrite = 13,
  kIdRead = 14,
  kOffsetAdjust = 17,
  kOffsetSave = 18,
  kOffsetRead = 19,
  kPositionLimitsWrite = 20,
  kPositionLimitsRead = 21,
  kVoltageLimitsWrite = 22,
  kVoltageLimitsRead = 23,
  kTemperatureLimitWrite = 24,
  kTemperatureLimitRead = 25,
  kTemperatureRead = 26,
  kVoltageRead = 27,
  kPositionRead = 28,
  kTorqueWrite = 31,
  kTorqueRead = 32,
};

struct BusServoFrame {
  std::array<std::uint8_t, kBusServoMaximumFrameBytes> bytes{};
  std::uint8_t size{0};
};

struct ParsedBusServoFrame {
  Result result{};
  std::uint8_t servo_id{0};
  BusServoOpcode opcode{BusServoOpcode::kMoveTimeWrite};
  std::array<std::uint8_t, kBusServoMaximumArguments> arguments{};
  std::uint8_t argument_count{0};
};

class BusServoCodec {
 public:
  static Result BuildFrame(std::uint8_t servo_id, BusServoOpcode opcode,
                           const std::uint8_t* arguments,
                           std::size_t argument_count, BusServoFrame* frame);
  static ParsedBusServoFrame ParseFrame(const std::uint8_t* bytes,
                                        std::size_t size);
  static std::uint8_t Checksum(const std::uint8_t* id_through_arguments,
                               std::size_t size);
};

enum class ScheduledBusFrameKind : std::uint8_t {
  kNone = 0,
  kMove = 1,
  kStop = 2,
};

struct ScheduledBusFrame {
  Result result{};
  BusServoFrame frame{};
  ScheduledBusFrameKind kind{ScheduledBusFrameKind::kNone};
  std::uint32_t generation{0};
  std::uint8_t batch_index{0};
};

struct BusMoveAdmission {
  Result result{};
  std::uint32_t generation{0};
  bool overwrote_pending{false};
};

// Hardware-independent move/stop ordering. The UART owner calls BeginFrame,
// transmits exactly one complete frame, then calls CompleteFrame. An accepted
// stop never truncates a frame and invalidates all unsent pre-stop movement.
class BusServoScheduler {
 public:
  BusMoveAdmission SubmitMove(const BusServoCommand& command);
  Result AcceptStop(const StopBusServosCommand& command);
  ScheduledBusFrame BeginFrame();
  void CompleteFrame(bool transmitted);
  void CancelAll();

  bool frame_in_progress() const { return frame_in_progress_; }
  bool has_work() const;
  std::uint32_t current_generation() const { return generation_; }
  std::uint32_t stop_watermark() const { return stop_watermark_; }
  std::uint32_t move_overwrite_count() const {
    return move_overwrite_count_.value();
  }

 private:
  struct MoveSlot {
    BusServoCommand command{};
    std::uint32_t generation{0};
    std::uint8_t next_index{0};
    bool valid{false};
  };

  static std::uint32_t NextGeneration(std::uint32_t generation);
  ScheduledBusFrame BuildMoveFrame();
  ScheduledBusFrame BuildStopFrame();

  MoveSlot pending_move_{};
  MoveSlot active_move_{};
  StopBusServosCommand stop_command_{};
  std::uint32_t generation_{0};
  std::uint32_t stop_watermark_{0};
  std::uint8_t stop_next_index_{0};
  ScheduledBusFrameKind in_progress_kind_{ScheduledBusFrameKind::kNone};
  bool stop_pending_{false};
  bool frame_in_progress_{false};
  bool cancel_active_after_frame_{false};
  SaturatingCounter<std::uint32_t> move_overwrite_count_{};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_BUS_SERVO_H_
