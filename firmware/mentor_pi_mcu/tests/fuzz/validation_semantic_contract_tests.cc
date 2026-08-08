// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

#include "fuzz_input.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/domain/validation.h"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data,
                                      std::size_t size);

namespace mentor_pi::mcu {
namespace {

std::uint32_t g_test_failures = 0U;

void Check(bool condition, const char* expression, const char* file, int line) {
  if (!condition) {
    ++g_test_failures;
    std::cerr << file << ':' << line << ": check failed: " << expression
              << '\n';
  }
}

#define CHECK(expression) \
  Check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

// The offsets below are the byte contract consumed independently by each
// Check* function in validation_oracle_fuzz.cc. Every buffer remains far below
// that harness's 512-byte maximum input.
constexpr std::size_t kInputCapacity = 96U;
constexpr std::size_t kPwmOffsetMaskOffset = 11U;
constexpr std::size_t kPwmOffsetsOffset = 12U;
constexpr std::size_t kMoveIdsOffset = 1U;
constexpr std::size_t kMovePositionsOffset = 17U;
constexpr std::size_t kMoveDurationOffset = 49U;
constexpr std::size_t kStopCountOffset = 51U;
constexpr std::size_t kStopIdsOffset = 52U;
constexpr std::size_t kConfigureIdOffset = 68U;
constexpr std::size_t kConfigureMaskOffset = 69U;
constexpr std::size_t kConfigureNewIdOffset = 71U;
constexpr std::size_t kConfigureOffsetOffset = 72U;
constexpr std::size_t kConfigurePositionMinOffset = 73U;
constexpr std::size_t kConfigurePositionMaxOffset = 75U;
constexpr std::size_t kConfigureVoltageMinOffset = 77U;
constexpr std::size_t kConfigureVoltageMaxOffset = 79U;
constexpr std::size_t kConfigureTemperatureOffset = 81U;
constexpr std::size_t kConfigureTorqueOffset = 82U;
constexpr std::size_t kGetIdOffset = 83U;
constexpr std::size_t kGetFieldsOffset = 84U;
constexpr std::size_t kRgbMaskOffset = 15U;
constexpr std::size_t kOledMaskOffset = 22U;
constexpr std::size_t kOledLine1SizeOffset = 23U;
constexpr std::size_t kOledLine1BytesOffset = 24U;
constexpr std::size_t kOledLine2SizeOffset = 48U;
constexpr std::size_t kOledLine2BytesOffset = 49U;
constexpr std::size_t kBatteryThresholdOffset = 73U;
constexpr std::size_t kMotorModelOffset = 75U;

class InputBuffer {
 public:
  void SetU8(std::size_t offset, std::uint8_t value) {
    CHECK(offset < bytes_.size());
    if (offset >= bytes_.size()) {
      return;
    }
    bytes_[offset] = value;
    size_ = std::max(size_, offset + 1U);
  }

  void SetU16(std::size_t offset, std::uint16_t value) {
    SetU8(offset, static_cast<std::uint8_t>(value & 0xffU));
    SetU8(offset + 1U, static_cast<std::uint8_t>(value >> 8U));
  }

  void SetI16(std::size_t offset, std::int16_t value) {
    std::uint16_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    SetU16(offset, bits);
  }

  const std::uint8_t* data() const { return bytes_.data(); }
  std::size_t size() const { return size_; }

 private:
  std::array<std::uint8_t, kInputCapacity> bytes_{};
  std::size_t size_{0U};
};

namespace input_type {
constexpr std::uint32_t kMotor = 1U << 0U;
constexpr std::uint32_t kPwm = 1U << 1U;
constexpr std::uint32_t kBusMove = 1U << 2U;
constexpr std::uint32_t kLed = 1U << 3U;
constexpr std::uint32_t kBuzzer = 1U << 4U;
constexpr std::uint32_t kRgb = 1U << 5U;
constexpr std::uint32_t kOled = 1U << 6U;
constexpr std::uint32_t kMotorModel = 1U << 7U;
constexpr std::uint32_t kPwmOffsets = 1U << 8U;
constexpr std::uint32_t kBusState = 1U << 9U;
constexpr std::uint32_t kBusConfigure = 1U << 10U;
constexpr std::uint32_t kBusStop = 1U << 11U;
constexpr std::uint32_t kBatteryThreshold = 1U << 12U;
constexpr std::uint32_t kAll = (1U << 13U) - 1U;
constexpr std::uint32_t kMaskBearing =
    kMotor | kPwm | kRgb | kOled | kPwmOffsets | kBusState | kBusConfigure;
}  // namespace input_type

namespace feature {
constexpr std::uint32_t kMaximumCount = 1U << 0U;
constexpr std::uint32_t kMalformedMask = 1U << 1U;
constexpr std::uint32_t kMalformedText = 1U << 2U;
constexpr std::uint32_t kDuplicateId = 1U << 3U;
constexpr std::uint32_t kEveryEnumByte = 1U << 4U;
constexpr std::uint32_t kNanInf = 1U << 5U;
constexpr std::uint32_t kSignedBoundary = 1U << 6U;
constexpr std::uint32_t kUnsignedBoundary = 1U << 7U;
constexpr std::uint32_t kAll = (1U << 8U) - 1U;
}  // namespace feature

struct Evidence {
  std::uint32_t input_types{0U};
  std::uint32_t boundary_types{0U};
  std::uint32_t malformed_mask_types{0U};
  std::uint32_t features{0U};
  std::uint32_t oracle_inputs{0U};
};

void ExerciseOracle(const InputBuffer& input, std::uint32_t input_types,
                    std::uint32_t boundary_types, std::uint32_t mask_types,
                    std::uint32_t features, Evidence* evidence) {
  CHECK(evidence != nullptr);
  CHECK(input.size() <= 512U);
  CHECK(LLVMFuzzerTestOneInput(input.data(), input.size()) == 0);
  if (evidence == nullptr) {
    return;
  }
  evidence->input_types |= input_types;
  evidence->boundary_types |= boundary_types;
  evidence->malformed_mask_types |= mask_types;
  evidence->features |= features;
  ++evidence->oracle_inputs;
}

InputBuffer ValidPwmInput() {
  InputBuffer input;
  input.SetU8(0U, kAllPwmServoMask);
  input.SetU16(1U, 20U);
  for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
    input.SetU16(3U + (index * 2U), 1500U);
    input.SetI16(kPwmOffsetsOffset + (index * 2U), 0);
  }
  input.SetU8(kPwmOffsetMaskOffset, kAllPwmServoMask);
  return input;
}

InputBuffer ValidBusInput() {
  InputBuffer input;
  input.SetU8(0U, 1U);
  input.SetU8(kMoveIdsOffset, 1U);
  input.SetU16(kMovePositionsOffset, 500U);
  input.SetU16(kMoveDurationOffset, 20U);
  input.SetU8(kStopCountOffset, 1U);
  input.SetU8(kStopIdsOffset, 1U);
  input.SetU8(kConfigureIdOffset, 1U);
  input.SetU16(kConfigureMaskOffset, ConfigureBusServoCommand::kSetTorque);
  input.SetU8(kConfigureNewIdOffset, 2U);
  input.SetU8(kConfigureOffsetOffset, 0U);
  input.SetU16(kConfigurePositionMinOffset, 0U);
  input.SetU16(kConfigurePositionMaxOffset, 1000U);
  input.SetU16(kConfigureVoltageMinOffset, 4500U);
  input.SetU16(kConfigureVoltageMaxOffset, 14000U);
  input.SetU8(kConfigureTemperatureOffset, 100U);
  input.SetU8(kConfigureTorqueOffset, 1U);
  input.SetU8(kGetIdOffset, 1U);
  input.SetU16(kGetFieldsOffset, GetBusServoStateCommand::kFieldId);
  return input;
}

InputBuffer ValidPeripheralInput() {
  InputBuffer input;
  input.SetU8(0U, 1U);
  input.SetU16(1U, 0U);
  input.SetU16(3U, 0U);
  input.SetU16(5U, 0U);
  input.SetU16(7U, 1000U);
  input.SetU16(9U, 10U);
  input.SetU16(11U, 0U);
  input.SetU16(13U, 0U);
  input.SetU8(kRgbMaskOffset, kHostRgbPixelMask);
  input.SetU8(kOledMaskOffset, kAllOledLineMask);
  input.SetU8(kOledLine1SizeOffset, 0U);
  input.SetU8(kOledLine1BytesOffset, 0U);
  input.SetU8(kOledLine2SizeOffset, 0U);
  input.SetU8(kOledLine2BytesOffset, 0U);
  input.SetU16(kBatteryThresholdOffset, 5000U);
  input.SetU8(kMotorModelOffset, 0U);
  return input;
}

void TestMotorSemantics(Evidence* evidence) {
  constexpr std::array<std::uint8_t, 8> kSelectors{2U, 3U, 4U, 5U,
                                                   6U, 7U, 8U, 9U};
  for (std::uint8_t selector : kSelectors) {
    InputBuffer input;
    input.SetU8(0U, 1U);
    input.SetU8(1U, selector);
    input.SetU8(2U, 0U);
    input.SetU8(3U, 0U);
    input.SetU8(4U, 0U);
    input.SetU8(5U, 13U);

    fuzz::FuzzInput decoder(input.data(), input.size());
    CHECK(decoder.ReadU8() == 1U);
    const float value = decoder.ReadBiasedFloat();
    if (selector == 2U || selector == 3U) {
      CHECK(std::isinf(value));
    }
    if (selector == 4U) {
      CHECK(std::isnan(value));
    }
    MotorCommand command{};
    command.update_mask = 1U;
    command.target_rps[0] = value;
    const Result result = ValidateMotorCommand(command, 6.0F);
    CHECK(result.ok() == (std::isfinite(value) && std::fabs(value) <= 6.0F));
    ExerciseOracle(input, input_type::kMotor, input_type::kMotor, 0U,
                   feature::kNanInf | feature::kSignedBoundary, evidence);
  }

  for (std::uint8_t mask : std::array<std::uint8_t, 2>{0U, 0xffU}) {
    InputBuffer input;
    input.SetU8(0U, mask);
    input.SetU8(1U, 0U);
    input.SetU8(2U, 0U);
    input.SetU8(3U, 0U);
    input.SetU8(4U, 0U);
    input.SetU8(5U, 13U);
    ExerciseOracle(
        input, input_type::kMotor, input_type::kMotor, input_type::kMotor,
        feature::kMalformedMask | feature::kUnsignedBoundary, evidence);
  }
}

void TestPwmSemantics(Evidence* evidence) {
  InputBuffer maximum = ValidPwmInput();
  maximum.SetU16(1U, 30000U);
  for (std::size_t index = 0U; index < kPwmServoCount; ++index) {
    maximum.SetU16(3U + (index * 2U), 2500U);
  }
  ExerciseOracle(maximum, input_type::kPwm | input_type::kPwmOffsets,
                 input_type::kPwm | input_type::kPwmOffsets, 0U,
                 feature::kUnsignedBoundary, evidence);

  InputBuffer malformed = ValidPwmInput();
  malformed.SetU8(0U, 0xffU);
  malformed.SetU8(kPwmOffsetMaskOffset, 0xffU);
  ExerciseOracle(malformed, input_type::kPwm | input_type::kPwmOffsets,
                 input_type::kPwm | input_type::kPwmOffsets,
                 input_type::kPwm | input_type::kPwmOffsets,
                 feature::kMalformedMask | feature::kUnsignedBoundary,
                 evidence);

  for (std::int16_t boundary_value :
       std::array<std::int16_t, 2>{std::numeric_limits<std::int16_t>::min(),
                                   std::numeric_limits<std::int16_t>::max()}) {
    InputBuffer input = ValidPwmInput();
    input.SetU8(kPwmOffsetMaskOffset, 1U);
    input.SetI16(kPwmOffsetsOffset, boundary_value);
    PwmServoOffsetCommand command{};
    command.update_mask = 1U;
    command.offset_us[0] = boundary_value;
    CHECK(ValidatePwmServoOffsets(command).code == ResultCode::kOutOfRange);
    constexpr std::uint32_t kBoundaryTypeMask = input_type::kPwmOffsets;
    constexpr std::uint32_t kFeatureMask = feature::kSignedBoundary;
    ExerciseOracle(input, input_type::kPwmOffsets, kBoundaryTypeMask, 0U,
                   kFeatureMask, evidence);
  }
}

void PopulateMaximumBusInput(InputBuffer* input) {
  CHECK(input != nullptr);
  if (input == nullptr) {
    return;
  }
  input->SetU8(0U, static_cast<std::uint8_t>(kBusServoBatchCapacity));
  input->SetU8(kStopCountOffset,
               static_cast<std::uint8_t>(kBusServoBatchCapacity));
  for (std::size_t index = 0U; index < kBusServoBatchCapacity; ++index) {
    const auto id = static_cast<std::uint8_t>(index + 1U);
    input->SetU8(kMoveIdsOffset + index, id);
    input->SetU16(kMovePositionsOffset + (index * 2U), 1000U);
    input->SetU8(kStopIdsOffset + index, id);
  }
  input->SetU16(kMoveDurationOffset, 30000U);
}

void TestBusSemantics(Evidence* evidence) {
  constexpr std::uint32_t kBusTypes =
      input_type::kBusMove | input_type::kBusStop | input_type::kBusConfigure |
      input_type::kBusState;
  InputBuffer maximum = ValidBusInput();
  PopulateMaximumBusInput(&maximum);
  ExerciseOracle(maximum, kBusTypes, kBusTypes, 0U,
                 feature::kMaximumCount | feature::kUnsignedBoundary, evidence);

  InputBuffer duplicate = maximum;
  duplicate.SetU8(kMoveIdsOffset + kBusServoBatchCapacity - 1U, 1U);
  duplicate.SetU8(kStopIdsOffset + kBusServoBatchCapacity - 1U, 1U);
  BusServoCommand move{};
  StopBusServosCommand stop{};
  move.count = static_cast<std::uint8_t>(kBusServoBatchCapacity);
  stop.count = static_cast<std::uint8_t>(kBusServoBatchCapacity);
  for (std::size_t index = 0U; index < kBusServoBatchCapacity; ++index) {
    move.servo_id[index] = static_cast<std::uint8_t>(index + 1U);
    stop.servo_id[index] = static_cast<std::uint8_t>(index + 1U);
  }
  move.servo_id.back() = 1U;
  stop.servo_id.back() = 1U;
  CHECK(ValidateBusServoCommand(move).code == ResultCode::kInvalidArgument);
  CHECK(ValidateStopBusServosCommand(stop).code ==
        ResultCode::kInvalidArgument);
  ExerciseOracle(duplicate, input_type::kBusMove | input_type::kBusStop,
                 input_type::kBusMove | input_type::kBusStop, 0U,
                 feature::kDuplicateId, evidence);

  InputBuffer masks = ValidBusInput();
  masks.SetU16(kConfigureMaskOffset, 0xffffU);
  masks.SetU16(kGetFieldsOffset, 0xffffU);
  ExerciseOracle(masks, input_type::kBusConfigure | input_type::kBusState,
                 input_type::kBusConfigure | input_type::kBusState,
                 input_type::kBusConfigure | input_type::kBusState,
                 feature::kMalformedMask | feature::kUnsignedBoundary,
                 evidence);

  for (std::int8_t boundary_value :
       std::array<std::int8_t, 2>{std::numeric_limits<std::int8_t>::min(),
                                  std::numeric_limits<std::int8_t>::max()}) {
    InputBuffer input = ValidBusInput();
    constexpr std::uint16_t kFieldValue = ConfigureBusServoCommand::kSetOffset;
    input.SetU16(kConfigureMaskOffset, kFieldValue);
    input.SetU8(kConfigureOffsetOffset,
                static_cast<std::uint8_t>(boundary_value));
    ConfigureBusServoCommand configure{};
    configure.servo_id = 1U;
    configure.update_mask = ConfigureBusServoCommand::kSetOffset;
    configure.offset = boundary_value;
    CHECK(ValidateConfigureBusServoCommand(configure).code ==
          ResultCode::kOutOfRange);
    constexpr std::uint32_t kBoundaryTypeMask = input_type::kBusConfigure;
    constexpr std::uint32_t kFeatureMask = feature::kSignedBoundary;
    ExerciseOracle(input, input_type::kBusConfigure, kBoundaryTypeMask, 0U,
                   kFeatureMask, evidence);
  }

  for (std::uint8_t boundary : std::array<std::uint8_t, 2>{0U, 0xffU}) {
    InputBuffer input = ValidBusInput();
    input.SetU8(0U, boundary);
    input.SetU8(kStopCountOffset, boundary);
    input.SetU8(kGetIdOffset, boundary);
    ExerciseOracle(
        input,
        input_type::kBusMove | input_type::kBusStop | input_type::kBusState,
        input_type::kBusMove | input_type::kBusStop | input_type::kBusState, 0U,
        feature::kUnsignedBoundary, evidence);
  }
}

void SetOledLine(InputBuffer* input, std::size_t size_offset,
                 std::size_t bytes_offset, std::uint8_t size, char byte,
                 bool terminated) {
  CHECK(input != nullptr);
  if (input == nullptr) {
    return;
  }
  input->SetU8(size_offset, size);
  for (std::size_t index = 0U; index <= kOledLineCapacity; ++index) {
    input->SetU8(bytes_offset + index, static_cast<std::uint8_t>(byte));
  }
  if (terminated && size <= kOledLineCapacity) {
    input->SetU8(bytes_offset + size, 0U);
  }
}

void TestPeripheralSemantics(Evidence* evidence) {
  constexpr std::uint32_t kPeripheralTypes =
      input_type::kLed | input_type::kBuzzer | input_type::kRgb |
      input_type::kOled | input_type::kBatteryThreshold |
      input_type::kMotorModel;

  InputBuffer boundaries = ValidPeripheralInput();
  boundaries.SetU8(0U, 0xffU);
  boundaries.SetU16(1U, 0xffffU);
  boundaries.SetU16(7U, 0xffffU);
  boundaries.SetU16(9U, 1U);
  boundaries.SetU8(kRgbMaskOffset, 0xffU);
  boundaries.SetU16(kBatteryThresholdOffset, 0xffffU);
  boundaries.SetU8(kMotorModelOffset, 0xffU);
  ExerciseOracle(
      boundaries, kPeripheralTypes, kPeripheralTypes, input_type::kRgb,
      feature::kMalformedMask | feature::kUnsignedBoundary, evidence);

  InputBuffer oled_mask = ValidPeripheralInput();
  oled_mask.SetU8(kOledMaskOffset, 0xffU);
  ExerciseOracle(
      oled_mask, input_type::kOled, input_type::kOled, input_type::kOled,
      feature::kMalformedMask | feature::kUnsignedBoundary, evidence);

  InputBuffer maximum_text = ValidPeripheralInput();
  maximum_text.SetU8(kOledMaskOffset, 1U);
  SetOledLine(&maximum_text, kOledLine1SizeOffset, kOledLine1BytesOffset,
              static_cast<std::uint8_t>(kOledLineCapacity), '~', true);
  ExerciseOracle(maximum_text, input_type::kOled, input_type::kOled, 0U,
                 feature::kUnsignedBoundary, evidence);

  InputBuffer oversized_text = ValidPeripheralInput();
  oversized_text.SetU8(kOledMaskOffset, 1U);
  SetOledLine(&oversized_text, kOledLine1SizeOffset, kOledLine1BytesOffset,
              static_cast<std::uint8_t>(kOledLineCapacity + 1U), 'A', false);
  ExerciseOracle(oversized_text, input_type::kOled, input_type::kOled, 0U,
                 feature::kMalformedText | feature::kUnsignedBoundary,
                 evidence);

  InputBuffer unterminated_text = ValidPeripheralInput();
  unterminated_text.SetU8(kOledMaskOffset, 1U);
  SetOledLine(&unterminated_text, kOledLine1SizeOffset, kOledLine1BytesOffset,
              static_cast<std::uint8_t>(kOledLineCapacity), 'A', false);
  ExerciseOracle(unterminated_text, input_type::kOled, input_type::kOled, 0U,
                 feature::kMalformedText, evidence);

  std::array<bool, 256U> model_seen{};
  for (std::uint16_t value = 0U; value <= 0xffU; ++value) {
    InputBuffer input = ValidPeripheralInput();
    input.SetU8(kMotorModelOffset, static_cast<std::uint8_t>(value));
    model_seen[value] = true;
    CHECK(IsValidMotorModel(static_cast<MotorModel>(value)) == (value <= 3U));
    ExerciseOracle(input, input_type::kMotorModel, input_type::kMotorModel, 0U,
                   feature::kEveryEnumByte | feature::kUnsignedBoundary,
                   evidence);
  }
  CHECK(std::all_of(model_seen.begin(), model_seen.end(),
                    [](bool seen) { return seen; }));
}

void VerifyEvidence(const Evidence& evidence) {
  CHECK(evidence.input_types == input_type::kAll);
  CHECK(evidence.boundary_types == input_type::kAll);
  CHECK(evidence.malformed_mask_types == input_type::kMaskBearing);
  CHECK(evidence.features == feature::kAll);
  CHECK(evidence.oracle_inputs == 282U);
}

}  // namespace
}  // namespace mentor_pi::mcu

int main() {
  mentor_pi::mcu::Evidence evidence{};
  mentor_pi::mcu::TestMotorSemantics(&evidence);
  mentor_pi::mcu::TestPwmSemantics(&evidence);
  mentor_pi::mcu::TestBusSemantics(&evidence);
  mentor_pi::mcu::TestPeripheralSemantics(&evidence);
  mentor_pi::mcu::VerifyEvidence(evidence);

  if (mentor_pi::mcu::g_test_failures != 0U) {
    std::cerr << mentor_pi::mcu::g_test_failures
              << " semantic corpus contract check(s) failed\n";
    return 1;
  }
  std::cout << "validation semantic corpus contract passed: "
            << evidence.oracle_inputs << " deterministic oracle inputs, "
            << "13 v2 input types, 256 motor-model enum bytes\n";
  return 0;
}
