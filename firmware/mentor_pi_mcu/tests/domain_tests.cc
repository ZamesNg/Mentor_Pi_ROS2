#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

#include "mentor_pi_mcu/app/microros/reclaiming_arena.h"
#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/domain/battery_monitor.h"
#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/domain/button_controller.h"
#include "mentor_pi_mcu/domain/circular_dma_position.h"
#include "mentor_pi_mcu/domain/circular_rx_ring.h"
#include "mentor_pi_mcu/domain/command_mailboxes.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/fixed_containers.h"
#include "mentor_pi_mcu/domain/motor_controller.h"
#include "mentor_pi_mcu/domain/pattern_controller.h"
#include "mentor_pi_mcu/domain/pwm_servo_controller.h"
#include "mentor_pi_mcu/domain/state_merger.h"
#include "mentor_pi_mcu/domain/validation.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"

namespace mentor_pi::mcu {
namespace {

std::uint32_t test_failures = 0;

void Check(bool condition, const char* expression, const char* file, int line) {
  if (!condition) {
    ++test_failures;
    std::cerr << file << ':' << line << ": check failed: " << expression
              << '\n';
  }
}

#define CHECK(expression) \
  Check(static_cast<bool>(expression), #expression, __FILE__, __LINE__)

void CheckNear(float actual, float expected, float tolerance,
               const char* expression, const char* file, int line) {
  Check(std::fabs(actual - expected) <= tolerance, expression, file, line);
}

#define CHECK_NEAR(actual, expected, tolerance)                          \
  CheckNear((actual), (expected), (tolerance), #actual " ~= " #expected, \
            __FILE__, __LINE__)

BoundedText MakeText(const char* text) {
  BoundedText result{};
  const std::size_t length = std::strlen(text);
  CHECK(length <= kOledLineCapacity);
  result.size = static_cast<std::uint8_t>(length);
  std::memcpy(result.bytes.data(), text, length);
  result.bytes[length] = '\0';
  return result;
}

MotorControlConfiguration FullRangeTestMotorConfiguration() {
  // This enables mathematical unit coverage only. It is not hardware HIL
  // evidence and must never be reused as the production target default.
  MotorControlConfiguration configuration{};
  configuration.mode = MotorControlMode::kClosedLoop;
  configuration.maximum_accepted_rps = 6.0F;
  configuration.output_limit_permille = kMotorOutputLimitPermille;
  return configuration;
}

template <typename Command>
void PopulateUniqueBusServoIds(Command* command, std::size_t count) {
  command->count = static_cast<std::uint8_t>(count);
  for (std::size_t index = 0U; index < count; ++index) {
    command->servo_id[index] = static_cast<std::uint8_t>(index + 1U);
  }
}

void TestValidationAndStateMerging() {
  MotorCommand motor{};
  motor.update_mask = 1U;
  motor.target_rps[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = 6.0F;
  CHECK(ValidateMotorCommand(motor, 6.0F).ok());
  motor.update_mask = 0U;
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kInvalidArgument);
  for (std::uint16_t mask = 0; mask <= 0xffU; ++mask) {
    motor.update_mask = static_cast<std::uint8_t>(mask);
    motor.target_rps.fill(0.0F);
    const bool expected_valid = mask >= 1U && mask <= kAllMotorMask;
    CHECK(ValidateMotorCommand(motor, 6.0F).ok() == expected_valid);
  }

  BusServoCommand bus{};
  bus.count = 2U;
  bus.servo_id[0] = 1U;
  bus.servo_id[1] = 1U;
  CHECK(ValidateBusServoCommand(bus).code == ResultCode::kInvalidArgument);
  bus.servo_id[1] = 253U;
  bus.position[1] = 1001U;
  CHECK(ValidateBusServoCommand(bus).code == ResultCode::kOutOfRange);

  ConfigureBusServoCommand configure{};
  configure.servo_id = 1U;
  configure.update_mask = ConfigureBusServoCommand::kSetPositionLimits;
  configure.position_min = 900U;
  configure.position_max = 100U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kInvalidArgument);

  RgbState rgb{};
  RgbCommand rgb_first{};
  rgb_first.update_mask = 1U;
  rgb_first.red[0] = 10U;
  rgb_first.green[0] = 20U;
  rgb_first.blue[0] = 30U;
  CHECK(MergeRgbCommand(rgb_first, &rgb).ok());
  RgbCommand rgb_second{};
  rgb_second.update_mask = 2U;
  rgb_second.red[1] = 40U;
  CHECK(MergeRgbCommand(rgb_second, &rgb).code == ResultCode::kInvalidArgument);
  CHECK(rgb.red[0] == 10U && rgb.green[0] == 20U && rgb.blue[0] == 30U);
  CHECK(rgb.red[1] == 0U);

  OledState oled{};
  OledCommand oled_command{};
  oled_command.update_mask = 1U;
  oled_command.lines[0] = MakeText("ready");
  oled_command.lines[1].size = 1U;
  oled_command.lines[1].bytes[0] = static_cast<char>(0x01);
  CHECK(MergeOledCommand(oled_command, &oled).ok());
  CHECK(oled.lines[0].size == 5U);
  oled_command.update_mask = 2U;
  CHECK(MergeOledCommand(oled_command, &oled).code ==
        ResultCode::kInvalidArgument);

  OledCommand byte_command{};
  byte_command.update_mask = 1U;
  byte_command.lines[0].size = 1U;
  byte_command.lines[0].bytes[1] = '\0';
  for (std::uint16_t byte = 0; byte <= 0xffU; ++byte) {
    byte_command.lines[0].bytes[0] = static_cast<char>(byte);
    const bool printable = byte >= 0x20U && byte <= 0x7eU;
    CHECK(ValidateOledCommand(byte_command).ok() == printable);
  }

  RgbCommand rgb_mask{};
  for (std::uint16_t mask = 0; mask <= 0xffU; ++mask) {
    rgb_mask.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask == kHostRgbPixelMask;
    CHECK(ValidateRgbCommand(rgb_mask).ok() == expected_valid);
  }
}

void TestValidationBoundaries() {
  MotorCommand motor{};
  motor.update_mask = 1U;
  motor.target_rps[0] = -6.0F;
  CHECK(ValidateMotorCommand(motor, 6.0F).ok());
  motor.target_rps[0] = std::nextafter(6.0F, 7.0F);
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = std::nextafter(-6.0F, -7.0F);
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = std::numeric_limits<float>::infinity();
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = -std::numeric_limits<float>::infinity();
  CHECK(ValidateMotorCommand(motor, 6.0F).code == ResultCode::kOutOfRange);
  motor.target_rps[0] = 0.0F;
  motor.target_rps[1] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateMotorCommand(motor, 6.0F).ok());
  CHECK(ValidateMotorCommand(motor, 0.0F).code == ResultCode::kInvalidArgument);
  CHECK(ValidateMotorCommand(motor, -1.0F).code ==
        ResultCode::kInvalidArgument);
  CHECK(ValidateMotorCommand(motor, std::numeric_limits<float>::infinity())
            .code == ResultCode::kInvalidArgument);
  CHECK(ValidateMotorCommand(motor, std::numeric_limits<float>::quiet_NaN())
            .code == ResultCode::kInvalidArgument);

  SetMotorPidCommand pid{};
  pid.update_mask = 1U;
  pid.proportional_gain[0] = 1000.0F;
  pid.integral_gain[0] = 1000.0F;
  pid.derivative_gain[0] = 1000.0F;
  pid.velocity_filter_new_weight[0] = 1.0F;
  CHECK(ValidateSetMotorPidCommand(pid).ok());
  pid.update_mask = 0U;
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kInvalidArgument);
  pid.update_mask = 0x10U;
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kInvalidArgument);
  pid.update_mask = 1U;
  pid.proportional_gain[0] = std::nextafter(
      kMotorPidMaximumGain, std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kOutOfRange);
  pid.proportional_gain[0] = -std::numeric_limits<float>::epsilon();
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kOutOfRange);
  pid.proportional_gain[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kInvalidArgument);
  pid.proportional_gain[0] = 0.0F;
  pid.integral_gain[0] = std::numeric_limits<float>::infinity();
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kInvalidArgument);
  pid.integral_gain[0] = 0.0F;
  pid.derivative_gain[0] = -std::numeric_limits<float>::infinity();
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kInvalidArgument);
  pid.derivative_gain[0] = 0.0F;
  pid.velocity_filter_new_weight[0] =
      std::nextafter(1.0F, std::numeric_limits<float>::infinity());
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kOutOfRange);
  pid.velocity_filter_new_weight[0] = std::numeric_limits<float>::quiet_NaN();
  CHECK(ValidateSetMotorPidCommand(pid).code == ResultCode::kInvalidArgument);

  PwmServoCommand pwm{};
  pwm.duration_ms = 20U;
  pwm.pulse_width_us.fill(500U);
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    pwm.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask >= 1U && mask <= kAllPwmServoMask;
    CHECK(ValidatePwmServoCommand(pwm).ok() == expected_valid);
  }
  pwm.update_mask = 4U;
  pwm.duration_ms = 19U;
  CHECK(ValidatePwmServoCommand(pwm).code == ResultCode::kOutOfRange);
  pwm.duration_ms = 30000U;
  pwm.pulse_width_us[2] = 2500U;
  CHECK(ValidatePwmServoCommand(pwm).ok());
  pwm.duration_ms = 30001U;
  CHECK(ValidatePwmServoCommand(pwm).code == ResultCode::kOutOfRange);
  pwm.duration_ms = 20U;
  pwm.pulse_width_us[2] = 499U;
  CHECK(ValidatePwmServoCommand(pwm).detail == 3U);
  pwm.pulse_width_us[2] = 2501U;
  CHECK(ValidatePwmServoCommand(pwm).detail == 3U);
  pwm.pulse_width_us[2] = 500U;
  pwm.pulse_width_us[3] = 65535U;
  CHECK(ValidatePwmServoCommand(pwm).ok());

  PwmServoOffsetCommand offsets{};
  offsets.update_mask = 8U;
  offsets.offset_us[3] = -100;
  CHECK(ValidatePwmServoOffsets(offsets).ok());
  offsets.offset_us[3] = 100;
  CHECK(ValidatePwmServoOffsets(offsets).ok());
  offsets.offset_us[3] = -101;
  CHECK(ValidatePwmServoOffsets(offsets).detail == 4U);
  offsets.offset_us[3] = 101;
  CHECK(ValidatePwmServoOffsets(offsets).detail == 4U);
  offsets.update_mask = 1U;
  CHECK(ValidatePwmServoOffsets(offsets).ok());
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    PwmServoOffsetCommand candidate{};
    candidate.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask >= 1U && mask <= kAllPwmServoMask;
    CHECK(ValidatePwmServoOffsets(candidate).ok() == expected_valid);
  }

  BusServoCommand bus{};
  CHECK(ValidateBusServoCommand(bus).code == ResultCode::kInvalidArgument);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.position.fill(1000U);
  bus.duration_ms = 30000U;
  CHECK(ValidateBusServoCommand(bus).ok());
  bus.count = static_cast<std::uint8_t>(kBusServoBatchCapacity + 1U);
  CHECK(ValidateBusServoCommand(bus).detail == kBusServoBatchCapacity + 1U);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.servo_id[0] = 0U;
  CHECK(ValidateBusServoCommand(bus).detail == 1U);
  bus.servo_id[0] = 254U;
  CHECK(ValidateBusServoCommand(bus).detail == 1U);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.servo_id[kBusServoBatchCapacity - 1U] = 1U;
  CHECK(ValidateBusServoCommand(bus).detail == kBusServoBatchCapacity);
  PopulateUniqueBusServoIds(&bus, kBusServoBatchCapacity);
  bus.position[kBusServoBatchCapacity - 1U] = 1001U;
  CHECK(ValidateBusServoCommand(bus).detail == kBusServoBatchCapacity);
  bus.position[kBusServoBatchCapacity - 1U] = 1000U;
  bus.duration_ms = 30001U;
  CHECK(ValidateBusServoCommand(bus).detail == 0U);

  StopBusServosCommand stop{};
  CHECK(ValidateStopBusServosCommand(stop).code ==
        ResultCode::kInvalidArgument);
  PopulateUniqueBusServoIds(&stop, kBusServoBatchCapacity);
  CHECK(ValidateStopBusServosCommand(stop).ok());
  stop.count = static_cast<std::uint8_t>(kBusServoBatchCapacity + 1U);
  CHECK(ValidateStopBusServosCommand(stop).detail ==
        kBusServoBatchCapacity + 1U);
  PopulateUniqueBusServoIds(&stop, kBusServoBatchCapacity);
  stop.servo_id[kBusServoBatchCapacity - 1U] = 1U;
  CHECK(ValidateStopBusServosCommand(stop).detail == kBusServoBatchCapacity);

  ConfigureBusServoCommand configure{};
  configure.servo_id = 1U;
  configure.new_id = 253U;
  configure.offset = 125;
  configure.position_min = 0U;
  configure.position_max = 1000U;
  configure.voltage_min_mv = 4500U;
  configure.voltage_max_mv = 14000U;
  configure.temperature_limit_c = 100U;
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    configure.update_mask = mask;
    const bool expected_valid =
        mask >= 1U && mask <= ConfigureBusServoCommand::kAllUpdates;
    CHECK(ValidateConfigureBusServoCommand(configure).ok() == expected_valid);
  }
  configure.update_mask = ConfigureBusServoCommand::kSetId;
  configure.servo_id = 0U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 0U);
  configure.servo_id = 254U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 0U);
  configure.servo_id = 1U;
  configure.new_id = 0U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 1U);
  configure.new_id = 254U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 1U);
  configure.update_mask = ConfigureBusServoCommand::kSetOffset;
  configure.offset = -125;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.offset = -126;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 2U);
  configure.offset = 126;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 2U);
  configure.update_mask = ConfigureBusServoCommand::kSetPositionLimits;
  configure.position_min = 1000U;
  configure.position_max = 1000U;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.position_max = 1001U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kOutOfRange);
  configure.position_min = 900U;
  configure.position_max = 100U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kInvalidArgument);
  configure.update_mask = ConfigureBusServoCommand::kSetVoltageLimits;
  configure.voltage_min_mv = 4500U;
  configure.voltage_max_mv = 14000U;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.voltage_min_mv = 4499U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kOutOfRange);
  configure.voltage_min_mv = 14000U;
  configure.voltage_max_mv = 13999U;
  CHECK(ValidateConfigureBusServoCommand(configure).code ==
        ResultCode::kInvalidArgument);
  configure.update_mask = ConfigureBusServoCommand::kSetTemperatureLimit;
  configure.temperature_limit_c = 100U;
  CHECK(ValidateConfigureBusServoCommand(configure).ok());
  configure.temperature_limit_c = 101U;
  CHECK(ValidateConfigureBusServoCommand(configure).detail == 6U);

  GetBusServoStateCommand get{};
  get.servo_id = 1U;
  for (std::uint16_t fields = 0U; fields <= 0x03ffU; ++fields) {
    get.fields = fields;
    const bool expected_valid =
        fields >= 1U && fields <= GetBusServoStateCommand::kAllFields;
    CHECK(ValidateGetBusServoStateCommand(get).ok() == expected_valid);
  }
  get.fields = GetBusServoStateCommand::kFieldId;
  get.servo_id = 253U;
  CHECK(ValidateGetBusServoStateCommand(get).ok());
  get.servo_id = 254U;
  CHECK(ValidateGetBusServoStateCommand(get).ok());
  get.fields = 2U;
  CHECK(ValidateGetBusServoStateCommand(get).code ==
        ResultCode::kInvalidArgument);
  get.fields = GetBusServoStateCommand::kFieldId;
  get.servo_id = 0U;
  CHECK(ValidateGetBusServoStateCommand(get).code == ResultCode::kOutOfRange);
  get.servo_id = 255U;
  CHECK(ValidateGetBusServoStateCommand(get).code == ResultCode::kOutOfRange);

  CHECK(ValidateLedCommand({1U, 0U, 0U, 0U}).ok());
  CHECK(ValidateLedCommand({2U, 0U, 0U, 0U}).ok());
  CHECK(ValidateLedCommand({3U, 0U, 0U, 0U}).code == ResultCode::kOutOfRange);
  CHECK(ValidateLedCommand({0U, 0U, 0U, 0U}).code == ResultCode::kOutOfRange);
  CHECK(ValidateLedCommand({4U, 0U, 0U, 0U}).code == ResultCode::kOutOfRange);
  CHECK(ValidateBuzzerCommand({0U, 65535U, 65535U, 65535U}).ok());
  CHECK(ValidateBuzzerCommand({65535U, 0U, 65535U, 65535U}).ok());
  CHECK(ValidateBuzzerCommand({10U, 1U, 0U, 0U}).ok());
  CHECK(ValidateBuzzerCommand({20000U, 1U, 0U, 0U}).ok());
  CHECK(ValidateBuzzerCommand({9U, 1U, 0U, 0U}).code ==
        ResultCode::kOutOfRange);
  CHECK(ValidateBuzzerCommand({20001U, 1U, 0U, 0U}).code ==
        ResultCode::kOutOfRange);

  OledCommand oled{};
  for (std::uint16_t mask = 0U; mask <= 0xffU; ++mask) {
    oled.update_mask = static_cast<std::uint8_t>(mask);
    const bool expected_valid = mask >= 1U && mask <= kAllOledLineMask;
    CHECK(ValidateOledCommand(oled).ok() == expected_valid);
  }
  oled.update_mask = 1U;
  oled.lines[0].size = static_cast<std::uint8_t>(kOledLineCapacity);
  oled.lines[0].bytes.fill('~');
  oled.lines[0].bytes[kOledLineCapacity] = '\0';
  CHECK(ValidateOledCommand(oled).ok());
  oled.lines[0].size = static_cast<std::uint8_t>(kOledLineCapacity + 1U);
  CHECK(ValidateOledCommand(oled).code == ResultCode::kInvalidArgument);
  oled.lines[0] = MakeText("A");
  oled.lines[0].bytes[1] = 'X';
  CHECK(ValidateOledCommand(oled).code == ResultCode::kInvalidArgument);
  oled.lines[0] = MakeText("A");
  oled.lines[1].size = static_cast<std::uint8_t>(kOledLineCapacity + 1U);
  CHECK(ValidateOledCommand(oled).ok());

  RgbCommand invalid_rgb{};
  RgbState rgb_state{};
  rgb_state.red = {10U, 20U};
  const RgbState rgb_before = rgb_state;
  CHECK(MergeRgbCommand(invalid_rgb, &rgb_state).code ==
        ResultCode::kInvalidArgument);
  CHECK(rgb_state.red == rgb_before.red);
  invalid_rgb.update_mask = 1U;
  CHECK(MergeRgbCommand(invalid_rgb, nullptr).code ==
        ResultCode::kInvalidArgument);

  OledCommand invalid_oled{};
  invalid_oled.update_mask = 1U;
  invalid_oled.lines[0].size = 1U;
  invalid_oled.lines[0].bytes[0] = static_cast<char>(0x1f);
  OledState oled_state{};
  oled_state.lines[0] = MakeText("unchanged");
  const OledState oled_before = oled_state;
  CHECK(MergeOledCommand(invalid_oled, &oled_state).code ==
        ResultCode::kInvalidArgument);
  CHECK(oled_state.lines[0].size == oled_before.lines[0].size);
  CHECK(oled_state.lines[0].bytes == oled_before.lines[0].bytes);
  CHECK(MergeOledCommand(invalid_oled, nullptr).code ==
        ResultCode::kInvalidArgument);

  CHECK(ValidateBatteryThreshold(4999U).code == ResultCode::kOutOfRange);
  CHECK(ValidateBatteryThreshold(5000U).ok());
  CHECK(ValidateBatteryThreshold(20000U).ok());
  CHECK(ValidateBatteryThreshold(20001U).code == ResultCode::kOutOfRange);
  for (std::uint16_t value = 0U; value <= 0xffU; ++value) {
    const auto model = static_cast<MotorModel>(value);
    CHECK(IsValidMotorModel(model) == (value <= 3U));
  }
}

void TestFixedContainers() {
  SaturatingCounter<std::uint8_t> counter{254U};
  counter.Increment();
  counter.Increment();
  CHECK(counter.value() == 255U);
  SaturatingCounter<std::uint32_t> diagnostic_counter{
      std::numeric_limits<std::uint32_t>::max() - 1U};
  diagnostic_counter.Increment();
  diagnostic_counter.Increment();
  CHECK(diagnostic_counter.value() ==
        std::numeric_limits<std::uint32_t>::max());

  LatestMailbox<std::uint32_t> mailbox;
  CHECK(!mailbox.Publish(10U));
  CHECK(mailbox.Publish(20U));
  std::uint32_t value = 0;
  CHECK(mailbox.ConsumeLatest(&value));
  CHECK(value == 20U);
  CHECK(!mailbox.ConsumeLatest(&value));
  CHECK(!mailbox.DiscardLatest());
  CHECK(!mailbox.Publish(30U));
  CHECK(mailbox.has_unread());
  CHECK(mailbox.DiscardLatest());
  CHECK(!mailbox.has_unread());
  CHECK(!mailbox.ConsumeLatest(&value));
  CHECK(!mailbox.Publish(40U));
  CHECK(mailbox.ConsumeLatest(&value));
  CHECK(value == 40U);

  DropOldestQueue<std::uint32_t, 16> queue;
  for (std::uint32_t item = 1; item <= 32U; ++item) {
    queue.PushDropOldest(item);
  }
  CHECK(queue.dropped_count() == 16U);
  for (std::uint32_t expected = 17U; expected <= 32U; ++expected) {
    value = 0;
    CHECK(queue.TryPop(&value));
    CHECK(value == expected);
  }
  CHECK(!queue.TryPop(&value));
}

void TestCommandMailboxes() {
  MotorCommandMailbox motors;
  MotorCommand first{};
  first.update_mask = 1U;
  first.target_rps[0] = 1.0F;
  CHECK(motors.Publish(first, 6.0F, 100U).result.ok());
  MotorCommand second{};
  second.update_mask = 2U;
  second.target_rps[1] = 2.0F;
  const CommandAdmission second_admission = motors.Publish(second, 6.0F, 200U);
  CHECK(second_admission.result.ok() && second_admission.overwrote_unread);
  CHECK(motors.overwrite_count() == 1U);
  MotorCommandSnapshot motor_snapshot{};
  CHECK(motors.ConsumeLatest(&motor_snapshot));
  CHECK_NEAR(motor_snapshot.target_rps[0], 1.0F, 0.0001F);
  CHECK_NEAR(motor_snapshot.target_rps[1], 2.0F, 0.0001F);
  CHECK(motor_snapshot.accepted_at_us[0] == 100U);
  CHECK(motor_snapshot.accepted_at_us[1] == 200U);
  CHECK(motor_snapshot.field_generation[0] == 1U);
  CHECK(motor_snapshot.field_generation[1] == 2U);

  MotorCommand invalid = second;
  invalid.target_rps[1] = 7.0F;
  CHECK(motors.Publish(invalid, 6.0F, 300U).result.code ==
        ResultCode::kOutOfRange);
  CHECK(!motors.ConsumeLatest(&motor_snapshot));

  PwmCommandMailbox pwm;
  PwmServoCommand pwm_first{};
  pwm_first.update_mask = 1U;
  pwm_first.duration_ms = 40U;
  pwm_first.pulse_width_us[0] = 1000U;
  CHECK(pwm.Publish(pwm_first).result.ok());
  PwmServoCommand pwm_second{};
  pwm_second.update_mask = 2U;
  pwm_second.duration_ms = 60U;
  pwm_second.pulse_width_us[1] = 2000U;
  CHECK(pwm.Publish(pwm_second).result.ok());
  PwmCommandSnapshot pwm_snapshot{};
  CHECK(pwm.ConsumeLatest(&pwm_snapshot));
  CHECK(pwm_snapshot.pulse_width_us[0] == 1000U);
  CHECK(pwm_snapshot.duration_ms[0] == 40U);
  CHECK(pwm_snapshot.pulse_width_us[1] == 2000U);
  CHECK(pwm_snapshot.duration_ms[1] == 60U);

  LedCommandMailbox leds;
  CHECK(leds.Publish({1U, 10U, 20U, 1U}).result.ok());
  CHECK(leds.Publish({2U, 30U, 40U, 2U}).result.ok());
  LedCommandSnapshot led_snapshot{};
  CHECK(leds.ConsumeLatest(&led_snapshot));
  CHECK(led_snapshot.commands[0].on_time_ms == 10U);
  CHECK(led_snapshot.commands[1].on_time_ms == 30U);
  CHECK(led_snapshot.commands[2].on_time_ms == 0U);

  BusMotionMailbox bus;
  BusServoCommand bus_first{};
  bus_first.count = 1U;
  bus_first.servo_id[0] = 1U;
  bus_first.position[0] = 100U;
  BusServoCommand bus_second = bus_first;
  bus_second.position[0] = 200U;
  CHECK(bus.Publish(bus_first).result.ok());
  CHECK(bus.Publish(bus_second).overwrote_unread);
  BusMotionSnapshot bus_snapshot{};
  CHECK(bus.ConsumeLatest(&bus_snapshot));
  CHECK(bus_snapshot.command.position[0] == 200U);

  BuzzerCommandMailbox buzzer;
  CHECK(buzzer.Publish({440U, 10U, 10U, 1U}).result.ok());
  CHECK(buzzer.Publish({880U, 20U, 20U, 2U}).overwrote_unread);
  BuzzerCommandSnapshot buzzer_snapshot{};
  CHECK(buzzer.ConsumeLatest(&buzzer_snapshot));
  CHECK(buzzer_snapshot.command.frequency_hz == 880U);

  RgbCommandMailbox rgb;
  RgbCommand rgb_full{};
  rgb_full.update_mask = kHostRgbPixelMask;
  rgb_full.red = {10U, 20U};
  rgb_full.green = {30U, 40U};
  rgb_full.blue = {50U, 60U};
  const CommandAdmission rgb_first = rgb.Publish(rgb_full);
  CHECK(rgb_first.result.ok());
  CHECK(!rgb_first.overwrote_unread);
  CHECK(rgb_first.generation == 1U);
  RgbCommand rgb_update{};
  rgb_update.update_mask = 1U;
  rgb_update.red[0] = 70U;
  rgb_update.green[0] = 80U;
  rgb_update.blue[0] = 90U;
  const CommandAdmission rgb_second = rgb.Publish(rgb_update);
  CHECK(rgb_second.result.ok());
  CHECK(rgb_second.overwrote_unread);
  CHECK(rgb_second.generation == 2U);
  rgb_update.red[0] = 100U;
  rgb_update.green[0] = 110U;
  rgb_update.blue[0] = 120U;
  const CommandAdmission rgb_third = rgb.Publish(rgb_update);
  CHECK(rgb_third.result.ok());
  CHECK(rgb_third.overwrote_unread);
  CHECK(rgb_third.generation == 3U);
  CHECK(rgb.overwrite_count() == 2U);
  RgbCommandSnapshot rgb_snapshot{};
  CHECK(rgb.ConsumeLatest(&rgb_snapshot));
  CHECK(rgb_snapshot.state.red[0] == 100U);
  CHECK(rgb_snapshot.state.green[0] == 110U);
  CHECK(rgb_snapshot.state.blue[0] == 120U);
  CHECK(rgb_snapshot.state.red[1] == 0U);
  CHECK(rgb_snapshot.state.green[1] == 0U);
  CHECK(rgb_snapshot.state.blue[1] == 0U);
  CHECK(!rgb.ConsumeLatest(&rgb_snapshot));
  RgbCommand invalid_rgb{};
  const CommandAdmission rejected_rgb = rgb.Publish(invalid_rgb);
  CHECK(rejected_rgb.result.code == ResultCode::kInvalidArgument);
  CHECK(rejected_rgb.generation == 3U);
  CHECK(!rejected_rgb.overwrote_unread);
  CHECK(rgb.overwrite_count() == 2U);
  CHECK(!rgb.ConsumeLatest(&rgb_snapshot));

  OledCommandMailbox oled;
  OledCommand oled_full{};
  oled_full.update_mask = kAllOledLineMask;
  oled_full.lines[0] = MakeText("first");
  oled_full.lines[1] = MakeText("retained");
  const CommandAdmission oled_first = oled.Publish(oled_full);
  CHECK(oled_first.result.ok());
  CHECK(!oled_first.overwrote_unread);
  CHECK(oled_first.generation == 1U);
  OledCommand oled_update{};
  oled_update.update_mask = 1U;
  oled_update.lines[0] = MakeText("second");
  const CommandAdmission oled_second = oled.Publish(oled_update);
  CHECK(oled_second.result.ok());
  CHECK(oled_second.overwrote_unread);
  CHECK(oled_second.generation == 2U);
  oled_update.lines[0] = MakeText("newest");
  const CommandAdmission oled_third = oled.Publish(oled_update);
  CHECK(oled_third.result.ok());
  CHECK(oled_third.overwrote_unread);
  CHECK(oled_third.generation == 3U);
  CHECK(oled.overwrite_count() == 2U);
  OledCommandSnapshot oled_snapshot{};
  CHECK(oled.ConsumeLatest(&oled_snapshot));
  CHECK(oled_snapshot.state.lines[0].size == 6U);
  CHECK(std::memcmp(oled_snapshot.state.lines[0].bytes.data(), "newest", 6U) ==
        0);
  CHECK(oled_snapshot.state.lines[1].size == 8U);
  CHECK(std::memcmp(oled_snapshot.state.lines[1].bytes.data(), "retained",
                    8U) == 0);
  CHECK(!oled.ConsumeLatest(&oled_snapshot));
  OledCommand invalid_oled{};
  const CommandAdmission rejected_oled = oled.Publish(invalid_oled);
  CHECK(rejected_oled.result.code == ResultCode::kInvalidArgument);
  CHECK(rejected_oled.generation == 3U);
  CHECK(!rejected_oled.overwrote_unread);
  CHECK(oled.overwrite_count() == 2U);
  CHECK(!oled.ConsumeLatest(&oled_snapshot));

  // A session boundary discards unread merged fields without resetting the
  // monotonic mailbox generation. The first disjoint update in the next
  // session therefore contains only fields accepted after the reset.
  CHECK(motors.Publish(first, 6.0F, 400U).result.ok());
  motors.ResetMergedFields();
  CHECK(motors.Publish(second, 6.0F, 500U).result.ok());
  CHECK(motors.ConsumeLatest(&motor_snapshot));
  CHECK(motor_snapshot.field_generation[0] == 0U);
  CHECK(motor_snapshot.target_rps[0] == 0.0F);
  CHECK(motor_snapshot.field_generation[1] == 4U);

  CHECK(pwm.Publish(pwm_first).result.ok());
  pwm.ResetMergedFields();
  CHECK(pwm.Publish(pwm_second).result.ok());
  CHECK(pwm.ConsumeLatest(&pwm_snapshot));
  CHECK(pwm_snapshot.field_generation[0] == 0U);
  CHECK(pwm_snapshot.pulse_width_us[0] == 1500U);
  CHECK(pwm_snapshot.field_generation[1] == 4U);

  CHECK(leds.Publish({1U, 10U, 20U, 1U}).result.ok());
  leds.ResetMergedFields();
  CHECK(leds.Publish({2U, 30U, 40U, 2U}).result.ok());
  CHECK(leds.ConsumeLatest(&led_snapshot));
  CHECK(led_snapshot.field_generation[0] == 0U);
  CHECK(led_snapshot.field_generation[1] == 4U);

  CHECK(rgb.Publish(rgb_update).result.ok());
  rgb.ResetMergedFields();
  RgbCommand rgb_new_session{};
  rgb_new_session.update_mask = kHostRgbPixelMask;
  rgb_new_session.blue[0] = 200U;
  CHECK(rgb.Publish(rgb_new_session).result.ok());
  CHECK(rgb.ConsumeLatest(&rgb_snapshot));
  CHECK(rgb_snapshot.field_generation[0] == 5U);
  CHECK(rgb_snapshot.state.red[0] == 0U);
  CHECK(rgb_snapshot.state.blue[0] == 200U);
  CHECK(rgb_snapshot.field_generation[1] == 0U);
  CHECK(rgb_snapshot.state.blue[1] == 0U);

  CHECK(oled.Publish(oled_update).result.ok());
  oled.ResetMergedFields();
  OledCommand oled_new_session{};
  oled_new_session.update_mask = 2U;
  oled_new_session.lines[1] = MakeText("new session");
  CHECK(oled.Publish(oled_new_session).result.ok());
  CHECK(oled.ConsumeLatest(&oled_snapshot));
  CHECK(oled_snapshot.field_generation[0] == 0U);
  CHECK(oled_snapshot.state.lines[0].size == 0U);
  CHECK(oled_snapshot.field_generation[1] == 5U);
  CHECK(oled_snapshot.state.lines[1].size == 11U);
}

void TestMotorController() {
  MotorController controller(FullRangeTestMotorConfiguration());
  CHECK(controller.profile().model == MotorModel::kJga27);
  controller.SetSessionActive(true);

  MotorCommand command{};
  command.update_mask = 1U;
  command.target_rps[0] = 1.0F;
  CHECK(controller.AcceptCommand(command, 1000U).ok());

  MotorCommand invalid{};
  invalid.update_mask = 3U;
  invalid.target_rps[0] = 1.0F;
  invalid.target_rps[1] = 7.0F;
  CHECK(controller.AcceptCommand(invalid, 100000U).code ==
        ResultCode::kOutOfRange);
  CHECK_NEAR(controller.channels()[0].target_rps, 1.0F, 0.0001F);

  controller.EvaluateLeases(198999U);
  CHECK(controller.channels()[0].armed);
  controller.EvaluateLeases(199000U);
  CHECK(!controller.channels()[0].armed);
  CHECK(controller.watchdog_stop_mask() == 1U);
  CHECK(controller.lease_expiry_count(0) == 1U);

  command.target_rps[0] = 0.5F;
  CHECK(controller.AcceptCommand(command, 0xfffffff0U).ok());
  controller.EvaluateLeases(0x00000020U);
  CHECK(controller.channels()[0].armed);
  CHECK(controller.AcceptCommand(command, 200U).ok());
  CHECK(controller.SetModel(MotorModel::kJgb37).result.code ==
        ResultCode::kBusy);
  MotorCommand stop = command;
  stop.target_rps[0] = 0.0F;
  CHECK(controller.AcceptCommand(stop, 300U).ok());
  CHECK(controller.SetModel(MotorModel::kJgb37).result.ok());
  CHECK(controller.profile().ticks_per_revolution == 1980U);

  MotorController model_reset(FullRangeTestMotorConfiguration());
  std::array<std::uint32_t, kMotorCount> model_counters{};
  model_reset.ControlStep(model_counters);
  model_counters[0] = 10U;
  model_reset.ControlStep(model_counters);
  CHECK(std::fabs(model_reset.channels()[0].measured_rps) > 0.0F);
  CHECK(model_reset.SetModel(MotorModel::kJgb37).result.ok());
  CHECK_NEAR(model_reset.channels()[0].measured_rps, 0.0F, 0.0001F);
  const std::int64_t count_before_rebaseline =
      model_reset.channels()[0].encoder_count;
  model_reset.ControlStep(model_counters);
  CHECK_NEAR(model_reset.channels()[0].measured_rps, 0.0F, 0.0001F);
  CHECK(model_reset.channels()[0].encoder_count == count_before_rebaseline);

  CHECK(MotorController::SignedCounterDelta(2U, 65534U, 16U) == 4);
  CHECK(MotorController::SignedCounterDelta(65534U, 2U, 16U) == -4);
  CHECK(MotorController::SignedCounterDelta(1U, 0xffffffffU, 32U) == 2);

  MotorCommand drive{};
  drive.update_mask = 1U;
  drive.target_rps[0] = 1.0F;
  CHECK(controller.AcceptCommand(drive, 400U).ok());
  const std::array<std::uint32_t, kMotorCount> counters{};
  const auto outputs = controller.ControlStep(counters);
  CHECK(outputs[0] >= kMotorOutputDeadbandPermille);
  CHECK(outputs[0] <= kMotorOutputLimitPermille);
  controller.SetSessionActive(false);
  CHECK(controller.channels()[0].output_permille == 0);
  CHECK(controller.watchdog_stop_mask() == 1U);

  MotorController independent(FullRangeTestMotorConfiguration());
  independent.SetSessionActive(true);
  MotorCommand both{};
  both.update_mask = 3U;
  both.target_rps[0] = 1.0F;
  both.target_rps[1] = 1.0F;
  CHECK(independent.AcceptCommand(both, 100U).ok());
  MotorCommand refresh_first{};
  refresh_first.update_mask = 1U;
  refresh_first.target_rps[0] = 1.0F;
  CHECK(independent.AcceptCommand(refresh_first, 100000U).ok());
  independent.EvaluateLeases(198100U);
  CHECK(independent.channels()[0].armed);
  CHECK(!independent.channels()[1].armed);
  CHECK(independent.watchdog_stop_mask() == 2U);
  independent.EvaluateLeases(297999U);
  CHECK(independent.channels()[0].armed);
  independent.EvaluateLeases(298000U);
  CHECK(!independent.channels()[0].armed);

  MotorController zero_target;
  zero_target.SetSessionActive(true);
  MotorCommand zero{};
  zero.update_mask = 1U;
  zero.target_rps[0] = 0.0F;
  CHECK(zero_target.AcceptCommand(zero, 10U).ok());
  zero_target.EvaluateLeases(1000000U);
  CHECK(zero_target.watchdog_stop_mask() == 0U);

  // The default configuration is deliberately motor-locked. Every valid
  // subset and every model rejects motion atomically, while selected zero
  // remains a valid stop and a model change cannot bypass the lock.
  constexpr std::array<MotorModel, 4> kModels{
      MotorModel::kJgb520, MotorModel::kJgb37, MotorModel::kJga27,
      MotorModel::kJgb528};
  MotorController locked;
  locked.SetSessionActive(true);
  for (MotorModel model : kModels) {
    CHECK(locked.SetModel(model).result.ok());
    for (std::uint16_t raw_mask = 1U; raw_mask <= kAllMotorMask; ++raw_mask) {
      MotorCommand rejected{};
      rejected.update_mask = static_cast<std::uint8_t>(raw_mask);
      MotorCommand zero_selected{};
      zero_selected.update_mask = rejected.update_mask;
      for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
        const auto bit = static_cast<std::uint8_t>(1U << motor);
        if ((rejected.update_mask & bit) != 0U) {
          rejected.target_rps[motor] = 0.1F;
        }
      }
      CHECK(locked.AcceptCommand(rejected, 1000U).code ==
            ResultCode::kUnsupported);
      for (const MotorChannelState& channel : locked.channels()) {
        CHECK(!channel.armed);
        CHECK_NEAR(channel.target_rps, 0.0F, 0.0001F);
      }
      CHECK(locked.AcceptCommand(zero_selected, 2000U).ok());
    }
  }
  MotorCommand mixed_locked{};
  mixed_locked.update_mask = 3U;
  mixed_locked.target_rps[0] = 0.0F;
  mixed_locked.target_rps[1] = 0.1F;
  CHECK(locked.AcceptCommand(mixed_locked, 3000U).code ==
        ResultCode::kUnsupported);
  for (const MotorChannelState& channel : locked.channels()) {
    CHECK(!channel.armed);
    CHECK_NEAR(channel.target_rps, 0.0F, 0.0001F);
  }

  // Commissioning limits are independent hard caps below model limits.
  const MotorControlConfiguration commissioning =
      CommissioningMotorControlConfiguration();
  CHECK(commissioning.mode == MotorControlMode::kDirectionCheck);
  CHECK_NEAR(commissioning.maximum_accepted_rps, 0.25F, 0.0001F);
  CHECK(commissioning.output_limit_permille == 1000);
  MotorController capped(commissioning);
  capped.SetSessionActive(true);
  MotorCommand limited{};
  limited.update_mask = 1U;
  limited.target_rps[0] = 0.25F;
  CHECK(capped.AcceptCommand(limited, 0U).ok());
  std::array<std::uint32_t, kMotorCount> stationary{};
  for (std::size_t sample = 0U; sample < 100U; ++sample) {
    const auto capped_output = capped.ControlStep(stationary);
    CHECK(capped_output[0] == kMotorDirectionCheckDutyPermille);
    CHECK(capped.channels()[0].output_permille ==
          kMotorDirectionCheckDutyPermille);
  }
  limited.target_rps[0] = -0.25F;
  CHECK(capped.AcceptCommand(limited, 50000U).ok());
  limited.target_rps[0] = 0.251F;
  CHECK(capped.AcceptCommand(limited, 100000U).code == ResultCode::kOutOfRange);
  CHECK_NEAR(capped.channels()[0].target_rps, -0.25F, 0.0001F);
  limited.target_rps[0] = -0.251F;
  CHECK(capped.AcceptCommand(limited, 100001U).code == ResultCode::kOutOfRange);
  CHECK_NEAR(capped.channels()[0].target_rps, -0.25F, 0.0001F);
  CHECK(capped.command_rejection_count(0U) == 2U);
  CHECK(capped.command_rejection_count(1U) == 0U);
  CHECK(capped.command_rejection_count(2U) == 0U);
  CHECK(capped.command_rejection_count(3U) == 0U);
  capped.EvaluateLeases(50000U + kMotorLeaseExpiryUs);
  CHECK(!capped.channels()[0].armed);

  MotorController overspeed(commissioning);
  overspeed.SetSessionActive(true);
  limited.target_rps[0] = 0.1F;
  CHECK(overspeed.AcceptCommand(limited, 0U).ok());
  stationary.fill(100U);
  CHECK(overspeed.ControlStep(stationary)[0] ==
        kMotorDirectionCheckDutyPermille);
  stationary[0] = 0U;
  CHECK(overspeed.ControlStep(stationary)[0] == 0);
  CHECK(!overspeed.channels()[0].armed);
  CHECK(overspeed.watchdog_stop_mask() == 1U);

  MotorController rejection_accounting(commissioning);
  rejection_accounting.RecordRejectedCommand(0U);
  rejection_accounting.RecordRejectedCommand(0x10U);
  rejection_accounting.RecordRejectedCommand(0x15U);
  for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
    CHECK(rejection_accounting.command_rejection_count(motor) == 0U);
  }
  rejection_accounting.RecordRejectedCommand(0x05U);
  CHECK(rejection_accounting.command_rejection_count(0U) == 1U);
  CHECK(rejection_accounting.command_rejection_count(1U) == 0U);
  CHECK(rejection_accounting.command_rejection_count(2U) == 1U);
  CHECK(rejection_accounting.command_rejection_count(3U) == 0U);
  CHECK(!capped.channels()[0].armed);
  CHECK(capped.lease_expiry_count(0U) == 1U);

  SetMotorPidCommand pid_update{};
  pid_update.update_mask = 3U;
  pid_update.proportional_gain[0] = 1000.0F;
  pid_update.proportional_gain[1] = 500.0F;
  pid_update.velocity_filter_new_weight[0] = 1.0F;
  pid_update.velocity_filter_new_weight[1] = 1.0F;
  CHECK(locked.SetPid(pid_update).result.code == ResultCode::kUnsupported);
  CHECK(capped.SetPid(pid_update).result.code == ResultCode::kUnsupported);

  MotorController pid_controller(FullRangeTestMotorConfiguration());
  MotorPidUpdate pid_result = pid_controller.SetPid(pid_update);
  CHECK(pid_result.result.ok());
  CHECK(pid_result.applied_mask == 3U);

  SetMotorPidCommand invalid_atomic = pid_update;
  invalid_atomic.proportional_gain[1] = 1000.1F;
  pid_result = pid_controller.SetPid(invalid_atomic);
  CHECK(pid_result.result.code == ResultCode::kOutOfRange);
  CHECK(pid_result.applied_mask == 0U);

  pid_controller.SetSessionActive(true);
  MotorCommand pid_drive{};
  pid_drive.update_mask = 3U;
  pid_drive.target_rps[0] = 0.5F;
  pid_drive.target_rps[1] = 0.5F;
  CHECK(pid_controller.AcceptCommand(pid_drive, 0U).ok());
  const auto overridden_output = pid_controller.ControlStep(stationary);
  CHECK(overridden_output[0] == 500);
  CHECK(overridden_output[1] == kMotorOutputDeadbandPermille);
  CHECK(pid_controller.SetPid(pid_update).result.code == ResultCode::kBusy);

  MotorCommand pid_stop = pid_drive;
  pid_stop.target_rps.fill(0.0F);
  CHECK(pid_controller.AcceptCommand(pid_stop, 1U).ok());
  CHECK(pid_controller.SetPid(pid_update).result.ok());

  // A transport/session loss disarms and clears PID history, but the volatile
  // gain override survives reconnection until reset or a model change.
  pid_controller.SetSessionActive(false);
  pid_controller.SetSessionActive(true);
  CHECK(pid_controller.AcceptCommand(pid_drive, 2U).ok());
  CHECK(pid_controller.ControlStep(stationary)[0] == 500);
  CHECK(pid_controller.AcceptCommand(pid_stop, 3U).ok());
  CHECK(pid_controller.SetModel(MotorModel::kJgb37).result.ok());
  CHECK(pid_controller.AcceptCommand(pid_drive, 4U).ok());
  CHECK(pid_controller.ControlStep(stationary)[0] ==
        kMotorOutputDeadbandPermille);

  // Measured motion alone blocks all-channel updates, even while disarmed.
  MotorController moving_pid(FullRangeTestMotorConfiguration());
  moving_pid.ControlStep(stationary);
  stationary[0] = 1U;
  moving_pid.ControlStep(stationary);
  CHECK(std::fabs(moving_pid.channels()[0].measured_rps) >=
        kMotorPidUpdateMaximumMeasuredRps);
  CHECK(moving_pid.SetPid(pid_update).result.code == ResultCode::kBusy);
  CHECK(moving_pid.SetModel(MotorModel::kJgb37).result.ok());
  CHECK(moving_pid.SetPid(pid_update).result.ok());

  MotorController saturated_pid(FullRangeTestMotorConfiguration());
  SetMotorPidCommand saturation_gains{};
  saturation_gains.update_mask = 1U;
  saturation_gains.proportional_gain[0] = 1000.0F;
  saturation_gains.velocity_filter_new_weight[0] = 1.0F;
  CHECK(saturated_pid.SetPid(saturation_gains).result.ok());
  saturated_pid.SetSessionActive(true);
  MotorCommand saturation_drive{};
  saturation_drive.update_mask = 1U;
  saturation_drive.target_rps[0] = 6.0F;
  CHECK(saturated_pid.AcceptCommand(saturation_drive, 0U).ok());
  stationary.fill(0U);
  CHECK(saturated_pid.ControlStep(stationary)[0] == 1000);

  constexpr std::array<float, 4> kProfileLimits{1.5F, 3.0F, 6.0F, 1.1F};
  MotorController profile_limits(FullRangeTestMotorConfiguration());
  profile_limits.SetSessionActive(true);
  for (std::size_t index = 0U; index < kModels.size(); ++index) {
    CHECK(profile_limits.SetModel(kModels[index]).result.ok());
    CHECK_NEAR(profile_limits.profile().pid.proportional_gain, 250.0F, 0.0001F);
    CHECK_NEAR(profile_limits.profile().pid.integral_gain, 0.1F, 0.0001F);
    CHECK_NEAR(profile_limits.profile().pid.derivative_gain, 0.5F, 0.0001F);
    CHECK_NEAR(profile_limits.profile().pid.velocity_filter_new_weight, 0.5F,
               0.0001F);
    MotorCommand limit_command{};
    limit_command.update_mask = 1U;
    limit_command.target_rps[0] = kProfileLimits[index];
    CHECK(profile_limits.AcceptCommand(limit_command, 10U).ok());
    limit_command.target_rps[0] = 0.0F;
    CHECK(profile_limits.AcceptCommand(limit_command, 11U).ok());
    limit_command.target_rps[0] = std::nextafter(kProfileLimits[index], 7.0F);
    CHECK(profile_limits.AcceptCommand(limit_command, 12U).code ==
          ResultCode::kOutOfRange);
  }

  // Legacy evidence gives JGA27 the opposite provisional model polarity. The
  // per-channel wiring sign multiplies this model sign; neither is claimed as
  // HIL-qualified by this portable test.
  CHECK(MotorController::ProvisionalModelEncoderPolarity(MotorModel::kJga27) ==
        -1);
  CHECK(MotorController::ProvisionalModelEncoderPolarity(MotorModel::kJgb520) ==
        1);
  CHECK(MotorController::ProvisionalModelEncoderPolarity(MotorModel::kJgb37) ==
        1);
  CHECK(MotorController::ProvisionalModelEncoderPolarity(MotorModel::kJgb528) ==
        1);
  MotorController polarity(FullRangeTestMotorConfiguration());
  polarity.ControlStep(stationary);
  stationary[0] = 10U;
  polarity.ControlStep(stationary);
  CHECK(polarity.channels()[0].encoder_count == -10);
  MotorControlConfiguration reversed_wiring = FullRangeTestMotorConfiguration();
  reversed_wiring.channel_wiring_sign[0] = -1;
  MotorController compensated(reversed_wiring);
  stationary.fill(0U);
  compensated.ControlStep(stationary);
  stationary[0] = 10U;
  compensated.ControlStep(stationary);
  CHECK(compensated.channels()[0].encoder_count == 10);
}

void TestPositionalPidController() {
  const std::array<std::uint32_t, kMotorCount> stationary{};
  auto configure_pid = [](MotorController* controller, float kp, float ki,
                          float kd, float filter_weight = 1.0F) {
    SetMotorPidCommand gains{};
    gains.update_mask = 1U;
    gains.proportional_gain[0] = kp;
    gains.integral_gain[0] = ki;
    gains.derivative_gain[0] = kd;
    gains.velocity_filter_new_weight[0] = filter_weight;
    CHECK(controller->SetPid(gains).result.ok());
    controller->SetSessionActive(true);
  };
  auto command_speed = [](MotorController* controller, float target_rps,
                          std::uint32_t now_us = 0U) {
    MotorCommand command{};
    command.update_mask = 1U;
    command.target_rps[0] = target_rps;
    CHECK(controller->AcceptCommand(command, now_us).ok());
  };

  // P uses the current error directly: u = Kp * e.
  MotorController proportional(FullRangeTestMotorConfiguration());
  configure_pid(&proportional, 400.0F, 0.0F, 0.0F);
  command_speed(&proportional, 1.0F);
  CHECK(proportional.ControlStep(stationary)[0] == 400);
  command_speed(&proportional, 0.75F, 1U);
  CHECK(proportional.ControlStep(stationary)[0] == 300);

  // I stores the time integral of error. Five 10 ms samples at 6 RPS with
  // Ki=1000 produce 1000 * (6 * 0.05) = 300 permille.
  MotorController integral(FullRangeTestMotorConfiguration());
  configure_pid(&integral, 0.0F, 1000.0F, 0.0F);
  command_speed(&integral, 6.0F);
  std::int16_t integral_output = 0;
  for (std::size_t sample = 0U; sample < 5U; ++sample) {
    integral_output = integral.ControlStep(stationary, 10000U)[0];
  }
  CHECK(integral_output == 300);

  MotorController longer_period(FullRangeTestMotorConfiguration());
  configure_pid(&longer_period, 0.0F, 1000.0F, 0.0F);
  command_speed(&longer_period, 6.0F);
  for (std::size_t sample = 0U; sample < 3U; ++sample) {
    integral_output = longer_period.ControlStep(stationary, 20000U)[0];
  }
  CHECK(integral_output == 360);

  // D is the first difference of error. A 1.0 RPS step at 10 ms with Kd=6
  // produces 600; changing the target to 0.5 RPS produces -300.
  MotorController derivative(FullRangeTestMotorConfiguration());
  configure_pid(&derivative, 0.0F, 0.0F, 6.0F);
  command_speed(&derivative, 1.0F);
  CHECK(derivative.ControlStep(stationary)[0] == 600);
  command_speed(&derivative, 0.5F, 1U);
  CHECK(derivative.ControlStep(stationary)[0] == -300);

  MotorController combined(FullRangeTestMotorConfiguration());
  configure_pid(&combined, 300.0F, 1000.0F, 2.0F);
  command_speed(&combined, 1.0F);
  CHECK(combined.ControlStep(stationary)[0] == 510);

  // Saturation rejects only an integral update that would push farther into
  // the active limit. The next smaller same-sign command therefore starts
  // from zero stored integral: 1000*0.5 + 1000*(0.5*0.01) = 505.
  MotorController positive_windup(FullRangeTestMotorConfiguration());
  configure_pid(&positive_windup, 1000.0F, 1000.0F, 0.0F);
  command_speed(&positive_windup, 6.0F);
  CHECK(positive_windup.ControlStep(stationary)[0] == 1000);
  command_speed(&positive_windup, 0.5F, 1U);
  CHECK(positive_windup.ControlStep(stationary)[0] == 505);

  MotorController negative_windup(FullRangeTestMotorConfiguration());
  configure_pid(&negative_windup, 1000.0F, 1000.0F, 0.0F);
  command_speed(&negative_windup, -6.0F);
  CHECK(negative_windup.ControlStep(stationary)[0] == -1000);
  command_speed(&negative_windup, -0.5F, 1U);
  CHECK(negative_windup.ControlStep(stationary)[0] == -505);

  MotorController deadband(FullRangeTestMotorConfiguration());
  configure_pid(&deadband, 100.0F, 0.0F, 0.0F);
  command_speed(&deadband, 1.0F);
  CHECK(deadband.ControlStep(stationary)[0] == kMotorOutputDeadbandPermille);
  command_speed(&deadband, -1.0F, 1U);
  CHECK(deadband.ControlStep(stationary)[0] == -kMotorOutputDeadbandPermille);

  // The PID error uses filtered RPS. For default JGA27 polarity, decrementing
  // the raw counter by one yields positive instantaneous speed.
  MotorController filtered(FullRangeTestMotorConfiguration());
  configure_pid(&filtered, 500.0F, 0.0F, 0.0F, 0.5F);
  command_speed(&filtered, 1.0F);
  std::array<std::uint32_t, kMotorCount> filter_counters{};
  filter_counters[0] = 100U;
  CHECK(filtered.ControlStep(filter_counters)[0] == 500);
  filter_counters[0] = 99U;
  CHECK_NEAR(filtered.ControlStep(filter_counters)[0], 476, 0.0F);
  CHECK_NEAR(filtered.channels()[0].measured_rps, 0.0480769F, 0.000001F);

  // Stop and lease-expiry paths clear positional I and D history.
  MotorController reset(FullRangeTestMotorConfiguration());
  configure_pid(&reset, 0.0F, 1000.0F, 0.0F);
  command_speed(&reset, 6.0F);
  for (std::size_t sample = 0U; sample < 5U; ++sample) {
    integral_output = reset.ControlStep(stationary)[0];
  }
  CHECK(integral_output == 300);
  command_speed(&reset, 0.0F, 10U);
  command_speed(&reset, 6.0F, 11U);
  CHECK(reset.ControlStep(stationary)[0] == kMotorOutputDeadbandPermille);
  reset.EvaluateLeases(11U + kMotorLeaseExpiryUs);
  CHECK(!reset.channels()[0].armed);
  command_speed(&reset, 6.0F, 12U + kMotorLeaseExpiryUs);
  CHECK(reset.ControlStep(stationary)[0] == kMotorOutputDeadbandPermille);
  reset.SetSessionActive(false);
  reset.SetSessionActive(true);
  command_speed(&reset, 6.0F, 13U + kMotorLeaseExpiryUs);
  CHECK(reset.ControlStep(stationary)[0] == kMotorOutputDeadbandPermille);
}

void TestMotorLeaseBoundariesAndScheduleModel() {
  constexpr std::uint32_t kSafetyLimitUs = 200000U;
  constexpr std::uint32_t kNominalReleasePeriodUs = 1000U;
  constexpr std::uint32_t kMaximumEvaluationGapUs = 2000U;

  MotorCommand all_motors{};
  all_motors.update_mask = kAllMotorMask;
  all_motors.target_rps.fill(1.0F);

  // Check every integer-microsecond age on the lower lease boundary.
  MotorController every_age(FullRangeTestMotorConfiguration());
  every_age.SetSessionActive(true);
  constexpr std::uint32_t kAcceptedAtUs = 1000U;
  CHECK(every_age.AcceptCommand(all_motors, kAcceptedAtUs).ok());
  const std::array<std::uint32_t, kMotorCount> counters{};
  const auto running_outputs = every_age.ControlStep(counters);
  for (const std::int16_t output : running_outputs) {
    CHECK(output != 0);
  }
  for (std::uint32_t age_us = 0U; age_us < kMotorLeaseExpiryUs; ++age_us) {
    every_age.EvaluateLeases(kAcceptedAtUs + age_us);
    for (const MotorChannelState& channel : every_age.channels()) {
      CHECK(channel.armed);
      CHECK(channel.output_permille != 0);
    }
  }
  every_age.EvaluateLeases(kAcceptedAtUs + kMotorLeaseExpiryUs);
  CHECK(every_age.watchdog_stop_mask() == kAllMotorMask);
  for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
    CHECK(!every_age.channels()[motor].armed);
    CHECK(every_age.channels()[motor].output_permille == 0);
    CHECK(every_age.lease_expiry_count(motor) == 1U);
  }

  // Every motor subset expires atomically at the inclusive 198 ms boundary;
  // unselected channels remain untouched.
  for (std::uint16_t raw_mask = 1U; raw_mask <= kAllMotorMask; ++raw_mask) {
    MotorController subset(FullRangeTestMotorConfiguration());
    subset.SetSessionActive(true);
    MotorCommand command{};
    command.update_mask = static_cast<std::uint8_t>(raw_mask);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      const auto bit = static_cast<std::uint8_t>(1U << motor);
      if ((command.update_mask & bit) != 0U) {
        command.target_rps[motor] = 1.0F;
      }
    }
    CHECK(subset.AcceptCommand(command, 37U).ok());
    subset.EvaluateLeases(37U + kMotorLeaseExpiryUs - 1U);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      const auto bit = static_cast<std::uint8_t>(1U << motor);
      CHECK(subset.channels()[motor].armed ==
            ((command.update_mask & bit) != 0U));
    }
    subset.EvaluateLeases(37U + kMotorLeaseExpiryUs);
    CHECK(subset.watchdog_stop_mask() == command.update_mask);
    for (std::size_t motor = 0U; motor < kMotorCount; ++motor) {
      const auto bit = static_cast<std::uint8_t>(1U << motor);
      CHECK(!subset.channels()[motor].armed);
      CHECK(subset.lease_expiry_count(motor) ==
            ((command.update_mask & bit) != 0U ? 1U : 0U));
    }
  }

  // Exhaust every acceptance phase relative to the nominal 1 kHz TIM7
  // release. Model the worst permitted completion jitter by skipping one
  // release, producing a completed-evaluation gap of exactly 2 ms.
  for (std::uint32_t phase_us = 0U; phase_us < kNominalReleasePeriodUs;
       ++phase_us) {
    const std::uint32_t expiry_us = phase_us + kMotorLeaseExpiryUs;
    const std::uint32_t first_release_at_or_after_expiry =
        ((expiry_us + kNominalReleasePeriodUs - 1U) / kNominalReleasePeriodUs) *
        kNominalReleasePeriodUs;
    const std::uint32_t delayed_evaluation_us =
        first_release_at_or_after_expiry + kNominalReleasePeriodUs;
    const std::uint32_t prior_evaluation_us =
        delayed_evaluation_us - kMaximumEvaluationGapUs;

    MotorController phased(FullRangeTestMotorConfiguration());
    phased.SetSessionActive(true);
    CHECK(phased.AcceptCommand(all_motors, phase_us).ok());
    phased.EvaluateLeases(prior_evaluation_us);
    for (const MotorChannelState& channel : phased.channels()) {
      CHECK(channel.armed);
    }
    phased.EvaluateLeases(delayed_evaluation_us);
    const std::uint32_t stop_age_us = delayed_evaluation_us - phase_us;
    CHECK(delayed_evaluation_us - prior_evaluation_us <=
          kMaximumEvaluationGapUs);
    CHECK(stop_age_us >= kMotorLeaseExpiryUs);
    CHECK(stop_age_us <= kSafetyLimitUs);
    for (const MotorChannelState& channel : phased.channels()) {
      CHECK(!channel.armed);
      CHECK(channel.output_permille == 0);
    }
  }

  // Unsigned age arithmetic preserves both sides of the boundary across the
  // 32-bit microsecond clock wrap.
  MotorController wrapped(FullRangeTestMotorConfiguration());
  wrapped.SetSessionActive(true);
  constexpr std::uint32_t kWrappedAcceptanceUs = 0xffffff00U;
  CHECK(wrapped.AcceptCommand(all_motors, kWrappedAcceptanceUs).ok());
  const std::uint32_t wrapped_expiry_us =
      kWrappedAcceptanceUs + kMotorLeaseExpiryUs;
  wrapped.EvaluateLeases(wrapped_expiry_us - 1U);
  for (const MotorChannelState& channel : wrapped.channels()) {
    CHECK(channel.armed);
  }
  wrapped.EvaluateLeases(wrapped_expiry_us);
  CHECK(wrapped.watchdog_stop_mask() == kAllMotorMask);
}

void TestPwmServoController() {
  PwmServoController controller;
  CHECK(controller.state().output_pulse_width_us[0] == 1500U);
  PwmFrameUpdate submitted = controller.PrepareFollowingFrame();
  const auto prepare_pending = [&controller, &submitted]() {
    const PwmFrameUpdate replacement =
        controller.PreparePendingFrame(submitted);
    controller.ConfirmPendingFrameSubmitted(replacement);
    submitted = replacement;
  };
  const auto commit_boundary = [&controller, &submitted]() {
    controller.CommitFrame(submitted);
    submitted = controller.PrepareFollowingFrame();
  };

  PwmServoCommand command{};
  command.update_mask = 1U;
  command.duration_ms = 40U;
  command.pulse_width_us[0] = 1601U;
  CHECK(controller.AcceptCommand(command).ok());
  CHECK(controller.state().target_pulse_width_us[0] == 1601U);

  prepare_pending();
  CHECK(submitted.output_pulse_width_us()[0] == 1500U);
  commit_boundary();  // B0 starts interpolation without installing step one.
  CHECK(controller.state().output_pulse_width_us[0] == 1500U);
  CHECK(controller.state().moving_mask == 1U);
  CHECK(submitted.output_pulse_width_us()[0] == 1551U);
  commit_boundary();  // B1.
  CHECK(controller.state().output_pulse_width_us[0] == 1551U);
  CHECK(submitted.output_pulse_width_us()[0] == 1601U);
  commit_boundary();  // B2, 40 ms after B0.
  CHECK(controller.state().output_pulse_width_us[0] == 1601U);
  CHECK(controller.state().moving_mask == 0U);

  command.pulse_width_us[0] = 1500U;
  CHECK(controller.AcceptCommand(command).ok());
  prepare_pending();
  commit_boundary();  // B0 remains at 1601 us.
  CHECK(submitted.output_pulse_width_us()[0] == 1550U);
  commit_boundary();
  commit_boundary();
  CHECK(controller.state().output_pulse_width_us[0] == 1500U);

  PwmServoOffsetCommand offsets{};
  offsets.update_mask = 1U;
  offsets.offset_us[0] = 100;
  CHECK(controller.StageOffsets(offsets).ok());
  CHECK(controller.offset_commit_pending());
  prepare_pending();
  CHECK(submitted.offset_commit_mask() == 1U);
  CHECK(submitted.output_pulse_width_us()[0] == 1600U);
  CHECK(controller.offset_commit_pending());
  commit_boundary();
  CHECK(!controller.offset_commit_pending());
  CHECK(controller.state().output_pulse_width_us[0] == 1600U);
  CHECK(controller.StageOffsets(offsets).ok());
  CHECK(!controller.offset_commit_pending());

  offsets.update_mask = 3U;
  offsets.offset_us[0] = 100;
  offsets.offset_us[1] = -50;
  CHECK(controller.StageOffsets(offsets).ok());
  prepare_pending();
  CHECK(submitted.offset_commit_mask() == 3U);
  commit_boundary();
  CHECK(controller.state().offset_us[0] == 100);
  CHECK(controller.state().offset_us[1] == -50);

  // Session loss freezes an in-progress trajectory at the exact active pulse.
  command.duration_ms = 100U;
  command.pulse_width_us[0] = 1800U;
  CHECK(controller.AcceptCommand(command).ok());
  prepare_pending();
  commit_boundary();  // B0.
  commit_boundary();  // B1 is now physical; B2 is only submitted.
  const std::uint16_t held_output = controller.state().output_pulse_width_us[0];
  CHECK(submitted.output_pulse_width_us()[0] > held_output);
  CHECK(controller.state().moving_mask == 1U);
  const auto held_offsets = controller.state().offset_us;
  controller.HoldCurrentOutputAndCancelPending(
      controller.state().output_pulse_width_us, held_offsets);
  submitted = controller.PrepareFollowingFrame();
  CHECK(controller.state().moving_mask == 0U);
  CHECK(controller.state().target_pulse_width_us[0] == held_output - 100U);
  for (std::size_t boundary = 0U; boundary < 8U; ++boundary) {
    commit_boundary();
    CHECK(controller.state().output_pulse_width_us[0] == held_output);
  }

  // An offset not yet committed at a common frame boundary is canceled too.
  offsets.update_mask = 1U;
  offsets.offset_us[0] = -100;
  CHECK(controller.StageOffsets(offsets).ok());
  CHECK(controller.offset_commit_pending());
  prepare_pending();
  CHECK(submitted.offset_commit_mask() == 1U);
  const auto committed_offsets = controller.state().offset_us;
  controller.HoldCurrentOutputAndCancelPending(
      controller.state().output_pulse_width_us, committed_offsets);
  submitted = controller.PrepareFollowingFrame();
  CHECK(!controller.offset_commit_pending());
  CHECK(submitted.offset_commit_mask() == 0U);
  CHECK(controller.state().offset_us[0] == 100);
  commit_boundary();
  CHECK(controller.state().output_pulse_width_us[0] == held_output);
}

void TestButtons() {
  ButtonController buttons;
  std::array<bool, kButtonCount> raw{};
  ButtonEvent event{};

  raw[0] = true;
  buttons.Sample(raw, 0U);
  CHECK(!buttons.PopEvent(&event));
  buttons.Sample(raw, 30U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kPressed);

  raw[0] = false;
  buttons.Sample(raw, 60U);
  buttons.Sample(raw, 90U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kReleaseFromShortPress);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kClick);

  raw[0] = true;
  buttons.Sample(raw, 120U);
  buttons.Sample(raw, 150U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kPressed);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kDoubleClick);

  raw[0] = false;
  buttons.Sample(raw, 180U);
  buttons.Sample(raw, 210U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kReleaseFromShortPress);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kClick);
  raw[0] = true;
  buttons.Sample(raw, 240U);
  buttons.Sample(raw, 270U);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kPressed);
  CHECK(buttons.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kTripleClick);

  ButtonController long_press;
  raw = {};
  raw[1] = true;
  long_press.Sample(raw, 0U);
  long_press.Sample(raw, 30U);
  CHECK(long_press.PopEvent(&event));
  for (std::uint32_t now_ms = 60U; now_ms <= 1530U; now_ms += 30U) {
    long_press.Sample(raw, now_ms);
  }
  CHECK(long_press.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kLongPress);
  CHECK(event.timestamp_ms == 1530U);
  for (std::uint32_t now_ms = 1560U; now_ms <= 1950U; now_ms += 30U) {
    long_press.Sample(raw, now_ms);
  }
  CHECK(long_press.PopEvent(&event));
  CHECK(event.event == ButtonEventType::kLongPressRepeat);
  CHECK(event.timestamp_ms == 1950U);
}

ButtonEvent SequencedButtonEvent(std::uint32_t sequence) {
  return {sequence,
          static_cast<std::uint8_t>(((sequence - 1U) % kButtonCount) + 1U),
          ButtonEventType::kClick};
}

void CheckButtonEventRange(ButtonEventQueue* queue, std::uint32_t first,
                           std::uint32_t last) {
  ButtonEvent event{};
  for (std::uint32_t expected = first; expected <= last; ++expected) {
    CHECK(queue->TryPop(&event));
    CHECK(event.timestamp_ms == expected);
    CHECK(event.button_id == SequencedButtonEvent(expected).button_id);
    CHECK(event.event == ButtonEventType::kClick);
  }
  CHECK(!queue->TryPop(&event));
}

void TestButtonEventQueueBounds() {
  ButtonEventQueue exactly_full;
  for (std::uint32_t sequence = 1U; sequence <= 16U; ++sequence) {
    CHECK(!exactly_full.PushDropOldest(SequencedButtonEvent(sequence)));
  }
  CHECK(exactly_full.dropped_count() == 0U);
  CheckButtonEventRange(&exactly_full, 1U, 16U);

  ButtonEventQueue seventeen;
  for (std::uint32_t sequence = 1U; sequence <= 17U; ++sequence) {
    CHECK(seventeen.PushDropOldest(SequencedButtonEvent(sequence)) ==
          (sequence == 17U));
  }
  CHECK(seventeen.dropped_count() == 1U);
  CheckButtonEventRange(&seventeen, 2U, 17U);

  ButtonEventQueue thirty_two;
  for (std::uint32_t sequence = 1U; sequence <= 32U; ++sequence) {
    CHECK(thirty_two.PushDropOldest(SequencedButtonEvent(sequence)) ==
          (sequence > 16U));
  }
  CHECK(thirty_two.dropped_count() == 16U);
  CheckButtonEventRange(&thirty_two, 17U, 32U);
}

void TestBatteryMonitor() {
  BatteryMonitor battery;
  BatteryUpdate update = battery.AddSample(10000U, true, 0U);
  CHECK(update.state.valid);
  CHECK(update.state.voltage_mv == 10000U);
  update = battery.AddSample(12000U, true, 50U);
  CHECK(update.state.voltage_mv == 10100U);
  update = battery.AddSample(0U, false, 100U);
  CHECK(!update.state.valid && update.state.voltage_mv == 0U);

  BatteryMonitor battery_presence;
  update = battery_presence.AddSample(kBatteryPresentMinimumMv - 1U, true, 0U);
  CHECK(!update.state.valid && update.state.voltage_mv == 0U);
  CHECK(!update.state.below_threshold && !update.request_alarm_pattern);
  update = battery_presence.AddSample(kBatteryPresentMinimumMv, true, 50U);
  CHECK(update.state.valid &&
        update.state.voltage_mv == kBatteryPresentMinimumMv);

  BatteryMonitor low_battery;
  for (std::uint32_t now_ms = 0U; now_ms <= 10000U; now_ms += 50U) {
    update = low_battery.AddSample(6000U, true, now_ms);
  }
  CHECK(update.state.below_threshold);
  CHECK(update.request_alarm_pattern);
  update = low_battery.AddSample(6000U, true, 20000U);
  CHECK(update.request_alarm_pattern);

  bool cleared = false;
  for (std::uint32_t now_ms = 20050U; now_ms <= 23000U; now_ms += 50U) {
    update = low_battery.AddSample(20000U, true, now_ms);
    if (!update.state.below_threshold) {
      cleared = true;
      break;
    }
  }
  CHECK(cleared);

  const BatteryThresholdUpdate rejected = low_battery.SetLowThreshold(4999U);
  CHECK(rejected.result.code == ResultCode::kOutOfRange);
  const BatteryThresholdUpdate accepted = low_battery.SetLowThreshold(5000U);
  CHECK(accepted.result.ok() && accepted.active_threshold_mv == 5000U);
}

void TestPatterns() {
  LedController leds;
  LedCommand led{1U, 100U, 50U, 2U};
  CHECK(leds.AcceptCommand(led, 0U).ok());
  CHECK(leds.Update(0U)[0]);
  CHECK(!leds.Update(100U)[0]);
  CHECK(leds.Update(150U)[0]);
  CHECK(!leds.Update(300U)[0]);

  BuzzerController buzzer;
  const BuzzerCommand host{440U, 100U, 100U, 0U};
  CHECK(buzzer.AcceptHostCommand(host, 0U).ok());
  CHECK(buzzer.Update(0U).frequency_hz == 440U);
  buzzer.TriggerBatteryAlarm(10U);
  BuzzerOutput output = buzzer.Update(10U);
  CHECK(output.frequency_hz == 2100U && output.battery_alarm_active);
  output = buzzer.Update(5010U);
  CHECK(!output.battery_alarm_active);
  CHECK(output.frequency_hz == 440U);
}

void TestBusServoCodecAndScheduler() {
  const std::array<std::uint8_t, 4> arguments{0xe8U, 0x03U, 0xe8U, 0x03U};
  BusServoFrame frame{};
  CHECK(BusServoCodec::BuildFrame(1U, BusServoOpcode::kMoveStop, nullptr, 0U,
                                  nullptr)
            .code == ResultCode::kInvalidArgument);
  CHECK(BusServoCodec::BuildFrame(1U, BusServoOpcode::kMoveStop, nullptr, 1U,
                                  &frame)
            .code == ResultCode::kInvalidArgument);
  CHECK(BusServoCodec::BuildFrame(0U, BusServoOpcode::kMoveStop, nullptr, 0U,
                                  &frame)
            .code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::BuildFrame(255U, BusServoOpcode::kMoveStop, nullptr, 0U,
                                  &frame)
            .code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::BuildFrame(
            1U, BusServoOpcode::kMoveStop, arguments.data(),
            mentor_pi::mcu::kBusServoMaximumArguments + 1U, &frame)
            .code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::BuildFrame(1U, BusServoOpcode::kMoveTimeWrite,
                                  arguments.data(), arguments.size(), &frame)
            .ok());
  CHECK(frame.size == 10U);
  CHECK(frame.bytes[9] == 0x20U);
  ParsedBusServoFrame parsed =
      BusServoCodec::ParseFrame(frame.bytes.data(), frame.size);
  CHECK(parsed.result.ok());
  CHECK(parsed.servo_id == 1U && parsed.argument_count == 4U);

  CHECK(BusServoCodec::ParseFrame(nullptr, 0U).result.code ==
        ResultCode::kInvalidArgument);
  CHECK(BusServoCodec::ParseFrame(frame.bytes.data(), 5U).result.code ==
        ResultCode::kInvalidArgument);
  BusServoFrame malformed = frame;
  malformed.bytes[0] = 0U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 1U);
  malformed = frame;
  malformed.bytes[1] = 0U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 1U);
  malformed = frame;
  malformed.bytes[3] = 2U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 2U);
  malformed = frame;
  malformed.bytes[3] = 12U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 2U);
  malformed = frame;
  malformed.bytes[3] = 3U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.detail == 2U);
  malformed = frame;
  malformed.bytes[2] = 0U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.code == ResultCode::kOutOfRange);
  malformed = frame;
  malformed.bytes[2] = 255U;
  CHECK(BusServoCodec::ParseFrame(malformed.bytes.data(), malformed.size)
            .result.code == ResultCode::kOutOfRange);
  CHECK(BusServoCodec::Checksum(nullptr, 1U) == 0U);
  frame.bytes[9] ^= 1U;
  parsed = BusServoCodec::ParseFrame(frame.bytes.data(), frame.size);
  CHECK(parsed.result.code == ResultCode::kIoError);

  BusServoScheduler scheduler;
  BusServoCommand move{};
  move.count = 2U;
  move.servo_id[0] = 1U;
  move.servo_id[1] = 2U;
  move.position[0] = 100U;
  move.position[1] = 200U;
  move.duration_ms = 500U;
  CHECK(scheduler.SubmitMove(move).result.ok());
  ScheduledBusFrame scheduled = scheduler.BeginFrame();
  CHECK(scheduled.result.ok());
  CHECK(scheduled.kind == ScheduledBusFrameKind::kMove);
  CHECK(scheduled.frame.bytes[2] == 1U);

  BusServoCommand pre_stop_pending{};
  pre_stop_pending.count = 1U;
  pre_stop_pending.servo_id[0] = 5U;
  pre_stop_pending.position[0] = 250U;
  CHECK(scheduler.SubmitMove(pre_stop_pending).result.ok());

  StopBusServosCommand stop{};
  stop.count = 1U;
  stop.servo_id[0] = 9U;
  CHECK(scheduler.AcceptStop(stop).ok());
  BusServoCommand post_stop{};
  post_stop.count = 1U;
  post_stop.servo_id[0] = 7U;
  post_stop.position[0] = 300U;
  CHECK(scheduler.SubmitMove(post_stop).result.ok());
  scheduler.CompleteFrame(true);

  scheduled = scheduler.BeginFrame();
  CHECK(scheduled.kind == ScheduledBusFrameKind::kStop);
  CHECK(scheduled.frame.bytes[2] == 9U);
  scheduler.CompleteFrame(true);
  scheduled = scheduler.BeginFrame();
  CHECK(scheduled.kind == ScheduledBusFrameKind::kMove);
  CHECK(scheduled.frame.bytes[2] == 7U);

  BusServoScheduler edge_scheduler;
  CHECK(!edge_scheduler.has_work());
  CHECK(edge_scheduler.SubmitMove(BusServoCommand{}).result.code ==
        ResultCode::kInvalidArgument);
  CHECK(edge_scheduler.AcceptStop(StopBusServosCommand{}).code ==
        ResultCode::kInvalidArgument);
  const BusMoveAdmission first = edge_scheduler.SubmitMove(move);
  CHECK(first.result.ok() && !first.overwrote_pending);
  const BusMoveAdmission overwritten = edge_scheduler.SubmitMove(move);
  CHECK(overwritten.result.ok() && overwritten.overwrote_pending);
  CHECK(edge_scheduler.move_overwrite_count() == 1U);
  CHECK(edge_scheduler.has_work());
  scheduled = edge_scheduler.BeginFrame();
  CHECK(scheduled.result.ok() && edge_scheduler.frame_in_progress());
  CHECK(edge_scheduler.BeginFrame().result.code == ResultCode::kBusy);
  edge_scheduler.CompleteFrame(false);
  CHECK(!edge_scheduler.has_work());
  edge_scheduler.CompleteFrame(true);

  BusServoScheduler progression;
  CHECK(progression.SubmitMove(move).result.ok());
  CHECK(progression.BeginFrame().batch_index == 0U);
  progression.CompleteFrame(true);
  CHECK(progression.has_work());
  CHECK(progression.BeginFrame().batch_index == 1U);
  progression.CompleteFrame(true);
  CHECK(!progression.has_work());
  CHECK(progression.BeginFrame().result.code == ResultCode::kBusy);

  BusServoScheduler stop_scheduler;
  StopBusServosCommand two_stops{};
  two_stops.count = 2U;
  two_stops.servo_id[0] = 3U;
  two_stops.servo_id[1] = 4U;
  CHECK(stop_scheduler.AcceptStop(two_stops).ok());
  CHECK(stop_scheduler.AcceptStop(two_stops).code == ResultCode::kBusy);
  CHECK(stop_scheduler.BeginFrame().batch_index == 0U);
  stop_scheduler.CompleteFrame(false);
  CHECK(!stop_scheduler.has_work());
  CHECK(stop_scheduler.AcceptStop(two_stops).ok());
  CHECK(stop_scheduler.BeginFrame().batch_index == 0U);
  stop_scheduler.CompleteFrame(true);
  CHECK(stop_scheduler.has_work());
  CHECK(stop_scheduler.BeginFrame().batch_index == 1U);
  stop_scheduler.CompleteFrame(true);
  CHECK(!stop_scheduler.has_work());

  BusServoScheduler cancel_scheduler;
  CHECK(cancel_scheduler.SubmitMove(move).result.ok());
  CHECK(cancel_scheduler.BeginFrame().result.ok());
  cancel_scheduler.CancelAll();
  CHECK(!cancel_scheduler.has_work() && !cancel_scheduler.frame_in_progress());

  BusServoScheduler between_frames;
  CHECK(between_frames.SubmitMove(move).result.ok());
  CHECK(between_frames.BeginFrame().result.ok());
  between_frames.CompleteFrame(true);
  CHECK(between_frames.AcceptStop(stop).ok());
  CHECK(between_frames.BeginFrame().kind == ScheduledBusFrameKind::kStop);
}

void TestCircularDmaPosition() {
  using TinyRing = CircularDmaPosition<8U>;
  static_assert(TinyRing::kHalfSizeBytes == 4U);

  CHECK(TinyRing::IsConsistent(0U, 0U));
  CHECK(TinyRing::IsConsistent(0U, 3U));
  CHECK(!TinyRing::IsConsistent(0U, 4U));
  CHECK(TinyRing::IsConsistent(1U, 4U));
  CHECK(TinyRing::IsConsistent(1U, 7U));
  CHECK(!TinyRing::IsConsistent(1U, 0U));
  CHECK(!TinyRing::IsConsistent(2U, 8U));

  CHECK(TinyRing::Reconstruct(0U, 3U) == 3U);
  CHECK(TinyRing::Reconstruct(1U, 4U) == 4U);
  CHECK(TinyRing::Reconstruct(2U, 0U) == 8U);
  CHECK(TinyRing::Reconstruct(3U, 7U) == 15U);
  CHECK(TinyRing::Reconstruct(4U, 0U) == 16U);

  for (std::uint32_t absolute = 0U; absolute < 1024U; ++absolute) {
    const std::uint32_t boundary_count = absolute / TinyRing::kHalfSizeBytes;
    const std::uint32_t cursor = absolute % 8U;
    CHECK(TinyRing::IsConsistent(boundary_count, cursor));
    CHECK(TinyRing::Reconstruct(boundary_count, cursor) == absolute);
    CHECK(!TinyRing::IsConsistent(boundary_count ^ 1U, cursor));
  }

  // A complete ring between observations remains visible instead of aliasing
  // to zero movement as it would with a cursor-only modulo subtraction.
  const std::uint32_t before = TinyRing::Reconstruct(1U, 6U);
  const std::uint32_t after_one_lap = TinyRing::Reconstruct(3U, 6U);
  CHECK(after_one_lap - before == 8U);

  using TargetRing = CircularDmaPosition<8192U>;
  CHECK(TargetRing::IsConsistent(0xfffffffeU, 4095U));
  CHECK(TargetRing::IsConsistent(0xffffffffU, 4096U));
  const std::uint32_t before_boundary_wrap =
      TargetRing::Reconstruct(0xffffffffU, 8191U);
  const std::uint32_t after_boundary_wrap = TargetRing::Reconstruct(0U, 0U);
  CHECK(after_boundary_wrap - before_boundary_wrap == 1U);
}

void TestUsart1WriteDeadlines() {
  using mentor_pi_mcu::platform::stm32::Usart1WriteDeadlineMs;

  CHECK(Usart1WriteDeadlineMs(23U, 115200U) == 4U);
  CHECK(Usart1WriteDeadlineMs(512U, 115200U) == 47U);
  CHECK(Usart1WriteDeadlineMs(1024U, 115200U) == 91U);
  CHECK(Usart1WriteDeadlineMs(512U, 921600U) == 8U);
  CHECK(Usart1WriteDeadlineMs(1024U, 921600U) == 14U);
  CHECK(Usart1WriteDeadlineMs(512U) == 8U);
}

void TestCircularRxRing() {
  using TinyRing = CircularRxRing<8U>;
  const std::array<std::uint8_t, 8> ring{0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};

  TinyRing exact;
  const TinyRing::ProducerUpdate exact_update = exact.UpdateProducer(4U, true);
  CHECK(exact_update.consistent);
  CHECK(!exact_update.overrun);
  CHECK(exact_update.delta == 4U);
  CHECK(exact_update.occupied == 4U);
  TinyRing::ReadPlan plan = exact.PrepareRead(3U);
  CHECK(!plan.overrun);
  CHECK(plan.copy_length == 3U);
  CHECK(plan.ring_offset == 0U);
  CHECK(plan.first_length == 3U);
  std::array<std::uint8_t, 8> output{};
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK((std::array<std::uint8_t, 3>{output[0], output[1], output[2]}) ==
        (std::array<std::uint8_t, 3>{0U, 1U, 2U}));
  CHECK(exact.CommitRead(plan));
  CHECK(!exact.CommitRead(plan));
  CHECK(exact.consumer_position() == 3U);
  plan = exact.PrepareRead(output.size());
  CHECK(plan.copy_length == 1U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(output[0] == 3U);
  CHECK(exact.CommitRead(plan));

  // DMA producer progress between the unlocked copy and the critical-section
  // commit is safe only while unread occupancy still fits in the ring.
  TinyRing producer_race_safe;
  CHECK(producer_race_safe.UpdateProducer(4U, true).consistent);
  plan = producer_race_safe.PrepareRead(3U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(!producer_race_safe.UpdateProducer(8U, true).overrun);
  CHECK(producer_race_safe.CommitRead(plan));
  CHECK(producer_race_safe.consumer_position() == 3U);

  TinyRing producer_race_overrun;
  CHECK(producer_race_overrun.UpdateProducer(4U, true).consistent);
  plan = producer_race_overrun.PrepareRead(3U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(producer_race_overrun.UpdateProducer(9U, true).overrun);
  CHECK(!producer_race_overrun.CommitRead(plan));
  CHECK(producer_race_overrun.consumer_position() == 0U);

  TinyRing wrapped;
  wrapped.ResetPositions(6U);
  const TinyRing::ProducerUpdate wrapped_update =
      wrapped.UpdateProducer(10U, true);
  CHECK(wrapped_update.delta == 4U);
  plan = wrapped.PrepareRead(4U);
  CHECK(plan.ring_offset == 6U);
  CHECK(plan.first_length == 2U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK((std::array<std::uint8_t, 4>{output[0], output[1], output[2],
                                     output[3]}) ==
        (std::array<std::uint8_t, 4>{6U, 7U, 0U, 1U}));
  CHECK(wrapped.CommitRead(plan));

  TinyRing full;
  const TinyRing::ProducerUpdate full_update = full.UpdateProducer(8U, true);
  CHECK(!full_update.overrun);
  CHECK(full_update.occupied == 8U);
  plan = full.PrepareRead(8U);
  CHECK(!plan.overrun && plan.copy_length == 8U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(output == ring);
  CHECK(full.CommitRead(plan));

  TinyRing overrun;
  const TinyRing::ProducerUpdate overrun_update =
      overrun.UpdateProducer(9U, true);
  CHECK(overrun_update.overrun);
  CHECK(overrun_update.occupied == 9U);
  plan = overrun.PrepareRead(output.size());
  CHECK(plan.overrun && plan.copy_length == 0U);
  CHECK(!TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK(!overrun.CommitRead(plan));
  CHECK(overrun.consumer_position() == 0U);
  CHECK(overrun.high_water_bytes() == 9U);
  CHECK(overrun.rx_wire_bytes() == 9U);

  TinyRing inconsistent;
  CHECK(inconsistent.UpdateProducer(3U, true).consistent);
  const std::uint32_t producer_before = inconsistent.producer_position();
  const std::uint32_t previous_before = inconsistent.previous_dma_position();
  const std::uint32_t high_water_before = inconsistent.high_water_bytes();
  const std::uint64_t wire_before = inconsistent.rx_wire_bytes();
  const TinyRing::ProducerUpdate rejected =
      inconsistent.UpdateProducer(99U, false);
  CHECK(!rejected.consistent && !rejected.overrun);
  CHECK(rejected.delta == 0U && rejected.occupied == 0U);
  CHECK(inconsistent.producer_position() == producer_before);
  CHECK(inconsistent.previous_dma_position() == previous_before);
  CHECK(inconsistent.high_water_bytes() == high_water_before);
  CHECK(inconsistent.rx_wire_bytes() == wire_before);

  TinyRing position_wrap;
  constexpr std::uint32_t kBeforeWrap =
      std::numeric_limits<std::uint32_t>::max() - 2U;
  position_wrap.ResetPositions(kBeforeWrap);
  const TinyRing::ProducerUpdate wrap_update =
      position_wrap.UpdateProducer(1U, true);
  CHECK(wrap_update.consistent && !wrap_update.overrun);
  CHECK(wrap_update.delta == 4U && wrap_update.occupied == 4U);
  CHECK(position_wrap.producer_position() == 1U);
  plan = position_wrap.PrepareRead(4U);
  CHECK(plan.ring_offset == 5U && plan.first_length == 3U);
  CHECK(TinyRing::CopyRead(ring.data(), plan, output.data()));
  CHECK((std::array<std::uint8_t, 4>{output[0], output[1], output[2],
                                     output[3]}) ==
        (std::array<std::uint8_t, 4>{5U, 6U, 7U, 0U}));
  CHECK(position_wrap.CommitRead(plan));
  CHECK(position_wrap.consumer_position() == 1U);

  const std::uint32_t retained_high_water = position_wrap.high_water_bytes();
  const std::uint64_t retained_wire = position_wrap.rx_wire_bytes();
  position_wrap.ResetPositions();
  CHECK(position_wrap.producer_position() == 0U);
  CHECK(position_wrap.consumer_position() == 0U);
  CHECK(position_wrap.previous_dma_position() == 0U);
  CHECK(position_wrap.high_water_bytes() == retained_high_water);
  CHECK(position_wrap.rx_wire_bytes() == retained_wire);

  TinyRing malformed;
  malformed.ResetPositions(6U);
  CHECK(malformed.UpdateProducer(10U, true).consistent);
  const std::uint32_t malformed_consumer = malformed.consumer_position();
  TinyRing::ReadPlan malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.copy_length = 9U;
  CHECK(!TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);

  malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.first_length = 1U;
  CHECK(!TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);

  malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.ring_offset = 7U;
  CHECK(!TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);

  malformed_plan = malformed.PrepareRead(4U);
  malformed_plan.consumer_position += 8U;
  CHECK(TinyRing::CopyRead(ring.data(), malformed_plan, output.data()));
  CHECK(!malformed.CommitRead(malformed_plan));
  CHECK(malformed.consumer_position() == malformed_consumer);
}

void TestPeriodicReleaseSchedule() {
  using mentor_pi_mcu::app::microros::AdvancePeriodicRelease;

  CHECK(AdvancePeriodicRelease(120U, 100U, 20U) == 120U);
  CHECK(AdvancePeriodicRelease(121U, 100U, 20U) == 120U);
  CHECK(AdvancePeriodicRelease(171U, 100U, 20U) == 160U);
  CHECK(AdvancePeriodicRelease(119U, 100U, 20U) == 100U);
  CHECK(AdvancePeriodicRelease(120U, 100U, 0U) == 100U);

  constexpr std::uint32_t kBeforeWrap =
      std::numeric_limits<std::uint32_t>::max() - 9U;
  CHECK(AdvancePeriodicRelease(15U, kBeforeWrap, 20U) == 10U);
}

void TestReclaimingArena() {
  using mentor_pi_mcu::app::microros::ReclaimingArena;

  alignas(std::max_align_t) std::array<std::uint8_t, 1024U> storage{};
  ReclaimingArena arena;
  CHECK(arena.Initialize(storage.data(), storage.size()));
  CHECK(arena.healthy());
  CHECK(arena.bytes_used() == 0U);

  void* const first = arena.Allocate(192U);
  void* const second = arena.Allocate(192U);
  void* const third = arena.Allocate(192U);
  CHECK(first != nullptr && second != nullptr && third != nullptr);
  CHECK(arena.Deallocate(first));
  CHECK(arena.Deallocate(second));
  void* const coalesced = arena.Allocate(384U);
  CHECK(coalesced == first);
  CHECK(arena.Deallocate(coalesced));
  CHECK(arena.Deallocate(third));
  CHECK(arena.bytes_used() == 0U);

  auto* bytes = static_cast<std::uint8_t*>(arena.Allocate(64U));
  CHECK(bytes != nullptr);
  if (bytes != nullptr) {
    for (std::size_t index = 0U; index < 64U; ++index) {
      bytes[index] = static_cast<std::uint8_t>(index);
    }
  }
  auto* const grown = static_cast<std::uint8_t*>(arena.Reallocate(bytes, 160U));
  CHECK(grown == bytes);
  if (grown != nullptr) {
    for (std::size_t index = 0U; index < 64U; ++index) {
      CHECK(grown[index] == static_cast<std::uint8_t>(index));
    }
  }
  CHECK(arena.Deallocate(grown));

  auto* const zeroed = static_cast<std::uint8_t*>(arena.ZeroAllocate(8U, 8U));
  CHECK(zeroed != nullptr);
  if (zeroed != nullptr) {
    for (std::size_t index = 0U; index < 64U; ++index) {
      CHECK(zeroed[index] == 0U);
    }
  }
  CHECK(arena.Deallocate(zeroed));
  CHECK(arena.ZeroAllocate(std::numeric_limits<std::size_t>::max(), 2U) ==
        nullptr);

  arena.Reset();
  CHECK(arena.healthy());
  CHECK(arena.bytes_used() == 0U);
  CHECK(arena.Allocate(900U) != nullptr);
}

}  // namespace
}  // namespace mentor_pi::mcu

int main() {
  mentor_pi::mcu::TestValidationAndStateMerging();
  mentor_pi::mcu::TestValidationBoundaries();
  mentor_pi::mcu::TestFixedContainers();
  mentor_pi::mcu::TestCommandMailboxes();
  mentor_pi::mcu::TestMotorController();
  mentor_pi::mcu::TestPositionalPidController();
  mentor_pi::mcu::TestMotorLeaseBoundariesAndScheduleModel();
  mentor_pi::mcu::TestPwmServoController();
  mentor_pi::mcu::TestButtons();
  mentor_pi::mcu::TestButtonEventQueueBounds();
  mentor_pi::mcu::TestBatteryMonitor();
  mentor_pi::mcu::TestPatterns();
  mentor_pi::mcu::TestBusServoCodecAndScheduler();
  mentor_pi::mcu::TestCircularDmaPosition();
  mentor_pi::mcu::TestUsart1WriteDeadlines();
  mentor_pi::mcu::TestCircularRxRing();
  mentor_pi::mcu::TestPeriodicReleaseSchedule();
  mentor_pi::mcu::TestReclaimingArena();

  if (mentor_pi::mcu::test_failures != 0U) {
    std::cerr << mentor_pi::mcu::test_failures << " test checks failed\n";
    return 1;
  }
  std::cout << "All MCU domain checks passed\n";
  return 0;
}
