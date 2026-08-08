#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>

#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/drivers/battery_adc.h"
#include "mentor_pi_mcu/drivers/bus_servo_uart.h"
#include "mentor_pi_mcu/drivers/gpio_peripherals.h"
#include "mentor_pi_mcu/drivers/motor_encoder.h"
#include "mentor_pi_mcu/drivers/pwm_servo.h"
#include "mentor_pi_mcu/drivers/qmi8658.h"
#include "mentor_pi_mcu/drivers/rgb_spi.h"
#include "mentor_pi_mcu/drivers/ssd1306.h"

namespace {

using mentor_pi::mcu::BusServoCodec;
using mentor_pi::mcu::BusServoCommand;
using mentor_pi::mcu::BusServoFrame;
using mentor_pi::mcu::BusServoOpcode;
using mentor_pi::mcu::ConfigureBusServoCommand;
using mentor_pi::mcu::GetBusServoStateCommand;
using mentor_pi::mcu::OledState;
using mentor_pi::mcu::Result;
using mentor_pi::mcu::ResultCode;
using mentor_pi::mcu::RgbState;
using mentor_pi::mcu::StopBusServosCommand;
using mentor_pi::mcu::drivers::AsyncSpi;
using mentor_pi::mcu::drivers::AxisTransform;
using mentor_pi::mcu::drivers::BatteryAdcCalibration;
using mentor_pi::mcu::drivers::BuildPwmServoFramePlan;
using mentor_pi::mcu::drivers::BusServoState;
using mentor_pi::mcu::drivers::BusServoUartDriver;
using mentor_pi::mcu::drivers::ConvertBatteryAdc;
using mentor_pi::mcu::drivers::EncodeRgbFrame;
using mentor_pi::mcu::drivers::GpioPeripheralDriver;
using mentor_pi::mcu::drivers::HalfDuplexUart;
using mentor_pi::mcu::drivers::ImuSample;
using mentor_pi::mcu::drivers::IoStatus;
using mentor_pi::mcu::drivers::kRgbEncodedOne;
using mentor_pi::mcu::drivers::kRgbEncodedSize;
using mentor_pi::mcu::drivers::kRgbEncodedZero;
using mentor_pi::mcu::drivers::kRgbResetBytes;
using mentor_pi::mcu::drivers::kSsd1306Address;
using mentor_pi::mcu::drivers::kSsd1306Width;
using mentor_pi::mcu::drivers::MotorEncoderDriver;
using mentor_pi::mcu::drivers::MotorHardware;
using mentor_pi::mcu::drivers::PeripheralHardware;
using mentor_pi::mcu::drivers::PwmServoFrameHardware;
using mentor_pi::mcu::drivers::PwmServoFramePlan;
using mentor_pi::mcu::drivers::Qmi8658Driver;
using mentor_pi::mcu::drivers::RawI2c;
using mentor_pi::mcu::drivers::RegisterI2c;
using mentor_pi::mcu::drivers::Ssd1306Driver;

#define CHECK(condition)                                                    \
  do {                                                                      \
    if (condition) {                                                        \
      break;                                                                \
    }                                                                       \
    std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, \
                 #condition);                                               \
    return false;                                                           \
  } while (false)

class FakeMotor final : public MotorHardware {
 public:
  std::uint32_t PwmPeriodTicks(std::size_t motor) const override {
    return periods[motor];
  }
  void WriteDrive(std::size_t motor, std::uint32_t positive_ticks,
                  std::uint32_t negative_ticks) override {
    positive[motor] = positive_ticks;
    negative[motor] = negative_ticks;
  }
  std::uint32_t ReadEncoder(std::size_t motor) const override {
    return encoder[motor];
  }
  void EnableOutputs(bool value) override { enabled = value; }

  std::array<std::uint32_t, 4> periods{1000U, 2000U, 1000U, 1000U};
  std::array<std::uint32_t, 4> positive{};
  std::array<std::uint32_t, 4> negative{};
  std::array<std::uint32_t, 4> encoder{};
  bool enabled{false};
};

class FakePwm final : public PwmServoFrameHardware {
 public:
  void SetPinsHigh(std::uint8_t mask) override { high_mask = mask; }
  void SetPinsLow(std::uint8_t mask) override { low_mask = mask; }
  void ArmCompareUs(std::uint16_t frame_offset_us) override {
    compare = frame_offset_us;
  }
  void DisableCompare() override { disabled = true; }

  std::uint8_t high_mask{0};
  std::uint8_t low_mask{0};
  std::uint16_t compare{0};
  bool disabled{false};
};

class FakePeripheral final : public PeripheralHardware {
 public:
  bool ReadButtonPin(std::size_t button) const override {
    return buttons[button];
  }
  void WriteLedPin(std::size_t led, bool high) override { leds[led] = high; }
  Result SetBuzzerTone(std::uint16_t frequency_hz, bool enabled) override {
    ++buzzer_calls;
    if (!buzzer_result.ok()) {
      return buzzer_result;
    }
    frequency = frequency_hz;
    buzzer_enabled = enabled;
    return {};
  }

  std::array<bool, 2> buttons{true, false};
  std::array<bool, 3> leds{};
  std::uint16_t frequency{0};
  std::uint32_t buzzer_calls{0U};
  Result buzzer_result{};
  bool buzzer_enabled{false};
};

class FakeRegisterI2c final : public RegisterI2c {
 public:
  struct WriteOperation {
    std::uint8_t reg{0U};
    std::uint8_t value{0U};
  };

  IoStatus Read(std::uint8_t address, std::uint8_t reg, std::uint8_t* data,
                std::size_t size, std::uint32_t deadline_us) override {
    last_deadline_us = deadline_us;
    if (address != present_address) {
      return IoStatus::kIoError;
    }
    if (reg == read_failure_register) {
      return read_failure_status;
    }
    for (std::size_t index = 0; index < size; ++index) {
      data[index] = registers[static_cast<std::size_t>(reg) + index];
    }
    return IoStatus::kOk;
  }
  IoStatus Write(std::uint8_t address, std::uint8_t reg,
                 const std::uint8_t* data, std::size_t size,
                 std::uint32_t) override {
    if (address != present_address) {
      return IoStatus::kIoError;
    }
    if (reg == write_failure_register) {
      return write_failure_status;
    }
    for (std::size_t index = 0; index < size; ++index) {
      registers[static_cast<std::size_t>(reg) + index] = data[index];
      if (write_count < writes.size()) {
        writes[write_count] = {static_cast<std::uint8_t>(reg + index),
                               data[index]};
        ++write_count;
      }
    }
    return IoStatus::kOk;
  }

  std::array<std::uint8_t, 128> registers{};
  std::uint32_t last_deadline_us{0U};
  std::uint8_t present_address{0x6aU};
  std::uint8_t read_failure_register{0xffU};
  std::uint8_t write_failure_register{0xffU};
  IoStatus read_failure_status{IoStatus::kIoError};
  IoStatus write_failure_status{IoStatus::kIoError};
  std::array<WriteOperation, 16> writes{};
  std::size_t write_count{0U};
};

class FakeSpi final : public AsyncSpi {
 public:
  IoStatus BeginTransmit(const std::uint8_t* data, std::size_t size,
                         std::uint32_t) override {
    if (begin_status != IoStatus::kOk) {
      return begin_status;
    }
    first = data[0];
    transmitted_size = size;
    active = true;
    return IoStatus::kOk;
  }
  IoStatus PollTransmit(std::uint32_t) override {
    if (poll_status != IoStatus::kBusy) {
      active = false;
    }
    return poll_status;
  }
  void Cancel() override { active = false; }

  std::size_t transmitted_size{0};
  std::uint8_t first{0};
  IoStatus begin_status{IoStatus::kOk};
  IoStatus poll_status{IoStatus::kOk};
  bool active{false};
};

class FakeRawI2c final : public RawI2c {
 public:
  IoStatus Write(std::uint8_t address, const std::uint8_t* data,
                 std::size_t size, std::uint32_t) override {
    ++calls;
    if (calls == fail_call) {
      return failure_status;
    }
    last_address = address;
    last_control = data[0];
    bytes += size;
    return IoStatus::kOk;
  }

  std::size_t bytes{0};
  std::size_t calls{0};
  std::size_t fail_call{0};
  std::uint8_t last_address{0};
  std::uint8_t last_control{0};
  IoStatus failure_status{IoStatus::kIoError};
};

class FakeUart final : public HalfDuplexUart {
 public:
  IoStatus BeginExchange(const std::uint8_t* tx, std::size_t tx_size,
                         std::size_t max_reply_size, std::uint32_t) override {
    if (begin_status != IoStatus::kOk) {
      return begin_status;
    }
    for (std::size_t index = 0; index < tx_size; ++index) {
      request[index] = tx[index];
    }
    request_size = tx_size;
    maximum_reply = max_reply_size;
    opcodes[exchange_count] = tx[4];
    ++exchange_count;
    return IoStatus::kOk;
  }
  IoStatus PollExchange(std::uint32_t, std::uint8_t* reply,
                        std::size_t capacity,
                        std::size_t* reply_size) override {
    if (always_busy || poll_status == IoStatus::kBusy) {
      return IoStatus::kBusy;
    }
    if (poll_status != IoStatus::kOk) {
      return poll_status;
    }
    if (fail_poll_exchange != 0U && exchange_count == fail_poll_exchange) {
      return IoStatus::kIoError;
    }
    if (maximum_reply == 0U) {
      *reply_size = 0U;
      return IoStatus::kOk;
    }
    const auto opcode = static_cast<BusServoOpcode>(request[4]);
    std::array<std::uint8_t, 4> arguments{};
    std::size_t count = 1U;
    if (opcode == BusServoOpcode::kIdRead) {
      arguments[0] = 7U;
    } else if (opcode == BusServoOpcode::kPositionRead) {
      arguments[0] = 0xccU;
      arguments[1] = 0xffU;
      count = 2U;
    } else if (opcode == BusServoOpcode::kOffsetRead) {
      arguments[0] = 0xf6U;
    } else if (opcode == BusServoOpcode::kVoltageRead) {
      arguments[0] = 0x20U;
      arguments[1] = 0x1cU;
      count = 2U;
    } else if (opcode == BusServoOpcode::kTemperatureRead) {
      arguments[0] = 42U;
    } else if (opcode == BusServoOpcode::kPositionLimitsRead) {
      arguments = {100U, 0U, 0x84U, 0x03U};
      count = 4U;
    } else if (opcode == BusServoOpcode::kVoltageLimitsRead) {
      arguments = {0x70U, 0x17U, 0x28U, 0x23U};
      count = 4U;
    } else if (opcode == BusServoOpcode::kTemperatureLimitRead) {
      arguments[0] = 75U;
    } else if (opcode == BusServoOpcode::kTorqueRead) {
      arguments[0] = 1U;
    }
    BusServoFrame frame{};
    const std::uint8_t response_id =
        response_servo_id == 0U ? request[2] : response_servo_id;
    const BusServoOpcode response_opcode =
        wrong_opcode ? BusServoOpcode::kMoveStop : opcode;
    const auto result = BusServoCodec::BuildFrame(
        response_id, response_opcode, arguments.data(), count, &frame);
    if (!result.ok() || frame.size > capacity) {
      return IoStatus::kIoError;
    }
    if (truncate_reply && frame.size != 0U) {
      --frame.size;
    }
    for (std::size_t index = 0; index < frame.size; ++index) {
      reply[index] = frame.bytes[index];
    }
    *reply_size = frame.size;
    return IoStatus::kOk;
  }
  void Cancel() override { cancelled = true; }

  std::array<std::uint8_t, 14> request{};
  std::array<std::uint8_t, 32> opcodes{};
  std::size_t request_size{0};
  std::size_t maximum_reply{0};
  std::size_t exchange_count{0};
  std::size_t fail_poll_exchange{0};
  IoStatus begin_status{IoStatus::kOk};
  IoStatus poll_status{IoStatus::kOk};
  std::uint8_t response_servo_id{0U};
  bool always_busy{false};
  bool cancelled{false};
  bool wrong_opcode{false};
  bool truncate_reply{false};
};

bool TestMotorAndPwm() {
  FakeMotor hardware;
  hardware.encoder = {0xfffffff0U, 100U, 0xfff0U, 10U};
  MotorEncoderDriver motor(hardware);
  motor.InitializeSafe();
  CHECK(!hardware.enabled);
  motor.Enable();
  motor.ApplyPermille({500, -250, 1200, 0});
  CHECK(hardware.positive[0] == 500U);
  CHECK(hardware.negative[1] == 500U);
  CHECK(hardware.positive[2] == 1000U);
  hardware.encoder = {0x10U, 90U, 0x10U, 5U};
  const auto delta = motor.SampleEncoderDeltas();
  CHECK(delta[0] == 32);
  CHECK(delta[1] == -10);
  CHECK(delta[2] == 32);
  CHECK(delta[3] == -5);

  PwmServoFramePlan plan{};
  CHECK(BuildPwmServoFramePlan({1500U, 1500U, 1500U, 1500U}, nullptr).code ==
        ResultCode::kInvalidArgument);
  CHECK(BuildPwmServoFramePlan({499U, 1500U, 1500U, 1500U}, &plan).code ==
        ResultCode::kOutOfRange);
  CHECK(BuildPwmServoFramePlan({1500U, 1500U, 1500U, 2501U}, &plan).detail ==
        4U);
  CHECK(BuildPwmServoFramePlan({1500U, 500U, 1500U, 2500U}, &plan).ok());
  CHECK(plan.edge_count == 3U);
  CHECK(plan.edges[0].at_us == 500U && plan.edges[0].clear_mask == 0x02U);
  CHECK(plan.edges[1].clear_mask == 0x05U);
  FakePwm pwm;
  mentor_pi::mcu::drivers::PwmServoFrameDriver driver(pwm);
  driver.LoadPlan(plan);
  driver.BeginFrame();
  CHECK(pwm.high_mask == 0x0fU && pwm.compare == 500U);
  driver.HandleCompare();
  CHECK(pwm.low_mask == 0x02U && pwm.compare == 1500U);
  driver.HandleCompare();
  CHECK(pwm.low_mask == 0x05U && pwm.compare == 2500U);
  driver.HandleCompare();
  CHECK(pwm.disabled);
  driver.HandleCompare();
  driver.Stop();
  CHECK(pwm.low_mask == 0x0fU && pwm.disabled);

  PwmServoFramePlan empty{};
  empty.initial_high_mask = 0U;
  pwm.disabled = false;
  driver.LoadPlan(empty);
  driver.BeginFrame();
  CHECK(pwm.high_mask == 0U && pwm.disabled);
  return true;
}

bool TestSimplePeripherals() {
  const auto battery = ConvertBatteryAdc(1210U, 700U, BatteryAdcCalibration{});
  CHECK(battery.valid && battery.voltage_mv == 7700U);
  CHECK(!ConvertBatteryAdc(0U, 700U, BatteryAdcCalibration{}).valid);
  CHECK(!ConvertBatteryAdc(4095U, 700U, BatteryAdcCalibration{}).valid);
  BatteryAdcCalibration invalid_calibration{};
  invalid_calibration.internal_reference_mv = 0U;
  CHECK(!ConvertBatteryAdc(1210U, 700U, invalid_calibration).valid);
  invalid_calibration = {};
  invalid_calibration.divider_denominator = 0U;
  CHECK(!ConvertBatteryAdc(1210U, 700U, invalid_calibration).valid);
  invalid_calibration = {};
  invalid_calibration.maximum_valid_mv = 100U;
  CHECK(!ConvertBatteryAdc(1210U, 700U, invalid_calibration).valid);

  FakePeripheral hardware;
  GpioPeripheralDriver gpio(hardware);
  CHECK(gpio.InitializeSafe().ok());
  CHECK(hardware.leds[0] && hardware.leds[1] && !hardware.leds[2]);
  CHECK(hardware.buzzer_calls == 1U);
  CHECK(!gpio.ButtonPressed(0U) && gpio.ButtonPressed(1U));
  CHECK(!gpio.ButtonPressed(2U));
  gpio.SetLed(0U, true);
  gpio.SetLed(2U, true);
  gpio.SetLed(3U, true);
  CHECK(!hardware.leds[0] && hardware.leds[2]);
  CHECK(gpio.SetBuzzer(2100U, true).ok());
  CHECK(hardware.buzzer_enabled && hardware.frequency == 2100U);
  CHECK(gpio.SetBuzzer(0U, true).ok());
  CHECK(!hardware.buzzer_enabled);
  hardware.buzzer_result = {ResultCode::kTimeout, 77U};
  CHECK(gpio.SetBuzzer(2100U, true).code == ResultCode::kTimeout);
  CHECK(!hardware.buzzer_enabled && hardware.frequency == 0U);
  CHECK(gpio.InitializeSafe().code == ResultCode::kTimeout);
  CHECK(hardware.leds[0] && hardware.leds[1] && !hardware.leds[2]);

  RgbState rgb{};
  rgb.green[0] = 0x80U;
  rgb.red[0] = 0x01U;
  const auto encoded = EncodeRgbFrame(rgb);
  CHECK(encoded.size() == kRgbEncodedSize);
  CHECK(encoded[0] == kRgbEncodedOne && encoded[1] == kRgbEncodedZero);
  CHECK(encoded[15] == kRgbEncodedOne);
  for (std::size_t index = encoded.size() - kRgbResetBytes;
       index < encoded.size(); ++index) {
    CHECK(encoded[index] == 0U);
  }
  FakeSpi spi;
  mentor_pi::mcu::drivers::RgbSpiDriver rgb_driver(spi);
  CHECK(rgb_driver.Begin(rgb, 1000U).ok());
  CHECK(spi.transmitted_size == kRgbEncodedSize && rgb_driver.busy());
  CHECK(rgb_driver.Begin(rgb, 1001U).code == ResultCode::kBusy);
  spi.poll_status = IoStatus::kBusy;
  CHECK(rgb_driver.Poll(1U).code == ResultCode::kBusy);
  CHECK(rgb_driver.busy());
  rgb_driver.Cancel();
  CHECK(!rgb_driver.busy() && !spi.active);
  rgb_driver.Cancel();

  for (const auto& [status, expected] :
       std::array<std::pair<IoStatus, ResultCode>, 3>{
           {{IoStatus::kBusy, ResultCode::kBusy},
            {IoStatus::kTimeout, ResultCode::kTimeout},
            {IoStatus::kIoError, ResultCode::kIoError}}}) {
    spi.begin_status = status;
    CHECK(rgb_driver.Begin(rgb, 2000U).code == expected);
    CHECK(!rgb_driver.busy());
  }
  spi.begin_status = IoStatus::kOk;
  spi.poll_status = IoStatus::kTimeout;
  CHECK(rgb_driver.Begin(rgb, 3000U).ok());
  CHECK(rgb_driver.Poll(3U).code == ResultCode::kTimeout);
  CHECK(!rgb_driver.busy());
  CHECK(rgb_driver.Poll(4U).ok());
  return true;
}

bool TestImuAndOled() {
  FakeRegisterI2c i2c;
  i2c.registers[0] = 0x05U;
  i2c.registers[1] = 0x42U;
  i2c.registers[46] = 0x03U;
  i2c.registers[53] = 0x00U;
  i2c.registers[54] = 0x20U;  // +1 g.
  i2c.registers[55] = 0x00U;
  i2c.registers[56] = 0xf0U;  // -0.5 g.
  i2c.registers[57] = 0x00U;
  i2c.registers[58] = 0x08U;  // +0.25 g.
  i2c.registers[59] = 0x00U;
  i2c.registers[60] = 0x01U;  // +1 degree/s.
  i2c.registers[61] = 0x80U;
  i2c.registers[62] = 0xffU;  // -0.5 degree/s.
  i2c.registers[63] = 0x00U;
  i2c.registers[64] = 0x02U;  // +2 degrees/s.
  Qmi8658Driver imu(i2c);
  ImuSample sample{};
  CHECK(!imu.DataReady(100U, nullptr));
  ResultCode uninitialized_code = ResultCode::kOk;
  mentor_pi::mcu::Result data_ready_result{};
  CHECK(!imu.DataReady(100U, &data_ready_result));
  uninitialized_code = data_ready_result.code;
  CHECK(uninitialized_code == ResultCode::kBusy);
  CHECK(imu.ReadRawSample(500U, &sample).code ==
        mentor_pi::mcu::ResultCode::kBusy);
  CHECK(imu.ReadRawSample(500U, nullptr).code ==
        mentor_pi::mcu::ResultCode::kInvalidArgument);
  CHECK(imu.Initialize(1000U).ok());
  CHECK(imu.address() == 0x6aU && imu.revision() == 0x42U);
  constexpr std::array<FakeRegisterI2c::WriteOperation, 7>
      kExpectedInitializationWrites{{
          {8U, 0x00U},
          {2U, 0x78U},
          {3U, 0x15U},
          {4U, 0x35U},
          {6U, 0x00U},
          {9U, 0xc0U},
          {8U, 0x03U},
      }};
  CHECK(i2c.write_count == kExpectedInitializationWrites.size());
  for (std::size_t index = 0U; index < kExpectedInitializationWrites.size();
       ++index) {
    CHECK(i2c.writes[index].reg == kExpectedInitializationWrites[index].reg);
    CHECK(i2c.writes[index].value ==
          kExpectedInitializationWrites[index].value);
  }
  CHECK(imu.ReadRawSample(4321U, &sample).ok());
  CHECK(i2c.last_deadline_us == 4321U);
  CHECK(std::fabs(sample.acceleration_mps2[0] - 9.80665F) < 0.0001F);
  CHECK(std::fabs(sample.acceleration_mps2[1] + 4.903325F) < 0.0001F);
  CHECK(std::fabs(sample.acceleration_mps2[2] - 2.4516625F) < 0.0001F);
  CHECK(std::fabs(sample.angular_velocity_rps[0] - 0.017453293F) < 0.0001F);
  CHECK(std::fabs(sample.angular_velocity_rps[1] + 0.008726646F) < 0.0001F);
  CHECK(std::fabs(sample.angular_velocity_rps[2] - 0.034906585F) < 0.0001F);
  CHECK(imu.DataReady(4322U, &data_ready_result));

  i2c.registers[46] = 0x00U;
  CHECK(imu.ReadRawSample(5000U, &sample).code ==
        mentor_pi::mcu::ResultCode::kBusy);
  i2c.registers[46] = 0x03U;
  i2c.read_failure_register = 46U;
  i2c.read_failure_status = IoStatus::kTimeout;
  const auto status_timeout = imu.ReadRawSample(5001U, &sample);
  CHECK(status_timeout.code == mentor_pi::mcu::ResultCode::kTimeout);
  CHECK(status_timeout.detail == 46U);
  i2c.read_failure_register = 53U;
  i2c.read_failure_status = IoStatus::kIoError;
  const auto data_error = imu.ReadRawSample(5002U, &sample);
  CHECK(data_error.code == mentor_pi::mcu::ResultCode::kIoError);
  CHECK(data_error.detail == 53U);
  i2c.read_failure_register = 0xffU;

  AxisTransform transform{};
  CHECK(imu.ReadSample(1000U, transform, &sample).code ==
        mentor_pi::mcu::ResultCode::kUnsupported);
  transform.verified = true;
  transform.output = {{{2U, -1}, {0U, 1}, {1U, -1}}};
  CHECK(imu.ReadSample(1000U, transform, &sample).ok());
  CHECK(std::fabs(sample.acceleration_mps2[0] + 2.4516625F) < 0.0001F);
  CHECK(std::fabs(sample.acceleration_mps2[1] - 9.80665F) < 0.0001F);
  CHECK(std::fabs(sample.acceleration_mps2[2] - 4.903325F) < 0.0001F);
  CHECK(std::fabs(sample.angular_velocity_rps[0] + 0.034906585F) < 0.0001F);
  CHECK(std::fabs(sample.angular_velocity_rps[1] - 0.017453293F) < 0.0001F);
  CHECK(std::fabs(sample.angular_velocity_rps[2] - 0.008726646F) < 0.0001F);

  transform.output[2] = transform.output[1];
  CHECK(imu.ReadSample(1000U, transform, &sample).code ==
        mentor_pi::mcu::ResultCode::kInvalidArgument);
  transform.output = {{{0U, 1}, {1U, 1}, {3U, 1}}};
  CHECK(imu.ReadSample(1000U, transform, &sample).code ==
        mentor_pi::mcu::ResultCode::kInvalidArgument);
  transform.output = {{{0U, 1}, {1U, 0}, {2U, 1}}};
  CHECK(imu.ReadSample(1000U, transform, &sample).code ==
        mentor_pi::mcu::ResultCode::kInvalidArgument);

  FakeRegisterI2c fallback_i2c;
  fallback_i2c.present_address = 0x6bU;
  fallback_i2c.registers[0] = 0x05U;
  fallback_i2c.registers[1] = 0x24U;
  Qmi8658Driver fallback_imu(fallback_i2c);
  CHECK(fallback_imu.Initialize(1000U).ok());
  CHECK(fallback_imu.address() == 0x6bU && fallback_imu.revision() == 0x24U);

  FakeRegisterI2c absent_i2c;
  absent_i2c.present_address = 0U;
  Qmi8658Driver absent_imu(absent_i2c);
  CHECK(absent_imu.Initialize(1000U).code == ResultCode::kIoError);

  for (const auto& [status, expected] :
       std::array<std::pair<IoStatus, ResultCode>, 2>{
           {{IoStatus::kBusy, ResultCode::kBusy},
            {IoStatus::kTimeout, ResultCode::kTimeout}}}) {
    FakeRegisterI2c failing_i2c;
    failing_i2c.registers[0] = 0x05U;
    failing_i2c.read_failure_register = 0U;
    failing_i2c.read_failure_status = status;
    Qmi8658Driver failing_imu(failing_i2c);
    const auto failure = failing_imu.Initialize(1000U);
    CHECK(failure.code == expected && failure.detail == 0x6aU);
  }

  FakeRegisterI2c revision_i2c;
  revision_i2c.registers[0] = 0x05U;
  revision_i2c.read_failure_register = 1U;
  revision_i2c.read_failure_status = IoStatus::kTimeout;
  Qmi8658Driver revision_imu(revision_i2c);
  const auto revision_failure = revision_imu.Initialize(1000U);
  CHECK(revision_failure.code == ResultCode::kTimeout &&
        revision_failure.detail == 1U);

  FakeRegisterI2c configuration_i2c;
  configuration_i2c.registers[0] = 0x05U;
  configuration_i2c.registers[1] = 1U;
  configuration_i2c.write_failure_register = 2U;
  configuration_i2c.write_failure_status = IoStatus::kBusy;
  Qmi8658Driver configuration_imu(configuration_i2c);
  const auto configuration_failure = configuration_imu.Initialize(1000U);
  CHECK(configuration_failure.code == ResultCode::kBusy &&
        configuration_failure.detail == 2U);

  FakeRawI2c display_i2c;
  Ssd1306Driver display(display_i2c);
  OledState state{};
  CHECK(display.Render(state, 0U, 1U).code == ResultCode::kBusy);
  CHECK(display.Initialize(10U).ok());
  constexpr std::array<char, 13> kGlyphExercise{
      'a', '0', ' ', '-', '_', '.', ':', '/', '+', '=', '%', '?', 'Z'};
  for (std::size_t index = 0U; index < kGlyphExercise.size(); ++index) {
    state.lines[0].bytes[index] = kGlyphExercise[index];
  }
  state.lines[0].size = static_cast<std::uint8_t>(kGlyphExercise.size());
  CHECK(display.Render(state, 7400U, 20U).ok());
  CHECK(display_i2c.last_address == kSsd1306Address);
  CHECK(display_i2c.calls == 35U);  // init + 2 complete framebuffer flushes.
  CHECK(display.framebuffer()[0] != 0U);

  state = {};
  state.lines[0].bytes[0] = '?';
  state.lines[0].size = 1U;
  CHECK(display.Render(state, 0U, 21U).ok());
  std::array<std::uint8_t, 5> question_glyph{};
  for (std::size_t column = 0U; column < question_glyph.size(); ++column) {
    question_glyph[column] = display.framebuffer()[column];
  }
  for (std::uint8_t character = 0x20U; character <= 0x7eU; ++character) {
    state.lines[0].bytes[0] = static_cast<char>(character);
    CHECK(display.Render(state, 0U, 22U).ok());
    bool matches_question = true;
    for (std::size_t column = 0U; column < question_glyph.size(); ++column) {
      matches_question = matches_question && display.framebuffer()[column] ==
                                                 question_glyph[column];
    }
    CHECK(character == static_cast<std::uint8_t>('?') || !matches_question);
  }

  state.lines[0].bytes[0] = 'A';
  CHECK(display.Render(state, 0U, 23U).ok());
  std::array<std::uint8_t, 5> uppercase_a{};
  for (std::size_t column = 0U; column < uppercase_a.size(); ++column) {
    uppercase_a[column] = display.framebuffer()[column];
  }
  state.lines[0].bytes[0] = 'a';
  CHECK(display.Render(state, 0U, 24U).ok());
  bool lowercase_is_distinct = false;
  for (std::size_t column = 0U; column < uppercase_a.size(); ++column) {
    lowercase_is_distinct =
        lowercase_is_distinct ||
        display.framebuffer()[column] != uppercase_a[column];
  }
  CHECK(lowercase_is_distinct);

  state = {};
  CHECK(display.Render(state, 7400U, 25U).ok());
  bool page_two_has_pixels = false;
  bool page_three_has_pixels = false;
  for (std::size_t column = 0U; column < kSsd1306Width; ++column) {
    page_two_has_pixels =
        page_two_has_pixels ||
        display.framebuffer()[(2U * kSsd1306Width) + column] != 0U;
    page_three_has_pixels =
        page_three_has_pixels ||
        display.framebuffer()[(3U * kSsd1306Width) + column] != 0U;
  }
  CHECK(!page_two_has_pixels && page_three_has_pixels);

  for (const auto& [status, expected] :
       std::array<std::pair<IoStatus, ResultCode>, 3>{
           {{IoStatus::kBusy, ResultCode::kBusy},
            {IoStatus::kTimeout, ResultCode::kTimeout},
            {IoStatus::kIoError, ResultCode::kIoError}}}) {
    FakeRawI2c initialization_i2c;
    initialization_i2c.fail_call = 1U;
    initialization_i2c.failure_status = status;
    Ssd1306Driver failed_display(initialization_i2c);
    const auto failure = failed_display.Initialize(10U);
    CHECK(failure.code == expected && failure.detail == 1U);
  }

  FakeRawI2c address_window_i2c;
  address_window_i2c.fail_call = 2U;
  address_window_i2c.failure_status = IoStatus::kIoError;
  Ssd1306Driver address_window_display(address_window_i2c);
  const auto address_window_failure = address_window_display.Initialize(10U);
  CHECK(address_window_failure.code == ResultCode::kIoError &&
        address_window_failure.detail == 2U);

  FakeRawI2c chunk_i2c;
  chunk_i2c.fail_call = 3U;
  chunk_i2c.failure_status = IoStatus::kTimeout;
  Ssd1306Driver chunk_display(chunk_i2c);
  const auto chunk_failure = chunk_display.Initialize(10U);
  CHECK(chunk_failure.code == ResultCode::kTimeout &&
        chunk_failure.detail == 3U);
  return true;
}

bool TestBusServoStateMachine() {
  FakeUart uart;
  BusServoUartDriver driver(uart);
  GetBusServoStateCommand query{};
  query.servo_id = 7U;
  query.fields = GetBusServoStateCommand::kAllFields;
  CHECK(driver.StartQuery(query, 100U).ok());
  CHECK(uart.maximum_reply == 7U);  // ID response: one argument + framing.
  mentor_pi::mcu::drivers::BusServoPollResult poll{};
  for (std::size_t attempt = 0; attempt < 20U; ++attempt) {
    poll = driver.Poll(101U + static_cast<std::uint32_t>(attempt));
    if (poll.complete) {
      break;
    }
  }
  CHECK(poll.complete && poll.result.ok());
  CHECK(uart.maximum_reply == 7U);  // Torque response: one + framing.
  CHECK(uart.exchange_count == 9U);
  CHECK(poll.completed_mask == query.fields);
  CHECK(poll.state.requested_id == 7U && poll.state.reported_id == 7U);
  CHECK(poll.state.position == -52);
  CHECK(poll.state.offset == -10);
  CHECK(poll.state.voltage_mv == 7200U);
  CHECK(poll.state.temperature_c == 42U);
  CHECK(poll.state.position_min == 100U && poll.state.position_max == 900U);
  CHECK(poll.state.voltage_min_mv == 6000U &&
        poll.state.voltage_max_mv == 9000U);
  CHECK(poll.state.temperature_limit_c == 75U);
  CHECK(poll.state.torque_enabled);

  FakeUart configure_uart;
  BusServoUartDriver configure_driver(configure_uart);
  ConfigureBusServoCommand configure{};
  configure.servo_id = 7U;
  configure.update_mask = ConfigureBusServoCommand::kAllUpdates;
  configure.new_id = 8U;
  configure.offset = -10;
  configure.position_min = 100U;
  configure.position_max = 900U;
  configure.voltage_min_mv = 6000U;
  configure.voltage_max_mv = 9000U;
  configure.temperature_limit_c = 75U;
  configure.torque_enabled = true;
  CHECK(configure_driver.StartConfigure(configure, 0U).ok());
  for (std::size_t attempt = 0; attempt < 14U; ++attempt) {
    poll = configure_driver.Poll(static_cast<std::uint32_t>(attempt + 1U));
    if (poll.complete) {
      break;
    }
  }
  CHECK(poll.complete && poll.result.ok());
  CHECK(configure_uart.exchange_count == 7U);
  CHECK(configure_uart.opcodes[0] ==
        static_cast<std::uint8_t>(BusServoOpcode::kOffsetAdjust));
  CHECK(configure_uart.opcodes[1] ==
        static_cast<std::uint8_t>(BusServoOpcode::kOffsetSave));
  CHECK(configure_uart.opcodes[6] ==
        static_cast<std::uint8_t>(BusServoOpcode::kIdWrite));

  FakeUart partial_uart;
  partial_uart.fail_poll_exchange = 2U;
  BusServoUartDriver partial_driver(partial_uart);
  CHECK(partial_driver.StartConfigure(configure, 0U).ok());
  poll = partial_driver.Poll(1U);
  CHECK(!poll.complete);
  poll = partial_driver.Poll(2U);
  CHECK(poll.complete);
  CHECK(poll.result.code == mentor_pi::mcu::ResultCode::kPartial);
  CHECK(poll.completed_mask == ConfigureBusServoCommand::kSetOffset);

  FakeUart blocked_uart;
  blocked_uart.always_busy = true;
  BusServoUartDriver blocked(blocked_uart);
  BusServoCommand move{};
  move.count = 1U;
  move.servo_id[0] = 1U;
  move.position[0] = 500U;
  move.duration_ms = 100U;
  CHECK(blocked.StartMove(move, 0U).ok());
  const auto timeout = blocked.Poll(200U);
  CHECK(timeout.complete);
  CHECK(timeout.result.code == mentor_pi::mcu::ResultCode::kTimeout);
  CHECK(blocked_uart.cancelled);

  FakeUart stop_uart;
  BusServoUartDriver stop_driver(stop_uart);
  StopBusServosCommand stop{};
  stop.count = 2U;
  stop.servo_id[0] = 1U;
  stop.servo_id[1] = 2U;
  CHECK(stop_driver.StartStop(stop, 10U).ok());
  poll = stop_driver.Poll(11U);
  CHECK(!poll.complete && poll.completed_mask == 1U);
  poll = stop_driver.Poll(12U);
  CHECK(poll.complete && poll.result.ok() && poll.completed_mask == 3U);

  FakeUart move_uart;
  BusServoUartDriver move_driver(move_uart);
  BusServoCommand two_moves{};
  two_moves.count = 2U;
  two_moves.servo_id[0] = 1U;
  two_moves.servo_id[1] = 2U;
  two_moves.position[0] = 400U;
  two_moves.position[1] = 600U;
  two_moves.duration_ms = 100U;
  CHECK(move_driver.StartMove(two_moves, 20U).ok());
  poll = move_driver.Poll(21U);
  CHECK(!poll.complete && poll.completed_mask == 1U);
  poll = move_driver.Poll(22U);
  CHECK(poll.complete && poll.result.ok() && poll.completed_mask == 3U);

  FakeUart broadcast_uart;
  broadcast_uart.response_servo_id = 7U;
  BusServoUartDriver broadcast_driver(broadcast_uart);
  GetBusServoStateCommand broadcast{};
  broadcast.servo_id = 254U;
  broadcast.fields = GetBusServoStateCommand::kFieldId;
  CHECK(broadcast_driver.StartQuery(broadcast, 30U).ok());
  poll = broadcast_driver.Poll(31U);
  CHECK(poll.complete && poll.result.ok() && poll.state.reported_id == 7U);
  return true;
}

bool TestBusServoFailurePaths() {
  BusServoCommand valid_move{};
  valid_move.count = 1U;
  valid_move.servo_id[0] = 1U;
  valid_move.position[0] = 500U;
  valid_move.duration_ms = 100U;
  StopBusServosCommand valid_stop{};
  valid_stop.count = 1U;
  valid_stop.servo_id[0] = 1U;

  FakeUart validation_uart;
  BusServoUartDriver validation_driver(validation_uart);
  CHECK(validation_driver.StartMove(BusServoCommand{}, 0U).code ==
        ResultCode::kInvalidArgument);
  CHECK(validation_driver.StartStop(StopBusServosCommand{}, 0U).code ==
        ResultCode::kInvalidArgument);
  CHECK(validation_driver.StartQuery(GetBusServoStateCommand{}, 0U).code ==
        ResultCode::kInvalidArgument);
  CHECK(validation_driver.StartConfigure(ConfigureBusServoCommand{}, 0U).code ==
        ResultCode::kOutOfRange);
  CHECK(validation_driver.StartMove(valid_move, 0U).ok());
  CHECK(validation_driver.StartStop(valid_stop, 0U).code == ResultCode::kBusy);
  validation_driver.Cancel();
  CHECK(validation_uart.cancelled);
  const auto canceled = validation_driver.Poll(1U);
  CHECK(canceled.complete && canceled.result.code == ResultCode::kIoError);

  for (const auto& [status, expected] :
       std::array<std::pair<IoStatus, ResultCode>, 3>{
           {{IoStatus::kBusy, ResultCode::kBusy},
            {IoStatus::kTimeout, ResultCode::kTimeout},
            {IoStatus::kIoError, ResultCode::kIoError}}}) {
    FakeUart begin_uart;
    begin_uart.begin_status = status;
    BusServoUartDriver begin_driver(begin_uart);
    const auto start = begin_driver.StartMove(valid_move, 0U);
    if (status == IoStatus::kBusy) {
      CHECK(start.ok() && begin_driver.busy());
      begin_uart.begin_status = IoStatus::kOk;
      CHECK(begin_driver.Poll(1U).complete);
    } else {
      CHECK(start.code == expected && !begin_driver.busy());
    }
  }

  for (const auto status :
       std::array<IoStatus, 2>{IoStatus::kTimeout, IoStatus::kIoError}) {
    FakeUart poll_uart;
    poll_uart.poll_status = status;
    BusServoUartDriver poll_driver(poll_uart);
    CHECK(poll_driver.StartMove(valid_move, 0U).ok());
    const auto failed = poll_driver.Poll(1U);
    CHECK(failed.complete);
    CHECK(failed.result.code == (status == IoStatus::kTimeout
                                     ? ResultCode::kTimeout
                                     : ResultCode::kIoError));
  }

  GetBusServoStateCommand query{};
  query.servo_id = 7U;
  query.fields = GetBusServoStateCommand::kFieldId;
  for (std::uint8_t failure = 0U; failure < 3U; ++failure) {
    FakeUart reply_uart;
    reply_uart.truncate_reply = failure == 0U;
    reply_uart.response_servo_id = failure == 1U ? 8U : 0U;
    reply_uart.wrong_opcode = failure == 2U;
    BusServoUartDriver reply_driver(reply_uart);
    CHECK(reply_driver.StartQuery(query, 0U).ok());
    const auto failed = reply_driver.Poll(1U);
    CHECK(failed.complete && !failed.result.ok());
  }
  return true;
}

}  // namespace

int main() {
  const std::array<bool (*)(), 5> tests{
      TestMotorAndPwm, TestSimplePeripherals, TestImuAndOled,
      TestBusServoStateMachine, TestBusServoFailurePaths};
  for (const auto test : tests) {
    if (!test()) {
      return 1;
    }
  }
  return 0;
}
