#include "mentor_pi_mcu/drivers/qmi8658.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu::drivers {
namespace {

constexpr std::array<std::uint8_t, 2> kAddresses{0x6aU, 0x6bU};
constexpr std::uint8_t kWhoAmIRegister = 0U;
constexpr std::uint8_t kRevisionRegister = 1U;
constexpr std::uint8_t kCtrl1Register = 2U;
constexpr std::uint8_t kCtrl2Register = 3U;
constexpr std::uint8_t kCtrl3Register = 4U;
constexpr std::uint8_t kCtrl5Register = 6U;
constexpr std::uint8_t kCtrl7Register = 8U;
constexpr std::uint8_t kCtrl8Register = 9U;
constexpr std::uint8_t kStatus0Register = 46U;
constexpr std::uint8_t kAccelerationDataRegister = 53U;
constexpr std::uint8_t kResetRegister = 96U;
constexpr std::uint8_t kWhoAmIValue = 0x05U;
constexpr std::uint8_t kSoftResetCommand = 0xb0U;
constexpr float kGravityMps2 = 9.80665F;
constexpr float kDegreesToRadians = 0.017453292519943295F;

Result ToResult(IoStatus status, std::uint16_t detail = 0U) {
  switch (status) {
    case IoStatus::kOk:
      return OkResult();
    case IoStatus::kBusy:
      return {ResultCode::kBusy, detail};
    case IoStatus::kTimeout:
      return {ResultCode::kTimeout, detail};
    case IoStatus::kIoError:
      return {ResultCode::kIoError, detail};
  }
  return {ResultCode::kIoError, detail};
}

std::int16_t ReadSignedLittleEndian(const std::uint8_t* bytes) {
  const std::uint16_t value = static_cast<std::uint16_t>(
      static_cast<std::uint16_t>(bytes[0]) |
      static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[1]) << 8U));
  if (value <= 0x7fffU) {
    return static_cast<std::int16_t>(value);
  }
  return static_cast<std::int16_t>(static_cast<std::int32_t>(value) - 0x10000);
}

}  // namespace

Result Qmi8658Driver::Initialize(std::uint32_t deadline_us) {
  initialized_ = false;
  address_ = 0U;
  revision_ = 0U;
  for (const std::uint8_t candidate : kAddresses) {
    std::uint8_t who_am_i = 0U;
    const IoStatus status =
        i2c_.Read(candidate, kWhoAmIRegister, &who_am_i, 1U, deadline_us);
    if (status == IoStatus::kOk && who_am_i == kWhoAmIValue) {
      address_ = candidate;
      break;
    }
    if (status == IoStatus::kBusy || status == IoStatus::kTimeout) {
      return ToResult(status, candidate);
    }
  }
  if (address_ == 0U) {
    return {ResultCode::kIoError, kWhoAmIRegister};
  }

  IoStatus status =
      i2c_.Read(address_, kRevisionRegister, &revision_, 1U, deadline_us);
  if (status != IoStatus::kOk) {
    return ToResult(status, kRevisionRegister);
  }

  // Configure while both sensors are disabled, then enable them as the final
  // write. This matches the preserved board-reference initialization order
  // and avoids changing CTRL8 while accelerometer/gyroscope startup is active.
  // The resulting configuration is INT pins enabled, ±4 g / 250 Hz,
  // ±128 dps / 250 Hz, sensor LPFs off, and both sensors enabled.
  constexpr std::array<std::array<std::uint8_t, 2>, 7> kConfiguration{{
      {kCtrl7Register, 0x00U},
      {kCtrl1Register, 0x78U},
      {kCtrl2Register, 0x15U},
      {kCtrl3Register, 0x35U},
      {kCtrl5Register, 0x00U},
      {kCtrl8Register, 0xc0U},
      {kCtrl7Register, 0x03U},
  }};
  for (const auto& setting : kConfiguration) {
    const Result result = WriteRegister(setting[0], setting[1], deadline_us);
    if (!result.ok()) {
      return result;
    }
  }
  initialized_ = true;
  return OkResult();
}

Result Qmi8658Driver::Reset(std::uint32_t deadline_us) {
  if (!initialized_ || address_ == 0U) {
    return {ResultCode::kBusy, 0U};
  }
  const Result result =
      WriteRegister(kResetRegister, kSoftResetCommand, deadline_us);
  initialized_ = false;
  return result;
}

bool Qmi8658Driver::DataReady(std::uint32_t deadline_us, Result* result) {
  if (result == nullptr) {
    return false;
  }
  if (!initialized_) {
    *result = {ResultCode::kBusy, 0};
    return false;
  }
  std::uint8_t status_register = 0U;
  const IoStatus status =
      i2c_.Read(address_, kStatus0Register, &status_register, 1U, deadline_us);
  *result = ToResult(status, kStatus0Register);
  return status == IoStatus::kOk && (status_register & 0x03U) != 0U;
}

Result Qmi8658Driver::ReadRawSample(std::uint32_t deadline_us,
                                    ImuSample* sample) {
  if (sample == nullptr) {
    return {ResultCode::kInvalidArgument, 0};
  }
  Result ready_result{};
  if (!DataReady(deadline_us, &ready_result)) {
    return ready_result.ok() ? Result{ResultCode::kBusy, 0} : ready_result;
  }
  std::array<std::uint8_t, 12> raw{};
  const IoStatus status = i2c_.Read(address_, kAccelerationDataRegister,
                                    raw.data(), raw.size(), deadline_us);
  if (status != IoStatus::kOk) {
    return ToResult(status, kAccelerationDataRegister);
  }

  ImuSample sensor_frame{};
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    sensor_frame.acceleration_mps2[axis] =
        static_cast<float>(ReadSignedLittleEndian(&raw[axis * 2U])) / 8192.0F *
        kGravityMps2;
    sensor_frame.angular_velocity_rps[axis] =
        static_cast<float>(ReadSignedLittleEndian(&raw[6U + (axis * 2U)])) /
        256.0F * kDegreesToRadians;
  }
  *sample = sensor_frame;
  return OkResult();
}

Result Qmi8658Driver::ReadSample(std::uint32_t deadline_us,
                                 const AxisTransform& transform,
                                 ImuSample* sample) {
  if (sample == nullptr || !ValidateTransform(transform)) {
    return {ResultCode::kInvalidArgument, 0};
  }
  if (!transform.verified) {
    return {ResultCode::kUnsupported, 1};
  }
  ImuSample sensor_frame{};
  const Result result = ReadRawSample(deadline_us, &sensor_frame);
  if (!result.ok()) {
    return result;
  }
  ImuSample transformed{};
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    const SignedAxis mapping = transform.output[axis];
    transformed.acceleration_mps2[axis] =
        sensor_frame.acceleration_mps2[mapping.source] *
        static_cast<float>(mapping.sign);
    transformed.angular_velocity_rps[axis] =
        sensor_frame.angular_velocity_rps[mapping.source] *
        static_cast<float>(mapping.sign);
  }
  *sample = transformed;
  return OkResult();
}

Result Qmi8658Driver::WriteRegister(std::uint8_t reg, std::uint8_t value,
                                    std::uint32_t deadline_us) {
  return ToResult(i2c_.Write(address_, reg, &value, 1U, deadline_us), reg);
}

bool Qmi8658Driver::ValidateTransform(const AxisTransform& transform) {
  std::uint8_t sources = 0U;
  for (const SignedAxis axis : transform.output) {
    if (axis.source >= 3U || (axis.sign != 1 && axis.sign != -1)) {
      return false;
    }
    const auto bit = static_cast<std::uint8_t>(1U << axis.source);
    if ((sources & bit) != 0U) {
      return false;
    }
    sources = static_cast<std::uint8_t>(sources | bit);
  }
  return sources == 0x07U;
}

}  // namespace mentor_pi::mcu::drivers
