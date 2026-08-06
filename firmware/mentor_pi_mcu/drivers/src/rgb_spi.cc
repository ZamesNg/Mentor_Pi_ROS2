#include "mentor_pi_mcu/drivers/rgb_spi.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu::drivers {
namespace {

void EncodeByte(std::uint8_t value, RgbEncodedFrame* frame,
                std::size_t* cursor) {
  for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
    const std::uint8_t mask = static_cast<std::uint8_t>(0x80U >> bit);
    (*frame)[*cursor] = (value & mask) == 0U ? kRgbEncodedZero : kRgbEncodedOne;
    ++(*cursor);
  }
}

Result ToResult(IoStatus status) {
  switch (status) {
    case IoStatus::kOk:
      return OkResult();
    case IoStatus::kBusy:
      return {ResultCode::kBusy, 0};
    case IoStatus::kTimeout:
      return {ResultCode::kTimeout, 0};
    case IoStatus::kIoError:
      return {ResultCode::kIoError, 0};
  }
  return {ResultCode::kIoError, 0};
}

}  // namespace

RgbEncodedFrame EncodeRgbFrame(const RgbState& state) {
  RgbEncodedFrame frame{};
  std::size_t cursor = 0U;
  for (std::size_t pixel = 0; pixel < kRgbPixelCount; ++pixel) {
    // The installed SK6812/WS2812-compatible pixels use GRB wire order.
    EncodeByte(state.green[pixel], &frame, &cursor);
    EncodeByte(state.red[pixel], &frame, &cursor);
    EncodeByte(state.blue[pixel], &frame, &cursor);
  }
  return frame;
}

Result RgbSpiDriver::Begin(const RgbState& state, std::uint32_t deadline_us) {
  if (busy_) {
    return {ResultCode::kBusy, 0};
  }
  frame_ = EncodeRgbFrame(state);
  const IoStatus status =
      spi_.BeginTransmit(frame_.data(), frame_.size(), deadline_us);
  busy_ = status == IoStatus::kOk;
  return status == IoStatus::kOk ? OkResult() : ToResult(status);
}

Result RgbSpiDriver::Poll(std::uint32_t now_us) {
  if (!busy_) {
    return OkResult();
  }
  const IoStatus status = spi_.PollTransmit(now_us);
  if (status != IoStatus::kBusy) {
    busy_ = false;
  }
  return ToResult(status);
}

void RgbSpiDriver::Cancel() {
  if (busy_) {
    spi_.Cancel();
  }
  busy_ = false;
}

}  // namespace mentor_pi::mcu::drivers
