#ifndef MENTOR_PI_MCU_TESTS_FUZZ_FUZZ_INPUT_H_
#define MENTOR_PI_MCU_TESTS_FUZZ_FUZZ_INPUT_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace mentor_pi::mcu::fuzz {

// Bounded, allocation-free decoder shared by the native fuzz targets. Missing
// bytes read as zero so every prefix remains a valid test input.
class FuzzInput {
 public:
  FuzzInput(const std::uint8_t* data, std::size_t size)
      : data_(data), size_(size) {}

  std::uint8_t ReadU8() {
    if (data_ == nullptr || offset_ >= size_) {
      return 0U;
    }
    return data_[offset_++];
  }

  std::uint16_t ReadU16() {
    const std::uint16_t low = ReadU8();
    const std::uint16_t high = ReadU8();
    return static_cast<std::uint16_t>(low | (high << 8U));
  }

  std::int16_t ReadI16() {
    const std::uint16_t bits = ReadU16();
    std::int16_t value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::int8_t ReadI8() {
    const std::uint8_t bits = ReadU8();
    std::int8_t value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  std::uint32_t ReadU32() {
    const std::uint32_t byte0 = ReadU8();
    const std::uint32_t byte1 = ReadU8();
    const std::uint32_t byte2 = ReadU8();
    const std::uint32_t byte3 = ReadU8();
    return byte0 | (byte1 << 8U) | (byte2 << 16U) | (byte3 << 24U);
  }

  float ReadFloat() {
    const std::uint32_t bits = ReadU32();
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  // Explicit selectors make critical IEEE-754 boundaries reachable from
  // one-byte corpus entries instead of relying on mutation to discover them.
  float ReadBiasedFloat() {
    constexpr std::array<float, 14> kBoundaryValues{
        0.0F,
        -0.0F,
        std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::quiet_NaN(),
        std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        std::numeric_limits<float>::denorm_min(),
        0.25F,
        -0.25F,
        1.1F,
        1.5F,
        3.0F,
        6.0F,
    };
    const std::uint8_t selector = ReadU8() & 0x0fU;
    if (selector < kBoundaryValues.size()) {
      return kBoundaryValues[selector];
    }
    return ReadFloat();
  }

  std::size_t remaining() const {
    return offset_ < size_ ? size_ - offset_ : 0U;
  }

 private:
  const std::uint8_t* data_;
  std::size_t size_;
  std::size_t offset_{0U};
};

[[noreturn]] inline void FailInvariant() { std::abort(); }

inline void Require(bool condition) {
  if (!condition) {
    FailInvariant();
  }
}

}  // namespace mentor_pi::mcu::fuzz

#endif  // MENTOR_PI_MCU_TESTS_FUZZ_FUZZ_INPUT_H_
