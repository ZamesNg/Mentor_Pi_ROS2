#ifndef MENTOR_PI_MCU_DOMAIN_RESULT_H_
#define MENTOR_PI_MCU_DOMAIN_RESULT_H_

#include <cstdint>

namespace mentor_pi::mcu {

// Values are wire-compatible with mentor_pi_interfaces/msg/Result.
enum class ResultCode : std::uint8_t {
  kOk = 0,
  kInvalidArgument = 1,
  kOutOfRange = 2,
  kBusy = 3,
  kTimeout = 4,
  kIoError = 5,
  kUnsupported = 6,
  kPartial = 7,
};

struct Result {
  ResultCode code{ResultCode::kOk};
  std::uint16_t detail{0};

  constexpr bool ok() const { return code == ResultCode::kOk; }
};

constexpr Result OkResult() { return {}; }

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_RESULT_H_
