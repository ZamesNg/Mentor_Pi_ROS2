#ifndef MENTOR_PI_MCU_HOST_CH9102_BOOT_CONTROL_H_
#define MENTOR_PI_MCU_HOST_CH9102_BOOT_CONTROL_H_

#include <chrono>
#include <cstdint>
#include <string_view>
#include <vector>

namespace mentor_pi::firmware::host {

enum class BootMode : std::uint8_t {
  kBootloader,
  kApplication,
};

enum class ModemOperation : std::uint8_t {
  kAssertRts,
  kDeassertRts,
  kAssertDtr,
  kDeassertDtr,
  kDelay,
};

struct ModemStep {
  ModemOperation operation;
  std::chrono::milliseconds delay{0};
};

class ModemBackend {
 public:
  virtual ~ModemBackend() = default;
  virtual bool Apply(ModemOperation operation) = 0;
  virtual void Delay(std::chrono::milliseconds duration) = 0;
};

BootMode ParseBootMode(std::string_view value);
const std::vector<ModemStep>& BootSequence(BootMode mode);
bool ExecuteBootSequence(BootMode mode, ModemBackend* backend);

}  // namespace mentor_pi::firmware::host

#endif  // MENTOR_PI_MCU_HOST_CH9102_BOOT_CONTROL_H_
