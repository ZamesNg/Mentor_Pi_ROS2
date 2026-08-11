#include "ch9102_boot_control.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

using mentor_pi::firmware::host::BootMode;
using mentor_pi::firmware::host::ModemBackend;
using mentor_pi::firmware::host::ModemOperation;

class FakeBackend final : public ModemBackend {
 public:
  bool Apply(ModemOperation operation) override {
    operations.push_back(operation);
    return operations.size() != fail_at;
  }

  void Delay(std::chrono::milliseconds duration) override {
    operations.push_back(ModemOperation::kDelay);
    delays.push_back(duration);
  }

  std::vector<ModemOperation> operations;
  std::vector<std::chrono::milliseconds> delays;
  std::size_t fail_at{0U};
};

void Require(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "CH9102F boot-control test failed: " << message << '\n';
    std::exit(1);
  }
}

void RequireSequence(BootMode mode,
                     const std::vector<ModemOperation>& expected) {
  FakeBackend backend;
  Require(mentor_pi::firmware::host::ExecuteBootSequence(mode, &backend),
          "sequence unexpectedly failed");
  Require(backend.operations == expected, "operation order differs");
  Require(backend.delays.size() == 2U, "expected two settle delays");
  Require(backend.delays[0] == std::chrono::milliseconds(100) &&
              backend.delays[1] == std::chrono::milliseconds(100),
          "settle delay differs from 100 ms");
}

}  // namespace

int main() {
  RequireSequence(
      BootMode::kBootloader,
      {ModemOperation::kAssertRts, ModemOperation::kDeassertDtr,
       ModemOperation::kDelay, ModemOperation::kAssertDtr,
       ModemOperation::kDelay});
  RequireSequence(
      BootMode::kApplication,
      {ModemOperation::kAssertRts, ModemOperation::kDeassertDtr,
       ModemOperation::kDelay, ModemOperation::kDeassertRts,
       ModemOperation::kDelay});

  FakeBackend failure;
  failure.fail_at = 2U;
  Require(!mentor_pi::firmware::host::ExecuteBootSequence(
              BootMode::kBootloader, &failure),
          "ioctl failure was not propagated");
  Require(failure.operations.size() == 2U,
          "sequence continued after ioctl failure");
  Require(!mentor_pi::firmware::host::ExecuteBootSequence(
              BootMode::kApplication, nullptr),
          "null backend was accepted");
  Require(mentor_pi::firmware::host::ParseBootMode("bootloader") ==
              BootMode::kBootloader,
          "bootloader mode parsing failed");
  Require(mentor_pi::firmware::host::ParseBootMode("application") ==
              BootMode::kApplication,
          "application mode parsing failed");
  std::cout << "CH9102F boot-control tests passed.\n";
  return 0;
}
