#include "ch9102_boot_control.h"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace mentor_pi::tools {
namespace {

constexpr std::chrono::milliseconds kModemSettleTime{100};

const std::vector<ModemStep> kBootloaderSequence{
    {ModemOperation::kAssertRts, {}},
    {ModemOperation::kDeassertDtr, {}},
    {ModemOperation::kDelay, kModemSettleTime},
    {ModemOperation::kAssertDtr, {}},
    {ModemOperation::kDelay, kModemSettleTime},
};

const std::vector<ModemStep> kApplicationSequence{
    {ModemOperation::kAssertRts, {}},
    {ModemOperation::kDeassertDtr, {}},
    {ModemOperation::kDelay, kModemSettleTime},
    {ModemOperation::kDeassertRts, {}},
    {ModemOperation::kDelay, kModemSettleTime},
};

std::string Trim(std::string value) {
  while (!value.empty() &&
         (value.back() == '\n' || value.back() == '\r' ||
          value.back() == ' ' || value.back() == '\t')) {
    value.pop_back();
  }
  return value;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return {};
  }
  std::string value;
  std::getline(input, value);
  return Trim(value);
}

bool HasCh9102Identity(const std::filesystem::path& tty_name) {
  std::error_code error;
  std::filesystem::path node =
      std::filesystem::canonical("/sys/class/tty" / tty_name / "device", error);
  if (error) {
    return false;
  }
  while (node != node.root_path()) {
    const std::string vendor = ReadTextFile(node / "idVendor");
    const std::string product = ReadTextFile(node / "idProduct");
    if (!vendor.empty() || !product.empty()) {
      return vendor == "1a86" && product == "55d4";
    }
    node = node.parent_path();
  }
  return false;
}

class SerialModemBackend final : public ModemBackend {
 public:
  explicit SerialModemBackend(const std::string& device) {
    struct stat path_status {};
    if (device.empty() || device.rfind("/dev/", 0) != 0 ||
        device.find("/../") != std::string::npos ||
        device.find("/./") != std::string::npos ||
        stat(device.c_str(), &path_status) != 0 ||
        !S_ISCHR(path_status.st_mode)) {
      throw std::runtime_error("device is not an existing /dev character device");
    }

    std::error_code error;
    const std::filesystem::path resolved =
        std::filesystem::canonical(device, error);
    if (error || resolved.parent_path() != "/dev" ||
        !HasCh9102Identity(resolved.filename())) {
      throw std::runtime_error(
          "device is not the Mentor Pi CH9102F (expected 1a86:55d4)");
    }

    descriptor_ =
        open(resolved.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC | O_NOFOLLOW |
                                   O_NONBLOCK);
    if (descriptor_ < 0) {
      throw std::runtime_error("cannot open serial device: " +
                               std::string(std::strerror(errno)));
    }
    struct stat descriptor_status {};
    if (fstat(descriptor_, &descriptor_status) != 0) {
      CloseAndThrow("cannot inspect the opened serial device");
    }
    if (descriptor_status.st_dev != path_status.st_dev ||
        descriptor_status.st_ino != path_status.st_ino) {
      errno = EAGAIN;
      CloseAndThrow("serial device changed while it was opened");
    }
    if (flock(descriptor_, LOCK_EX | LOCK_NB) != 0) {
      CloseAndThrow("serial device is already owned");
    }
    if (ioctl(descriptor_, TIOCEXCL) != 0) {
      CloseAndThrow("cannot claim exclusive serial access");
    }

    struct termios options {};
    if (tcgetattr(descriptor_, &options) != 0) {
      CloseAndThrow("cannot read serial settings");
    }
    options.c_cflag |= CLOCAL;
    options.c_cflag &= static_cast<tcflag_t>(~HUPCL);
    if (tcsetattr(descriptor_, TCSANOW, &options) != 0) {
      CloseAndThrow("cannot apply safe serial settings");
    }
    const int flags = fcntl(descriptor_, F_GETFL);
    if (flags < 0 || fcntl(descriptor_, F_SETFL, flags & ~O_NONBLOCK) != 0) {
      CloseAndThrow("cannot make serial access blocking");
    }
  }

  ~SerialModemBackend() override {
    if (descriptor_ >= 0) {
      close(descriptor_);
    }
  }

  bool Apply(ModemOperation operation) override {
    int request = 0;
    int bits = 0;
    switch (operation) {
      case ModemOperation::kAssertRts:
        request = TIOCMBIS;
        bits = TIOCM_RTS;
        break;
      case ModemOperation::kDeassertRts:
        request = TIOCMBIC;
        bits = TIOCM_RTS;
        break;
      case ModemOperation::kAssertDtr:
        request = TIOCMBIS;
        bits = TIOCM_DTR;
        break;
      case ModemOperation::kDeassertDtr:
        request = TIOCMBIC;
        bits = TIOCM_DTR;
        break;
      case ModemOperation::kDelay:
        return false;
    }
    return ioctl(descriptor_, request, &bits) == 0;
  }

  void Delay(std::chrono::milliseconds duration) override {
    std::this_thread::sleep_for(duration);
  }

 private:
  [[noreturn]] void CloseAndThrow(const std::string& message) {
    const int error_number = errno;
    close(descriptor_);
    descriptor_ = -1;
    throw std::runtime_error(message + ": " + std::strerror(error_number));
  }

  int descriptor_{-1};
};

}  // namespace

BootMode ParseBootMode(std::string_view value) {
  if (value == "bootloader") {
    return BootMode::kBootloader;
  }
  if (value == "application") {
    return BootMode::kApplication;
  }
  throw std::invalid_argument("mode must be bootloader or application");
}

const std::vector<ModemStep>& BootSequence(BootMode mode) {
  return mode == BootMode::kBootloader ? kBootloaderSequence
                                       : kApplicationSequence;
}

bool ExecuteBootSequence(BootMode mode, ModemBackend* backend) {
  if (backend == nullptr) {
    return false;
  }
  for (const ModemStep& step : BootSequence(mode)) {
    if (step.operation == ModemOperation::kDelay) {
      backend->Delay(step.delay);
    } else if (!backend->Apply(step.operation)) {
      return false;
    }
  }
  return true;
}

}  // namespace mentor_pi::tools

#ifndef MENTOR_PI_CH9102_BOOT_CONTROL_NO_MAIN
namespace {

int Main(int argc, char** argv) {
  if (argc != 5 || std::string_view(argv[1]) != "--device" ||
      std::string_view(argv[3]) != "--mode") {
    std::cerr << "Usage: ch9102_boot_control --device /dev/mentor_pi_mcu "
                 "--mode bootloader|application\n";
    return 2;
  }
  try {
    const mentor_pi::tools::BootMode mode =
        mentor_pi::tools::ParseBootMode(argv[4]);
    mentor_pi::tools::SerialModemBackend backend(argv[2]);
    if (!mentor_pi::tools::ExecuteBootSequence(mode, &backend)) {
      std::cerr << "CH9102F modem-control operation failed: "
                << std::strerror(errno) << '\n';
      return 1;
    }
    std::cout << "CH9102F reset sequence completed: " << argv[4] << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "CH9102F boot control failed: " << error.what() << '\n';
    return 1;
  }
}

}  // namespace

int main(int argc, char** argv) { return Main(argc, argv); }
#endif
