#ifndef MENTOR_PI_MCU_DRIVERS_HAL_H_
#define MENTOR_PI_MCU_DRIVERS_HAL_H_

#include <cstddef>
#include <cstdint>

namespace mentor_pi::mcu::drivers {

enum class IoStatus : std::uint8_t {
  kOk = 0,
  kBusy = 1,
  kTimeout = 2,
  kIoError = 3,
};

// All deadlines are absolute monotonic times. Implementations must return by
// the deadline; drivers never wait indefinitely.
class RegisterI2c {
 public:
  virtual IoStatus Read(std::uint8_t address, std::uint8_t reg,
                        std::uint8_t* data, std::size_t size,
                        std::uint32_t deadline_us) = 0;
  virtual IoStatus Write(std::uint8_t address, std::uint8_t reg,
                         const std::uint8_t* data, std::size_t size,
                         std::uint32_t deadline_us) = 0;

 protected:
  // Drivers are statically composed and never deleted through this interface.
  ~RegisterI2c() = default;
};

class RawI2c {
 public:
  virtual IoStatus Write(std::uint8_t address, const std::uint8_t* data,
                         std::size_t size, std::uint32_t deadline_ms) = 0;

 protected:
  ~RawI2c() = default;
};

class HalfDuplexUart {
 public:
  // tx remains valid until PollExchange returns a terminal status or Cancel is
  // called. max_reply_size == 0 means transmit-only.
  virtual IoStatus BeginExchange(const std::uint8_t* tx, std::size_t tx_size,
                                 std::size_t max_reply_size,
                                 std::uint32_t deadline_ms) = 0;
  virtual IoStatus PollExchange(std::uint32_t now_ms, std::uint8_t* reply,
                                std::size_t capacity,
                                std::size_t* reply_size) = 0;
  virtual void Cancel() = 0;

 protected:
  ~HalfDuplexUart() = default;
};

class AsyncSpi {
 public:
  // data remains valid until PollTransmit returns a terminal status or Cancel.
  virtual IoStatus BeginTransmit(const std::uint8_t* data, std::size_t size,
                                 std::uint32_t deadline_us) = 0;
  virtual IoStatus PollTransmit(std::uint32_t now_us) = 0;
  virtual void Cancel() = 0;

 protected:
  ~AsyncSpi() = default;
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_HAL_H_
