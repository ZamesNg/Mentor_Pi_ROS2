#ifndef MENTOR_PI_MCU_DRIVERS_BUS_SERVO_UART_H_
#define MENTOR_PI_MCU_DRIVERS_BUS_SERVO_UART_H_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mentor_pi_mcu/domain/bus_servo.h"
#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"
#include "mentor_pi_mcu/drivers/hal.h"

namespace mentor_pi::mcu::drivers {

struct BusServoState {
  static constexpr std::uint16_t kFieldId = 1U;
  static constexpr std::uint16_t kFieldPosition = 2U;
  static constexpr std::uint16_t kFieldOffset = 4U;
  static constexpr std::uint16_t kFieldVoltage = 8U;
  static constexpr std::uint16_t kFieldTemperature = 16U;
  static constexpr std::uint16_t kFieldPositionLimits = 32U;
  static constexpr std::uint16_t kFieldVoltageLimits = 64U;
  static constexpr std::uint16_t kFieldTemperatureLimit = 128U;
  static constexpr std::uint16_t kFieldTorque = 256U;

  std::uint16_t valid_fields{0};
  std::uint8_t requested_id{0};
  std::uint8_t reported_id{0};
  std::int16_t position{0};
  std::int8_t offset{0};
  std::uint16_t voltage_mv{0};
  std::uint8_t temperature_c{0};
  std::uint16_t position_min{0};
  std::uint16_t position_max{0};
  std::uint16_t voltage_min_mv{0};
  std::uint16_t voltage_max_mv{0};
  std::uint8_t temperature_limit_c{0};
  bool torque_enabled{false};
};

struct BusServoPollResult {
  Result result{ResultCode::kBusy, 0};
  bool complete{false};
  std::uint16_t completed_mask{0};
  BusServoState state{};
};

// Owns one bounded UART5 operation at a time. Callers poll it from the
// bus-servo worker; no callback blocks and no heap is used.
class BusServoUartDriver {
 public:
  explicit BusServoUartDriver(HalfDuplexUart& uart) : uart_(uart) {}

  Result StartMove(const BusServoCommand& command, std::uint32_t now_ms);
  Result StartStop(const StopBusServosCommand& command, std::uint32_t now_ms);
  Result StartQuery(const GetBusServoStateCommand& command,
                    std::uint32_t now_ms);
  Result StartConfigure(const ConfigureBusServoCommand& command,
                        std::uint32_t now_ms);
  BusServoPollResult Poll(std::uint32_t now_ms);
  void Cancel();
  bool busy() const { return operation_ != Operation::kIdle; }

 private:
  enum class Operation : std::uint8_t {
    kIdle = 0,
    kMove = 1,
    kStop = 2,
    kQuery = 3,
    kConfigure = 4,
  };

  struct Step {
    BusServoFrame frame{};
    BusServoOpcode response_opcode{BusServoOpcode::kMoveTimeWrite};
    std::uint16_t field{0};
    std::uint8_t response_arguments{0};
    bool expects_response{false};
  };

  static constexpr std::size_t kMaximumSteps = kBusServoBatchCapacity;
  static constexpr std::uint32_t kOperationTimeoutMs = 200U;

  Result BeginOperation(Operation operation, std::uint32_t now_ms);
  Result AddStep(std::uint8_t id, BusServoOpcode opcode,
                 const std::uint8_t* arguments, std::size_t argument_count,
                 bool expects_response, std::uint8_t response_arguments,
                 std::uint16_t field);
  Result StartCurrentExchange();
  Result ConsumeReply(const std::uint8_t* reply, std::size_t size);
  void Finish(Result result);

  HalfDuplexUart& uart_;
  std::array<Step, kMaximumSteps> steps_{};
  std::array<std::uint8_t, kBusServoMaximumFrameBytes> reply_{};
  BusServoState state_{};
  Result terminal_result_{};
  Operation operation_{Operation::kIdle};
  std::uint32_t deadline_ms_{0};
  std::uint16_t completed_mask_{0};
  std::uint8_t step_count_{0};
  std::uint8_t step_index_{0};
  bool exchange_started_{false};
};

}  // namespace mentor_pi::mcu::drivers

#endif  // MENTOR_PI_MCU_DRIVERS_BUS_SERVO_UART_H_
