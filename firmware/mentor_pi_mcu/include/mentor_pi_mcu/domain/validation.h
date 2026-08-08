#ifndef MENTOR_PI_MCU_DOMAIN_VALIDATION_H_
#define MENTOR_PI_MCU_DOMAIN_VALIDATION_H_

#include <cstdint>

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

Result ValidateMotorCommand(const MotorCommand& command, float max_rps);
Result ValidatePwmServoCommand(const PwmServoCommand& command);
Result ValidatePwmServoOffsets(const PwmServoOffsetCommand& command);
Result ValidateBusServoCommand(const BusServoCommand& command);
Result ValidateStopBusServosCommand(const StopBusServosCommand& command);
Result ValidateConfigureBusServoCommand(
    const ConfigureBusServoCommand& command);
Result ValidateSetMotorPidCommand(const SetMotorPidCommand& command);
Result ValidateGetBusServoStateCommand(const GetBusServoStateCommand& command);
Result ValidateLedCommand(const LedCommand& command);
Result ValidateBuzzerCommand(const BuzzerCommand& command);
Result ValidateRgbCommand(const RgbCommand& command);
Result ValidateOledCommand(const OledCommand& command);
Result ValidateBatteryThreshold(std::uint16_t threshold_mv);
bool IsValidMotorModel(MotorModel model);

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_VALIDATION_H_
