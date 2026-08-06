#ifndef MENTOR_PI_MCU_DOMAIN_STATE_MERGER_H_
#define MENTOR_PI_MCU_DOMAIN_STATE_MERGER_H_

#include "mentor_pi_mcu/domain/commands.h"
#include "mentor_pi_mcu/domain/result.h"

namespace mentor_pi::mcu {

Result MergeRgbCommand(const RgbCommand& command, RgbState* state);
Result MergeOledCommand(const OledCommand& command, OledState* state);

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_STATE_MERGER_H_
