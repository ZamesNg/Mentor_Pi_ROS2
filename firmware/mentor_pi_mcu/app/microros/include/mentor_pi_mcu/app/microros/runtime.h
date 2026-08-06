// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_RUNTIME_H_
#define MENTOR_PI_MCU_APP_MICROROS_RUNTIME_H_

#if defined(MENTOR_PI_MICROROS_ENABLE_FAULT_PROXY)
#include "mentor_pi_mcu/app/microros/middleware_fault_proxy.h"
#endif
#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/app/microros/runtime_hooks.h"

namespace mentor_pi_mcu::app::microros {

// Must be called before static tasks are started. The table is copied into
// static storage; the caller must keep hooks.context alive for the firmware
// lifetime.
bool ConfigureMicroRosRuntime(const RuntimeHooks& hooks);

// One bounded lifecycle/executor slice, exposed for deterministic tests.
void RunMicroRosRuntimeOnce();

// Production task entry. This function never returns.
[[noreturn]] void RunMicroRosRuntime();

SessionState MicroRosSessionState();

#if defined(MENTOR_PI_MICROROS_ENABLE_FAULT_PROXY)
// Test builds may configure one deterministic fixed-storage fault plan. The
// production firmware does not define this flag and exposes no injection API.
bool ConfigureMicroRosMiddlewareFaultForTesting(
    const MiddlewareFaultPlan& plan);
void ClearMicroRosMiddlewareFaultForTesting();
MiddlewareBoundaryStats MicroRosMiddlewareStatsForTesting(
    MiddlewareBoundary boundary);
#endif

}  // namespace mentor_pi_mcu::app::microros

extern "C" [[noreturn]] void MentorPiMicroRosTaskMain(void* context);

#endif  // MENTOR_PI_MCU_APP_MICROROS_RUNTIME_H_
