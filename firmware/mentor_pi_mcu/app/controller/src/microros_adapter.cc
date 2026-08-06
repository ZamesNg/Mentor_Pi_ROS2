// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <cstdint>

#include "mentor_pi_mcu/app/controller/controller_runtime.h"

namespace mentor_pi_mcu::app::controller {
namespace {

using mentor_pi_mcu::app::microros::BatteryTelemetry;
using mentor_pi_mcu::app::microros::BatteryThresholdReply;
using mentor_pi_mcu::app::microros::ConfigureBusServoReply;
using mentor_pi_mcu::app::microros::GetBusServoStateReply;
using mentor_pi_mcu::app::microros::HealthSnapshot;
using mentor_pi_mcu::app::microros::ImuTelemetry;
using mentor_pi_mcu::app::microros::MotorModelReply;
using mentor_pi_mcu::app::microros::MotorTelemetry;
using mentor_pi_mcu::app::microros::PwmOffsetsReply;
using mentor_pi_mcu::app::microros::PwmServoTelemetry;
using mentor_pi_mcu::app::microros::RuntimeHooks;
using mentor_pi_mcu::app::microros::ServiceToken;
using mentor_pi_mcu::app::microros::StopBusServosReply;
using mentor_pi_mcu::app::microros::WorkerDiagnostics;

ControllerRuntime* Runtime(void* context) {
  return static_cast<ControllerRuntime*>(context);
}

std::uint32_t HookMonotonicMilliseconds(void* context) {
  return Runtime(context)->MonotonicMilliseconds();
}

std::uint32_t HookMonotonicMicroseconds(void* context) {
  return Runtime(context)->MonotonicMicroseconds();
}

void HookWaitMilliseconds(void* context, std::uint32_t maximum_ms) {
  Runtime(context)->WaitForMicroRos(maximum_ms);
}

void HookAdvanceHeartbeat(void* context) {
  Runtime(context)->AdvanceMicroRosHeartbeat();
}

void HookEmergencyStop(void* context) {
  Runtime(context)->EmergencyStopMotors();
}

void HookSetSessionActive(void* context, bool active,
                          std::uint32_t generation) {
  Runtime(context)->SetSessionActive(active, generation);
}

void HookInvalidateSessionWork(void* context, std::uint32_t generation) {
  Runtime(context)->InvalidateSessionWork(generation);
}

float HookMotorMaximumRps(void* context) {
  return Runtime(context)->MotorMaximumRps();
}

mentor_pi::mcu::CommandAdmission HookPublishMotor(
    void* context, const mentor_pi::mcu::MotorCommand& command,
    std::uint32_t accepted_at_us) {
  return Runtime(context)->PublishMotorCommand(command, accepted_at_us);
}

mentor_pi::mcu::CommandAdmission HookPublishPwm(
    void* context, const mentor_pi::mcu::PwmServoCommand& command) {
  return Runtime(context)->PublishPwmServoCommand(command);
}

mentor_pi::mcu::CommandAdmission HookPublishBus(
    void* context, const mentor_pi::mcu::BusServoCommand& command) {
  return Runtime(context)->PublishBusServoCommand(command);
}

mentor_pi::mcu::CommandAdmission HookPublishLed(
    void* context, const mentor_pi::mcu::LedCommand& command) {
  return Runtime(context)->PublishLedCommand(command);
}

mentor_pi::mcu::CommandAdmission HookPublishBuzzer(
    void* context, const mentor_pi::mcu::BuzzerCommand& command) {
  return Runtime(context)->PublishBuzzerCommand(command);
}

mentor_pi::mcu::CommandAdmission HookPublishRgb(
    void* context, const mentor_pi::mcu::RgbCommand& command) {
  return Runtime(context)->PublishRgbCommand(command);
}

mentor_pi::mcu::CommandAdmission HookPublishOled(
    void* context, const mentor_pi::mcu::OledCommand& command) {
  return Runtime(context)->PublishOledCommand(command);
}

bool HookReadMotor(void* context, MotorTelemetry* output) {
  return Runtime(context)->ReadMotorTelemetry(output);
}

bool HookReadPwm(void* context, PwmServoTelemetry* output) {
  return Runtime(context)->ReadPwmServoTelemetry(output);
}

bool HookReadImu(void* context, ImuTelemetry* output) {
  return Runtime(context)->ReadImuTelemetry(output);
}

bool HookReadBattery(void* context, BatteryTelemetry* output) {
  return Runtime(context)->ReadBatteryTelemetry(output);
}

bool HookPopButton(void* context, mentor_pi::mcu::ButtonEvent* output) {
  return Runtime(context)->PopButtonEvent(output);
}

void HookReadHealth(void* context, HealthSnapshot* output) {
  Runtime(context)->ReadHealth(output);
}

void HookReadDiagnostics(void* context, WorkerDiagnostics* output) {
  Runtime(context)->ReadWorkerDiagnostics(output);
}

bool HookDispatchMotorModel(void* context, ServiceToken token,
                            mentor_pi::mcu::MotorModel model) {
  return Runtime(context)->DispatchMotorModel(token, model);
}

bool HookPollMotorModel(void* context, ServiceToken token,
                        MotorModelReply* output) {
  return Runtime(context)->PollMotorModel(token, output);
}

bool HookDispatchPwmOffsets(
    void* context, ServiceToken token,
    const mentor_pi::mcu::PwmServoOffsetCommand& command) {
  return Runtime(context)->DispatchPwmOffsets(token, command);
}

bool HookPollPwmOffsets(void* context, ServiceToken token,
                        PwmOffsetsReply* output) {
  return Runtime(context)->PollPwmOffsets(token, output);
}

bool HookDispatchBatteryThreshold(void* context, ServiceToken token,
                                  std::uint16_t threshold_mv) {
  return Runtime(context)->DispatchBatteryThreshold(token, threshold_mv);
}

bool HookPollBatteryThreshold(void* context, ServiceToken token,
                              BatteryThresholdReply* output) {
  return Runtime(context)->PollBatteryThreshold(token, output);
}

bool HookDispatchBusGet(
    void* context, ServiceToken token,
    const mentor_pi::mcu::GetBusServoStateCommand& command) {
  return Runtime(context)->DispatchBusGetState(token, command);
}

bool HookPollBusGet(void* context, ServiceToken token,
                    GetBusServoStateReply* output) {
  return Runtime(context)->PollBusGetState(token, output);
}

bool HookDispatchBusConfigure(
    void* context, ServiceToken token,
    const mentor_pi::mcu::ConfigureBusServoCommand& command) {
  return Runtime(context)->DispatchBusConfigure(token, command);
}

bool HookPollBusConfigure(void* context, ServiceToken token,
                          ConfigureBusServoReply* output) {
  return Runtime(context)->PollBusConfigure(token, output);
}

bool HookDispatchBusStop(void* context, ServiceToken token,
                         const mentor_pi::mcu::StopBusServosCommand& command) {
  return Runtime(context)->DispatchBusStop(token, command);
}

bool HookPollBusStop(void* context, ServiceToken token,
                     StopBusServosReply* output) {
  return Runtime(context)->PollBusStop(token, output);
}

}  // namespace

RuntimeHooks ControllerRuntime::BuildMicroRosHooks() {
  RuntimeHooks hooks{};
  hooks.context = this;
  hooks.monotonic_milliseconds = &HookMonotonicMilliseconds;
  hooks.monotonic_microseconds = &HookMonotonicMicroseconds;
  hooks.wait_milliseconds = &HookWaitMilliseconds;
  hooks.advance_task_heartbeat = &HookAdvanceHeartbeat;
  hooks.emergency_stop_motors = &HookEmergencyStop;
  hooks.set_session_active = &HookSetSessionActive;
  hooks.invalidate_session_work = &HookInvalidateSessionWork;
  hooks.motor_max_rps = &HookMotorMaximumRps;
  hooks.publish_motor_command = &HookPublishMotor;
  hooks.publish_pwm_servo_command = &HookPublishPwm;
  hooks.publish_bus_servo_command = &HookPublishBus;
  hooks.publish_led_command = &HookPublishLed;
  hooks.publish_buzzer_command = &HookPublishBuzzer;
  hooks.publish_rgb_command = &HookPublishRgb;
  hooks.publish_oled_command = &HookPublishOled;
  hooks.read_motor_telemetry = &HookReadMotor;
  hooks.read_pwm_servo_telemetry = &HookReadPwm;
  hooks.read_imu_telemetry = &HookReadImu;
  hooks.read_battery_telemetry = &HookReadBattery;
  hooks.pop_button_event = &HookPopButton;
  hooks.read_health = &HookReadHealth;
  hooks.read_worker_diagnostics = &HookReadDiagnostics;
  hooks.dispatch_motor_model = &HookDispatchMotorModel;
  hooks.poll_motor_model = &HookPollMotorModel;
  hooks.dispatch_pwm_offsets = &HookDispatchPwmOffsets;
  hooks.poll_pwm_offsets = &HookPollPwmOffsets;
  hooks.dispatch_battery_threshold = &HookDispatchBatteryThreshold;
  hooks.poll_battery_threshold = &HookPollBatteryThreshold;
  hooks.dispatch_bus_get_state = &HookDispatchBusGet;
  hooks.poll_bus_get_state = &HookPollBusGet;
  hooks.dispatch_bus_configure = &HookDispatchBusConfigure;
  hooks.poll_bus_configure = &HookPollBusConfigure;
  hooks.dispatch_bus_stop = &HookDispatchBusStop;
  hooks.poll_bus_stop = &HookPollBusStop;
  return hooks;
}

}  // namespace mentor_pi_mcu::app::controller
