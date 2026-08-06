// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string_view>
#include <type_traits>

#include "mentor_pi_mcu/app/microros/endpoint_contract.h"
#include "mentor_pi_mcu/app/microros/middleware_fault_proxy.h"
#include "mentor_pi_mcu/app/microros/runtime_core.h"
#include "mentor_pi_mcu/platform/stm32/transport.h"

namespace microros = mentor_pi_mcu::app::microros;
namespace stm32 = mentor_pi_mcu::platform::stm32;

namespace {

void Check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "check failed at line " << line << ": " << expression << '\n';
    std::exit(EXIT_FAILURE);
  }
}

#define CHECK(expression) Check((expression), #expression, __LINE__)

void TestBackoffAndSessions() {
  microros::SessionLifecycle lifecycle;
  CHECK(lifecycle.state() == microros::SessionState::kSafeBoot);
  lifecycle.LeaveSafeBoot();
  CHECK(lifecycle.state() == microros::SessionState::kWaitAgent);

  constexpr std::array<std::uint32_t, 7> kExpectedBackoff{
      100U, 200U, 400U, 800U, 1600U, 2000U, 2000U};
  std::uint32_t now_ms = 10U;
  for (const std::uint32_t expected : kExpectedBackoff) {
    lifecycle.OnAgentPing(false, now_ms);
    CHECK(lifecycle.state() == microros::SessionState::kBackoff);
    CHECK(lifecycle.current_backoff_ms() == expected);
    CHECK(!lifecycle.AdvanceBackoff(now_ms + expected - 1U));
    now_ms += expected;
    CHECK(lifecycle.AdvanceBackoff(now_ms));
    CHECK(lifecycle.state() == microros::SessionState::kWaitAgent);
  }

  lifecycle.OnAgentPing(true, now_ms);
  CHECK(lifecycle.state() == microros::SessionState::kCreateEntities);
  lifecycle.OnEntitiesCreated(true, now_ms);
  CHECK(lifecycle.state() == microros::SessionState::kActive);
  CHECK(lifecycle.session_generation() == 1U);
  CHECK(lifecycle.agent_reconnects() == 0U);

  lifecycle.BeginTeardown(microros::TeardownReason::kAgentLost);
  CHECK(lifecycle.state() == microros::SessionState::kTeardown);
  CHECK(lifecycle.teardown_reason() == microros::TeardownReason::kAgentLost);
  lifecycle.OnTeardownComplete(now_ms);
  CHECK(lifecycle.current_backoff_ms() == 100U);
  now_ms += 100U;
  CHECK(lifecycle.AdvanceBackoff(now_ms));
  lifecycle.OnAgentPing(true, now_ms);
  lifecycle.OnEntitiesCreated(true, now_ms);
  CHECK(lifecycle.session_generation() == 2U);
  CHECK(lifecycle.agent_reconnects() == 1U);
  CHECK(lifecycle.teardown_reason() == microros::TeardownReason::kAgentLost);
}

void TestFailureAndWrapHelpers() {
  microros::SessionLifecycle lifecycle;
  lifecycle.LeaveSafeBoot();
  lifecycle.OnAgentPing(true, 0U);
  lifecycle.OnEntitiesCreated(false, 0U);
  CHECK(lifecycle.state() == microros::SessionState::kTeardown);
  CHECK(lifecycle.teardown_reason() == microros::TeardownReason::kEntityError);

  CHECK(microros::NextNonzeroGeneration(0U) == 1U);
  CHECK(microros::NextNonzeroGeneration(
            std::numeric_limits<std::uint32_t>::max()) == 1U);
  CHECK(!microros::DeadlineReached(
      std::numeric_limits<std::uint32_t>::max() - 4U, 5U));
  CHECK(microros::DeadlineReached(5U, 5U));
  CHECK(microros::DeadlineReached(6U, 5U));
  CHECK(microros::NextNonzeroGeneration(41U) == 42U);
}

void TestOutOfOrderLifecycleEventsAreIgnored() {
  microros::SessionLifecycle lifecycle;

  lifecycle.OnAgentPing(true, 1U);
  lifecycle.OnEntitiesCreated(true, 1U);
  lifecycle.OnTeardownComplete(1U);
  CHECK(!lifecycle.AdvanceBackoff(1U));
  CHECK(lifecycle.state() == microros::SessionState::kSafeBoot);

  lifecycle.LeaveSafeBoot();
  lifecycle.LeaveSafeBoot();
  lifecycle.OnEntitiesCreated(true, 2U);
  lifecycle.OnTeardownComplete(2U);
  CHECK(!lifecycle.AdvanceBackoff(2U));
  CHECK(lifecycle.state() == microros::SessionState::kWaitAgent);

  lifecycle.OnAgentPing(true, 3U);
  lifecycle.OnAgentPing(false, 3U);
  lifecycle.OnTeardownComplete(3U);
  CHECK(lifecycle.state() == microros::SessionState::kCreateEntities);

  lifecycle.OnEntitiesCreated(false, 4U);
  lifecycle.BeginTeardown(microros::TeardownReason::kTaskStall);
  CHECK(lifecycle.teardown_reason() == microros::TeardownReason::kEntityError);
  lifecycle.OnTeardownComplete(5U);
  lifecycle.OnTeardownComplete(5U);
  CHECK(lifecycle.state() == microros::SessionState::kBackoff);
}

void TestTransportFaultClassification() {
  const auto bit = [](stm32::Usart1Error error) {
    return static_cast<std::uint8_t>(error);
  };
  CHECK(microros::ClassifyTransportErrorFlags(0U) ==
        microros::TeardownReason::kNone);
  CHECK(microros::ClassifyTransportErrorFlags(
            bit(stm32::Usart1Error::kFraming)) ==
        microros::TeardownReason::kUsart1Error);
  CHECK(
      microros::ClassifyTransportErrorFlags(bit(stm32::Usart1Error::kNoise)) ==
      microros::TeardownReason::kUsart1Error);
  CHECK(microros::ClassifyTransportErrorFlags(
            bit(stm32::Usart1Error::kOverrun)) ==
        microros::TeardownReason::kUsart1Error);
  CHECK(
      microros::ClassifyTransportErrorFlags(bit(stm32::Usart1Error::kParity)) ==
      microros::TeardownReason::kUsart1Error);
  CHECK(microros::ClassifyTransportErrorFlags(bit(stm32::Usart1Error::kDma)) ==
        microros::TeardownReason::kUsart1Error);
  CHECK(
      microros::ClassifyTransportErrorFlags(bit(stm32::Usart1Error::kTxDma)) ==
      microros::TeardownReason::kTxTimeout);
  CHECK(microros::ClassifyTransportErrorFlags(
            bit(stm32::Usart1Error::kTxTimeout)) ==
        microros::TeardownReason::kTxTimeout);
  CHECK(microros::ClassifyTransportErrorFlags(
            bit(stm32::Usart1Error::kRxRingOverrun)) ==
        microros::TeardownReason::kRxOverrun);

  const std::uint8_t all = static_cast<std::uint8_t>(
      bit(stm32::Usart1Error::kRxRingOverrun) |
      bit(stm32::Usart1Error::kTxTimeout) | bit(stm32::Usart1Error::kDma));
  CHECK(microros::ClassifyTransportErrorFlags(all) ==
        microros::TeardownReason::kRxOverrun);
}

void TestEndpointInventoryAndQos() {
  using microros::Reliability;
  constexpr std::array<std::string_view, 7> kPublishers{
      "motors/state",  "pwm_servos/state", "imu",        "buttons/events",
      "battery/state", "heartbeat",        "diagnostics"};
  constexpr std::array<std::string_view, 7> kSubscriptions{
      "motors/command", "pwm_servos/command", "bus_servos/command",
      "leds/command",   "buzzer/command",     "rgb/command",
      "oled/command"};
  constexpr std::array<std::string_view, 6> kServices{
      "motors/set_model",     "pwm_servos/set_offsets",
      "bus_servos/get_state", "bus_servos/configure",
      "bus_servos/stop",      "battery/set_low_threshold"};

  CHECK(microros::kPublisherEndpoints.size() == kPublishers.size());
  CHECK(microros::kSubscriptionEndpoints.size() == kSubscriptions.size());
  CHECK(microros::kServiceEndpoints.size() == kServices.size());
  for (std::size_t index = 0U; index < kPublishers.size(); ++index) {
    CHECK(microros::kPublisherEndpoints[index].relative_name ==
          kPublishers[index]);
  }
  for (std::size_t index = 0U; index < kSubscriptions.size(); ++index) {
    CHECK(microros::kSubscriptionEndpoints[index].relative_name ==
          kSubscriptions[index]);
    CHECK(microros::kSubscriptionEndpoints[index].depth == 1U);
  }
  for (std::size_t index = 0U; index < kServices.size(); ++index) {
    CHECK(microros::kServiceEndpoints[index].relative_name == kServices[index]);
  }

  CHECK(microros::kPublisherEndpoints[0].reliability ==
        Reliability::kBestEffort);
  CHECK(microros::kPublisherEndpoints[1].reliability ==
        Reliability::kBestEffort);
  CHECK(microros::kPublisherEndpoints[2].reliability ==
        Reliability::kBestEffort);
  CHECK(microros::kPublisherEndpoints[3].reliability == Reliability::kReliable);
  CHECK(microros::kPublisherEndpoints[3].depth == 8U);
  CHECK(microros::kSubscriptionEndpoints[0].reliability ==
        Reliability::kBestEffort);
  CHECK(microros::kSubscriptionEndpoints[1].reliability ==
        Reliability::kBestEffort);
  CHECK(microros::kSubscriptionEndpoints[2].reliability ==
        Reliability::kBestEffort);
  for (std::size_t index = 3U; index < microros::kSubscriptionEndpoints.size();
       ++index) {
    CHECK(microros::kSubscriptionEndpoints[index].reliability ==
          Reliability::kReliable);
  }
}

void TestRoundRobinCursor() {
  microros::RoundRobinCursor<4U> cursor;
  CHECK(cursor.Peek() == 0U);
  CHECK(cursor.Peek(1U) == 1U);
  CHECK(cursor.Peek(4U) == 0U);

  constexpr std::array<bool, 4U> kReady{false, true, false, true};
  constexpr std::array<std::size_t, 6U> kExpected{1U, 3U, 1U, 3U, 1U, 3U};
  for (const std::size_t expected : kExpected) {
    bool selected = false;
    for (std::size_t offset = 0U; offset < kReady.size(); ++offset) {
      const std::size_t candidate = cursor.Peek(offset);
      if (!kReady[candidate]) {
        continue;
      }
      CHECK(candidate == expected);
      cursor.AdvancePast(candidate);
      selected = true;
      break;
    }
    CHECK(selected);
  }

  cursor.Reset();
  CHECK(cursor.Peek() == 0U);
  cursor.AdvancePast(3U);
  CHECK(cursor.Peek() == 0U);
}

void TestActiveSliceBudget() {
  microros::ActiveSliceBudget budget;
  budget.Reset(microros::ActiveWorkClass::kService);
  CHECK(!budget.blocking_operation_started());
  CHECK(!budget.TryStartBlockingOperation(
      microros::ActiveWorkClass::kReliableTelemetry));
  CHECK(!budget.blocking_operation_started());
  CHECK(budget.TryStartBlockingOperation(microros::ActiveWorkClass::kService));
  CHECK(!budget.TryStartBlockingOperation(
      microros::ActiveWorkClass::kMaintenance));
  CHECK(budget.blocking_operation_started());

  budget.Reset(microros::ActiveWorkClass::kMaintenance);
  CHECK(!budget.blocking_operation_started());
  CHECK(!budget.TryStartBlockingOperation(microros::ActiveWorkClass::kService));
  CHECK(budget.TryStartBlockingOperation(
      microros::ActiveWorkClass::kMaintenance));
}

void TestActiveWorkScheduler() {
  microros::ActiveWorkScheduler scheduler;
  constexpr std::array<microros::ActiveWorkClass, 9U> kExpected{
      microros::ActiveWorkClass::kService,
      microros::ActiveWorkClass::kReliableTelemetry,
      microros::ActiveWorkClass::kMaintenance,
      microros::ActiveWorkClass::kService,
      microros::ActiveWorkClass::kReliableTelemetry,
      microros::ActiveWorkClass::kMaintenance,
      microros::ActiveWorkClass::kService,
      microros::ActiveWorkClass::kReliableTelemetry,
      microros::ActiveWorkClass::kMaintenance};
  for (const auto expected : kExpected) {
    CHECK(scheduler.Next() == expected);
  }
  scheduler.Reset();
  CHECK(scheduler.Next() == microros::ActiveWorkClass::kService);
}

void ExpectCreateStep(microros::EntityCreateCursor* cursor,
                      microros::EntityCreateStepKind kind, std::size_t index,
                      std::uint32_t deadline_ms, std::size_t* boundary_count) {
  CHECK(cursor->started());
  CHECK(!cursor->complete());
  const microros::EntityCreateStep step = cursor->current();
  CHECK(step.kind == kind);
  CHECK(step.index == index);
  CHECK(step.deadline_ms == deadline_ms);
  ++(*boundary_count);
  cursor->Advance();
}

void ExpectDestroyStep(microros::EntityDestroyCursor* cursor,
                       microros::EntityDestroyStepKind kind, std::size_t index,
                       std::size_t* boundary_count) {
  CHECK(cursor->started());
  CHECK(!cursor->complete());
  const microros::EntityDestroyStep step = cursor->current();
  CHECK(step.kind == kind);
  CHECK(step.index == index);
  CHECK(step.deadline_ms == microros::kDestroyCallTimeoutMs);
  ++(*boundary_count);
  cursor->Advance();
}

void TestEntityCreateCursorOrder() {
  microros::EntityCreateCursor cursor;
  CHECK(!cursor.started());
  CHECK(!cursor.complete());
  cursor.Reset(123U);
  CHECK(cursor.phase_start_ms() == 123U);

  std::size_t boundary_count = 0U;
  ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kSupportInit, 0U,
                   microros::kCreateCallTimeoutMs, &boundary_count);
  ExpectCreateStep(&cursor,
                   microros::EntityCreateStepKind::kSetContextCreateTimeout, 0U,
                   microros::kCreateCallTimeoutMs, &boundary_count);
  ExpectCreateStep(&cursor,
                   microros::EntityCreateStepKind::kSetContextDestroyTimeout,
                   0U, microros::kCreateCallTimeoutMs, &boundary_count);
  ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kNodeInit, 0U,
                   microros::kCreateCallTimeoutMs, &boundary_count);
  for (std::size_t index = 0U; index < microros::kLifecyclePublisherCount;
       ++index) {
    ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kPublisherInit,
                     index, microros::kCreateCallTimeoutMs, &boundary_count);
    ExpectCreateStep(&cursor,
                     microros::EntityCreateStepKind::kSetPublisherTimeout,
                     index, microros::kCreateCallTimeoutMs, &boundary_count);
  }
  for (std::size_t index = 0U; index < microros::kLifecycleSubscriptionCount;
       ++index) {
    ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kSubscriptionInit,
                     index, microros::kCreateCallTimeoutMs, &boundary_count);
  }
  for (std::size_t index = 0U; index < microros::kLifecycleServiceCount;
       ++index) {
    ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kServiceInit,
                     index, microros::kCreateCallTimeoutMs, &boundary_count);
    ExpectCreateStep(&cursor,
                     microros::EntityCreateStepKind::kSetServiceTimeout, index,
                     microros::kCreateCallTimeoutMs, &boundary_count);
  }
  ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kExecutorInit, 0U,
                   microros::kCreateCallTimeoutMs, &boundary_count);
  for (std::size_t index = 0U; index < microros::kLifecycleSubscriptionCount;
       ++index) {
    ExpectCreateStep(&cursor,
                     microros::EntityCreateStepKind::kExecutorAddSubscription,
                     index, microros::kCreateCallTimeoutMs, &boundary_count);
  }
  ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kExecutorPrime, 0U,
                   microros::kExecutorWaitMs, &boundary_count);
  ExpectCreateStep(&cursor, microros::EntityCreateStepKind::kInitialTimeSync,
                   0U, microros::kInitialTimeSyncTimeoutMs, &boundary_count);
  CHECK(cursor.complete());
  CHECK(boundary_count == microros::kLifecycleCreateBoundaryCount);

  cursor.Clear();
  CHECK(!cursor.started());
  CHECK(cursor.current().kind == microros::EntityCreateStepKind::kComplete);
}

void TestEntityDestroyCursorOrderAndAbandon() {
  microros::EntityDestroyCursor cursor;
  cursor.Reset(456U);
  CHECK(cursor.phase_start_ms() == 456U);
  CHECK(!cursor.remote_waits_abandoned());

  std::size_t boundary_count = 0U;
  ExpectDestroyStep(&cursor,
                    microros::EntityDestroyStepKind::kSetContextDestroyTimeout,
                    0U, &boundary_count);
  ExpectDestroyStep(&cursor, microros::EntityDestroyStepKind::kExecutorFini, 0U,
                    &boundary_count);
  for (std::size_t index = microros::kLifecycleServiceCount; index > 0U;
       --index) {
    ExpectDestroyStep(&cursor, microros::EntityDestroyStepKind::kServiceFini,
                      index - 1U, &boundary_count);
  }
  for (std::size_t index = microros::kLifecycleSubscriptionCount; index > 0U;
       --index) {
    ExpectDestroyStep(&cursor,
                      microros::EntityDestroyStepKind::kSubscriptionFini,
                      index - 1U, &boundary_count);
  }
  for (std::size_t index = microros::kLifecyclePublisherCount; index > 0U;
       --index) {
    ExpectDestroyStep(&cursor, microros::EntityDestroyStepKind::kPublisherFini,
                      index - 1U, &boundary_count);
  }
  ExpectDestroyStep(&cursor, microros::EntityDestroyStepKind::kNodeFini, 0U,
                    &boundary_count);
  ExpectDestroyStep(&cursor, microros::EntityDestroyStepKind::kSupportFini, 0U,
                    &boundary_count);
  CHECK(cursor.complete());
  CHECK(boundary_count == microros::kLifecycleDestroyBoundaryCount);

  cursor.Reset(1000U);
  cursor.Advance();
  cursor.Advance();
  const microros::EntityDestroyStep pending = cursor.current();
  CHECK(pending.kind == microros::EntityDestroyStepKind::kServiceFini);
  CHECK(pending.index == microros::kLifecycleServiceCount - 1U);
  cursor.RequestRemoteWaitAbandon();
  cursor.RequestRemoteWaitAbandon();
  CHECK(cursor.current().kind ==
        microros::EntityDestroyStepKind::kDisableRemoteWaits);
  CHECK(cursor.current().deadline_ms == 0U);
  cursor.Advance();
  CHECK(cursor.remote_waits_abandoned());
  CHECK(cursor.current().kind == pending.kind);
  CHECK(cursor.current().index == pending.index);

  cursor.Reset(2000U);
  cursor.RequestRemoteWaitAbandon();
  CHECK(cursor.current().kind ==
        microros::EntityDestroyStepKind::kDisableRemoteWaits);
  cursor.Advance();
  CHECK(cursor.remote_waits_abandoned());
  CHECK(cursor.current().kind ==
        microros::EntityDestroyStepKind::kExecutorFini);

  cursor.Reset(3000U);
  cursor.MarkRemoteWaitsAbandonedWithoutBoundary();
  CHECK(cursor.remote_waits_abandoned());
  CHECK(cursor.current().kind ==
        microros::EntityDestroyStepKind::kExecutorFini);
}

void TestLifecycleDeadlineAndWrap() {
  CHECK(microros::OperationFitsInPhase(100U, 100U, 2000U, 40U));
  CHECK(microros::OperationFitsInPhase(100U, 2059U, 2000U, 40U));
  CHECK(!microros::OperationFitsInPhase(100U, 2060U, 2000U, 40U));
  CHECK(!microros::OperationFitsInPhase(100U, 2100U, 2000U, 1U));
  CHECK(!microros::PhaseDeadlineReached(100U, 2099U, 2000U));
  CHECK(microros::PhaseDeadlineReached(100U, 2100U, 2000U));

  constexpr std::uint32_t kWrapStart =
      std::numeric_limits<std::uint32_t>::max() - 100U;
  microros::EntityCreateCursor create;
  create.Reset(kWrapStart);
  CHECK(create.CurrentOperationFits(kWrapStart + 1959U));
  CHECK(!create.CurrentOperationFits(kWrapStart + 1960U));
  CHECK(!create.PhaseDeadlineReached(kWrapStart + 1999U));
  CHECK(create.PhaseDeadlineReached(kWrapStart + 2000U));

  microros::EntityDestroyCursor destroy;
  destroy.Reset(kWrapStart);
  CHECK(destroy.CurrentRemoteOperationFits(kWrapStart + 489U));
  CHECK(!destroy.CurrentRemoteOperationFits(kWrapStart + 490U));
  CHECK(!destroy.RemotePhaseDeadlineReached(kWrapStart + 499U));
  CHECK(destroy.RemotePhaseDeadlineReached(kWrapStart + 500U));
}

void TestIncrementalLifecycleDriverAndReconnects() {
  CHECK(microros::kLifecycleCreateMaximumDeclaredMilliseconds == 1867U);
  CHECK(microros::kLifecycleDestroyMaximumDeclaredMilliseconds == 263U);
  std::uint32_t now_ms = 0U;
  std::uint32_t last_heartbeat_ms = 0U;
  std::uint32_t maximum_heartbeat_gap_ms = 0U;
  for (std::size_t cycle = 0U; cycle < 100U; ++cycle) {
    microros::EntityCreateCursor create;
    create.Reset(now_ms);
    std::size_t create_slices = 0U;
    while (!create.complete()) {
      CHECK(create.CurrentOperationFits(now_ms));
      const microros::EntityCreateStep step = create.current();
      std::size_t boundaries_started_this_slice = 0U;
      ++boundaries_started_this_slice;
      ++create_slices;
      now_ms += step.deadline_ms;
      maximum_heartbeat_gap_ms =
          std::max(maximum_heartbeat_gap_ms, now_ms - last_heartbeat_ms);
      last_heartbeat_ms = now_ms;
      CHECK(boundaries_started_this_slice == 1U);
      create.Advance();
      ++now_ms;
    }
    CHECK(create_slices == microros::kLifecycleCreateBoundaryCount);
    CHECK(!create.PhaseDeadlineReached(now_ms));

    microros::EntityDestroyCursor destroy;
    destroy.Reset(now_ms);
    std::size_t destroy_slices = 0U;
    while (!destroy.complete()) {
      CHECK(destroy.CurrentRemoteOperationFits(now_ms));
      const microros::EntityDestroyStep step = destroy.current();
      std::size_t boundaries_started_this_slice = 0U;
      ++boundaries_started_this_slice;
      ++destroy_slices;
      now_ms += step.deadline_ms;
      maximum_heartbeat_gap_ms =
          std::max(maximum_heartbeat_gap_ms, now_ms - last_heartbeat_ms);
      last_heartbeat_ms = now_ms;
      CHECK(boundaries_started_this_slice == 1U);
      destroy.Advance();
      ++now_ms;
    }
    CHECK(destroy_slices == microros::kLifecycleDestroyBoundaryCount);
    CHECK(!destroy.RemotePhaseDeadlineReached(now_ms));
  }
  CHECK(maximum_heartbeat_gap_ms <= 41U);

  // A failure at every possible creation boundary stops that slice without
  // advancing into a second boundary. A new generation always restarts from
  // support initialization with no retained index.
  for (std::size_t failure = 1U;
       failure <= microros::kLifecycleCreateBoundaryCount; ++failure) {
    microros::EntityCreateCursor create;
    create.Reset(0U);
    std::size_t calls = 0U;
    while (calls + 1U < failure) {
      ++calls;
      create.Advance();
    }
    ++calls;
    CHECK(calls == failure);
    CHECK(!create.complete());
    const microros::EntityCreateStep failed_step = create.current();
    CHECK(failed_step.kind != microros::EntityCreateStepKind::kComplete);

    create.Clear();
    create.Reset(500U);
    CHECK(create.current().kind ==
          microros::EntityCreateStepKind::kSupportInit);
    CHECK(create.current().index == 0U);
  }

  // Finalizer failures are recorded by the runtime but do not prevent the
  // fixed cursor from reaching every remaining local cleanup operation.
  for (std::size_t failure = 1U;
       failure <= microros::kLifecycleDestroyBoundaryCount; ++failure) {
    microros::EntityDestroyCursor destroy;
    destroy.Reset(0U);
    std::size_t calls = 0U;
    bool observed_failure = false;
    while (!destroy.complete()) {
      ++calls;
      if (calls == failure) {
        observed_failure = true;
      }
      destroy.Advance();
    }
    CHECK(observed_failure);
    CHECK(calls == microros::kLifecycleDestroyBoundaryCount);
  }
}

void TestMiddlewareFaultProxyBoundaries() {
  static_assert(std::is_trivially_copyable_v<microros::MiddlewareFaultProxy>);
  struct BoundaryCase {
    microros::MiddlewareBoundary boundary;
    std::uint32_t deadline_ms;
  };
  constexpr std::array<BoundaryCase, microros::kMiddlewareBoundaryCount + 2U>
      kCases{{
          {microros::MiddlewareBoundary::kExecutorSpin,
           microros::kExecutorWaitMs},
          {microros::MiddlewareBoundary::kRequestTake,
           microros::kNonblockingCallDeadlineMs},
          {microros::MiddlewareBoundary::kBestEffortPublish,
           microros::kNonblockingCallDeadlineMs},
          {microros::MiddlewareBoundary::kReliablePublish,
           microros::kReliableOperationTimeoutMs},
          {microros::MiddlewareBoundary::kSendResponse,
           microros::kReliableOperationTimeoutMs},
          {microros::MiddlewareBoundary::kAgentPing,
           microros::kActivePingTimeoutMs},
          {microros::MiddlewareBoundary::kAgentPing,
           microros::kWaitAgentPingTimeoutMs},
          {microros::MiddlewareBoundary::kTimeSync,
           microros::kActiveTimeSyncTimeoutMs},
          {microros::MiddlewareBoundary::kTimeSync,
           microros::kInitialTimeSyncTimeoutMs},
          {microros::MiddlewareBoundary::kEntityCreate,
           microros::kCreateCallTimeoutMs},
          {microros::MiddlewareBoundary::kEntityFinalize,
           microros::kDestroyCallTimeoutMs},
      }};

  for (const BoundaryCase& test : kCases) {
    microros::MiddlewareFaultProxy proxy;
    const microros::MiddlewareFaultPlan plan{
        test.boundary, 2U, 1U, std::numeric_limits<std::uint32_t>::max(), 73};
    CHECK(proxy.Configure(plan));

    std::uint32_t now_ms = 100U;
    std::uint32_t backend_calls = 0U;
    std::uint32_t waited_ms = 0U;
    const auto invoke = [&]() {
      return proxy.Invoke(
          test.boundary, test.deadline_ms,
          [&backend_calls]() {
            ++backend_calls;
            return 0;
          },
          [&now_ms]() { return now_ms; },
          [&now_ms, &waited_ms](std::uint32_t delay_ms) {
            waited_ms += delay_ms;
            now_ms += delay_ms;
          });
    };

    CHECK(invoke() == 0);
    CHECK(invoke() == 73);
    CHECK(invoke() == 0);
    CHECK(backend_calls == 2U);
    CHECK(waited_ms == test.deadline_ms);
    CHECK(!proxy.fault_enabled());
    const auto stats = proxy.stats(test.boundary);
    CHECK(stats.calls == 3U);
    CHECK(stats.injections == 1U);
    CHECK(stats.deadline_violations == 0U);
    CHECK(stats.max_elapsed_ms == test.deadline_ms);
  }
}

void TestMiddlewareFaultProxySelectionAndDeadline() {
  microros::MiddlewareFaultProxy proxy;
  CHECK(!proxy.Configure(
      {static_cast<microros::MiddlewareBoundary>(255U), 1U, 1U, 0U, 1}));
  CHECK(!proxy.Configure(
      {microros::MiddlewareBoundary::kAgentPing, 0U, 1U, 0U, 1}));
  CHECK(!proxy.Configure(
      {microros::MiddlewareBoundary::kAgentPing, 1U, 0U, 0U, 1}));
  CHECK(!proxy.Configure(
      {microros::MiddlewareBoundary::kAgentPing, 1U, 1U, 0U, 0}));

  CHECK(proxy.Configure({microros::MiddlewareBoundary::kAgentPing, 1U, 3U,
                         microros::kActivePingTimeoutMs, 91}));
  std::uint32_t now_ms = 0U;
  std::uint32_t backend_calls = 0U;
  const auto clock = [&now_ms]() { return now_ms; };
  const auto wait = [&now_ms](std::uint32_t delay_ms) { now_ms += delay_ms; };
  const auto backend = [&backend_calls]() {
    ++backend_calls;
    return 0;
  };

  CHECK(proxy.Invoke(microros::MiddlewareBoundary::kTimeSync,
                     microros::kActiveTimeSyncTimeoutMs, backend, clock,
                     wait) == 0);
  for (std::size_t count = 0U; count < 3U; ++count) {
    CHECK(proxy.Invoke(microros::MiddlewareBoundary::kAgentPing,
                       microros::kActivePingTimeoutMs, backend, clock,
                       wait) == 91);
  }
  CHECK(backend_calls == 1U);
  CHECK(!proxy.fault_enabled());
  CHECK(proxy.stats(microros::MiddlewareBoundary::kAgentPing).injections == 3U);

  proxy.ClearFault();
  proxy.ResetStats();
  now_ms = std::numeric_limits<std::uint32_t>::max() - 1U;
  const std::int32_t result = proxy.Invoke(
      microros::MiddlewareBoundary::kEntityCreate, 2U,
      [&now_ms]() {
        now_ms += 3U;
        return 0;
      },
      clock, wait);
  CHECK(result == 0);
  const auto stats = proxy.stats(microros::MiddlewareBoundary::kEntityCreate);
  CHECK(stats.calls == 1U);
  CHECK(stats.injections == 0U);
  CHECK(stats.deadline_violations == 1U);
  CHECK(stats.max_elapsed_ms == 3U);
  CHECK(proxy.stats(static_cast<microros::MiddlewareBoundary>(255U)).calls ==
        0U);
}

enum class RuntimeHookEventKind : std::uint8_t {
  kSessionActive = 0U,
  kSessionInactive,
  kEmergencyStop,
  kInvalidateSessionWork,
};

struct RuntimeHookEvent {
  RuntimeHookEventKind kind{RuntimeHookEventKind::kEmergencyStop};
  std::uint32_t generation{0U};
};

// Fixed-storage stand-in for the safety and clock hooks used by
// MicroRosRuntime. It deliberately records only the externally observable
// ordering required for a failed session; middleware behavior is supplied by
// the production MiddlewareFaultProxy below.
class FixedRuntimeHooks {
 public:
  void SetSessionActive(bool active, std::uint32_t generation) {
    Record(active ? RuntimeHookEventKind::kSessionActive
                  : RuntimeHookEventKind::kSessionInactive,
           generation);
  }

  void EmergencyStop() { Record(RuntimeHookEventKind::kEmergencyStop, 0U); }

  void InvalidateSessionWork(std::uint32_t generation) {
    Record(RuntimeHookEventKind::kInvalidateSessionWork, generation);
  }

  void Elapse(std::uint32_t milliseconds) { now_ms_ += milliseconds; }

  void WaitAndAdvanceHeartbeat(std::uint32_t milliseconds) {
    Elapse(milliseconds);
    AdvanceHeartbeat();
  }

  void AdvanceHeartbeat() {
    maximum_heartbeat_gap_ms_ =
        std::max(maximum_heartbeat_gap_ms_, now_ms_ - last_heartbeat_ms_);
    last_heartbeat_ms_ = now_ms_;
    ++heartbeat_count_;
  }

  std::uint32_t now_ms() const { return now_ms_; }
  std::uint32_t maximum_heartbeat_gap_ms() const {
    return maximum_heartbeat_gap_ms_;
  }
  std::uint32_t heartbeat_count() const { return heartbeat_count_; }
  std::size_t event_count() const { return event_count_; }
  const RuntimeHookEvent& event(std::size_t index) const {
    CHECK(index < event_count_);
    return events_[index];
  }

 private:
  void Record(RuntimeHookEventKind kind, std::uint32_t generation) {
    CHECK(event_count_ < events_.size());
    events_[event_count_] = {kind, generation};
    ++event_count_;
  }

  std::array<RuntimeHookEvent, 8U> events_{};
  std::uint32_t now_ms_{0U};
  std::uint32_t last_heartbeat_ms_{0U};
  std::uint32_t maximum_heartbeat_gap_ms_{0U};
  std::uint32_t heartbeat_count_{0U};
  std::size_t event_count_{0U};
};

microros::MiddlewareBoundary BoundaryForCreateStep(
    microros::EntityCreateStepKind kind) {
  switch (kind) {
    case microros::EntityCreateStepKind::kExecutorPrime:
      return microros::MiddlewareBoundary::kExecutorSpin;
    case microros::EntityCreateStepKind::kInitialTimeSync:
      return microros::MiddlewareBoundary::kTimeSync;
    default:
      return microros::MiddlewareBoundary::kEntityCreate;
  }
}

void TestExecutorFaultBoundedTeardownAndReconnect() {
  static_assert(std::is_trivially_copyable_v<FixedRuntimeHooks>);
  microros::SessionLifecycle lifecycle;
  FixedRuntimeHooks hooks;
  microros::MiddlewareFaultProxy proxy;

  lifecycle.LeaveSafeBoot();
  lifecycle.OnAgentPing(true, hooks.now_ms());
  lifecycle.OnEntitiesCreated(true, hooks.now_ms());
  CHECK(lifecycle.state() == microros::SessionState::kActive);
  CHECK(lifecycle.session_generation() == 1U);
  hooks.SetSessionActive(true, lifecycle.session_generation());

  // Withholding the executor result consumes no more than its declared 1 ms
  // boundary. The backend is not entered and the runtime's fatal-result path
  // immediately makes generation 1 inactive, stops motors, and invalidates
  // all pending work before entity finalization begins.
  constexpr std::int32_t kInjectedFailure = 73;
  CHECK(proxy.Configure({microros::MiddlewareBoundary::kExecutorSpin, 1U, 1U,
                         microros::kExecutorWaitMs, kInjectedFailure}));
  std::uint32_t executor_backend_calls = 0U;
  const std::int32_t spin_result = proxy.Invoke(
      microros::MiddlewareBoundary::kExecutorSpin, microros::kExecutorWaitMs,
      [&executor_backend_calls]() {
        ++executor_backend_calls;
        return 0;
      },
      [&hooks]() { return hooks.now_ms(); },
      [&hooks](std::uint32_t wait_ms) {
        hooks.WaitAndAdvanceHeartbeat(wait_ms);
      });
  CHECK(spin_result == kInjectedFailure);
  CHECK(executor_backend_calls == 0U);
  CHECK(hooks.now_ms() == microros::kExecutorWaitMs);
  CHECK(proxy.stats(microros::MiddlewareBoundary::kExecutorSpin).calls == 1U);
  CHECK(proxy.stats(microros::MiddlewareBoundary::kExecutorSpin).injections ==
        1U);

  hooks.SetSessionActive(false, lifecycle.session_generation());
  hooks.EmergencyStop();
  hooks.InvalidateSessionWork(lifecycle.session_generation());
  lifecycle.BeginTeardown(microros::TeardownReason::kEntityError);
  CHECK(lifecycle.state() == microros::SessionState::kTeardown);

  // PrepareDestroyEntities repeats the fail-closed hooks. Then make every
  // remote finalizer unavailable. Each call still returns after exactly its
  // 10 ms budget, every remaining cleanup boundary is visited, and the whole
  // remote phase remains well below 500 ms.
  hooks.SetSessionActive(false, lifecycle.session_generation());
  hooks.EmergencyStop();
  hooks.InvalidateSessionWork(lifecycle.session_generation());
  const std::uint32_t destroy_start_ms = hooks.now_ms();
  microros::EntityDestroyCursor destroy;
  destroy.Reset(destroy_start_ms);
  CHECK(proxy.Configure(
      {microros::MiddlewareBoundary::kEntityFinalize, 1U,
       static_cast<std::uint32_t>(microros::kLifecycleDestroyBoundaryCount),
       microros::kDestroyCallTimeoutMs, kInjectedFailure}));
  std::uint32_t finalizer_backend_calls = 0U;
  std::size_t finalizer_count = 0U;
  while (!destroy.complete()) {
    CHECK(destroy.CurrentRemoteOperationFits(hooks.now_ms()));
    const microros::EntityDestroyStep step = destroy.current();
    CHECK(step.deadline_ms == microros::kDestroyCallTimeoutMs);
    const std::int32_t result = proxy.Invoke(
        microros::MiddlewareBoundary::kEntityFinalize, step.deadline_ms,
        [&finalizer_backend_calls]() {
          ++finalizer_backend_calls;
          return 0;
        },
        [&hooks]() { return hooks.now_ms(); },
        [&hooks](std::uint32_t wait_ms) {
          hooks.WaitAndAdvanceHeartbeat(wait_ms);
        });
    CHECK(result == kInjectedFailure);
    ++finalizer_count;
    destroy.Advance();
    hooks.AdvanceHeartbeat();
    if (!destroy.complete()) {
      hooks.Elapse(1U);
    }
  }
  CHECK(finalizer_count == microros::kLifecycleDestroyBoundaryCount);
  CHECK(finalizer_backend_calls == 0U);
  CHECK(hooks.now_ms() - destroy_start_ms ==
        microros::kLifecycleDestroyMaximumDeclaredMilliseconds);
  CHECK(!destroy.RemotePhaseDeadlineReached(hooks.now_ms()));
  const auto finalize_stats =
      proxy.stats(microros::MiddlewareBoundary::kEntityFinalize);
  CHECK(finalize_stats.calls == microros::kLifecycleDestroyBoundaryCount);
  CHECK(finalize_stats.injections == microros::kLifecycleDestroyBoundaryCount);
  CHECK(finalize_stats.deadline_violations == 0U);
  CHECK(finalize_stats.max_elapsed_ms == microros::kDestroyCallTimeoutMs);

  lifecycle.OnTeardownComplete(hooks.now_ms());
  CHECK(lifecycle.state() == microros::SessionState::kBackoff);
  CHECK(lifecycle.current_backoff_ms() == 100U);
  while (lifecycle.state() == microros::SessionState::kBackoff) {
    hooks.Elapse(10U);
    static_cast<void>(lifecycle.AdvanceBackoff(hooks.now_ms()));
    hooks.AdvanceHeartbeat();
  }
  CHECK(lifecycle.state() == microros::SessionState::kWaitAgent);

  // A healthy Agent then recreates the exact fixed graph. The fault plan was
  // exhausted, no failed-session operation is replayed, and activation uses a
  // strictly newer nonzero generation.
  lifecycle.OnAgentPing(true, hooks.now_ms());
  microros::EntityCreateCursor create;
  create.Reset(hooks.now_ms());
  std::size_t create_count = 0U;
  std::uint32_t create_backend_calls = 0U;
  while (!create.complete()) {
    CHECK(create.CurrentOperationFits(hooks.now_ms()));
    const microros::EntityCreateStep step = create.current();
    const std::int32_t result = proxy.Invoke(
        BoundaryForCreateStep(step.kind), step.deadline_ms,
        [&create_backend_calls]() {
          ++create_backend_calls;
          return 0;
        },
        [&hooks]() { return hooks.now_ms(); },
        [&hooks](std::uint32_t wait_ms) {
          hooks.WaitAndAdvanceHeartbeat(wait_ms);
        });
    CHECK(result == 0);
    ++create_count;
    create.Advance();
    hooks.AdvanceHeartbeat();
    if (!create.complete()) {
      hooks.Elapse(1U);
    }
  }
  CHECK(create_count == microros::kLifecycleCreateBoundaryCount);
  CHECK(create_backend_calls == microros::kLifecycleCreateBoundaryCount);
  CHECK(!create.PhaseDeadlineReached(hooks.now_ms()));
  lifecycle.OnEntitiesCreated(true, hooks.now_ms());
  hooks.SetSessionActive(true, lifecycle.session_generation());
  CHECK(lifecycle.state() == microros::SessionState::kActive);
  CHECK(lifecycle.session_generation() == 2U);
  CHECK(lifecycle.agent_reconnects() == 1U);
  CHECK(hooks.maximum_heartbeat_gap_ms() <= 11U);
  CHECK(hooks.heartbeat_count() > 0U);

  constexpr std::array<RuntimeHookEvent, 8U> kExpectedEvents{{
      {RuntimeHookEventKind::kSessionActive, 1U},
      {RuntimeHookEventKind::kSessionInactive, 1U},
      {RuntimeHookEventKind::kEmergencyStop, 0U},
      {RuntimeHookEventKind::kInvalidateSessionWork, 1U},
      {RuntimeHookEventKind::kSessionInactive, 1U},
      {RuntimeHookEventKind::kEmergencyStop, 0U},
      {RuntimeHookEventKind::kInvalidateSessionWork, 1U},
      {RuntimeHookEventKind::kSessionActive, 2U},
  }};
  CHECK(hooks.event_count() == kExpectedEvents.size());
  for (std::size_t index = 0U; index < kExpectedEvents.size(); ++index) {
    CHECK(hooks.event(index).kind == kExpectedEvents[index].kind);
    CHECK(hooks.event(index).generation == kExpectedEvents[index].generation);
  }
}

}  // namespace

int main() {
  TestBackoffAndSessions();
  TestFailureAndWrapHelpers();
  TestOutOfOrderLifecycleEventsAreIgnored();
  TestTransportFaultClassification();
  TestEndpointInventoryAndQos();
  TestRoundRobinCursor();
  TestActiveSliceBudget();
  TestActiveWorkScheduler();
  TestEntityCreateCursorOrder();
  TestEntityDestroyCursorOrderAndAbandon();
  TestLifecycleDeadlineAndWrap();
  TestIncrementalLifecycleDriverAndReconnects();
  TestMiddlewareFaultProxyBoundaries();
  TestMiddlewareFaultProxySelectionAndDeadline();
  TestExecutorFaultBoundedTeardownAndReconnect();
  std::cout << "runtime core tests passed\n";
  return EXIT_SUCCESS;
}
