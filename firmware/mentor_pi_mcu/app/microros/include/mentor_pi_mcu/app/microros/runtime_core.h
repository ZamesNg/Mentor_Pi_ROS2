// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_RUNTIME_CORE_H_
#define MENTOR_PI_MCU_APP_MICROROS_RUNTIME_CORE_H_

#include <cstddef>
#include <cstdint>

namespace mentor_pi_mcu::app::microros {

enum class SessionState : std::uint8_t {
  kSafeBoot = 0,
  kWaitAgent = 1,
  kCreateEntities = 2,
  kActive = 3,
  kTeardown = 4,
  kBackoff = 5,
};

enum class TeardownReason : std::uint8_t {
  kNone = 0,
  kAgentLost = 1,
  kUsart1Error = 2,
  kRxOverrun = 3,
  kTxTimeout = 4,
  kEntityError = 5,
  kMemoryViolation = 6,
  kTaskStall = 7,
};

enum class HeartbeatState : std::uint8_t {
  kBooting = 0,
  kReady = 1,
  kDegraded = 2,
  kFault = 3,
};

enum class ErrorSource : std::uint8_t {
  kNone = 0,
  kTransport = 1,
  kMotors = 2,
  kPwmServos = 3,
  kBusServos = 4,
  kLeds = 5,
  kBuzzer = 6,
  kRgb = 7,
  kOled = 8,
  kImu = 9,
  kBattery = 10,
  kExecutor = 11,
  kMemory = 12,
};

enum class ActiveWorkClass : std::uint8_t {
  kService = 0,
  kReliableTelemetry = 1,
  kMaintenance = 2,
};
inline constexpr std::size_t kActiveWorkClassCount = 3U;
static_assert(static_cast<std::size_t>(ActiveWorkClass::kMaintenance) + 1U ==
              kActiveWorkClassCount);

inline constexpr std::uint32_t kWaitAgentPingTimeoutMs = 20U;
inline constexpr std::uint32_t kActivePingPeriodMs = 500U;
inline constexpr std::uint32_t kActivePingTimeoutMs = 10U;
inline constexpr std::uint8_t kActivePingFailureLimit = 3U;
inline constexpr std::uint32_t kCreateCallTimeoutMs = 40U;
inline constexpr std::uint32_t kCreatePhaseDeadlineMs = 2000U;
inline constexpr std::uint32_t kDestroyCallTimeoutMs = 10U;
inline constexpr std::uint32_t kDestroyPhaseDeadlineMs = 500U;
inline constexpr std::uint32_t kExecutorWaitMs = 1U;
inline constexpr std::uint32_t kNonblockingCallDeadlineMs = 1U;
inline constexpr std::uint32_t kReliableOperationTimeoutMs = 10U;
inline constexpr std::uint32_t kInitialTimeSyncTimeoutMs = 20U;
inline constexpr std::uint32_t kActiveTimeSyncTimeoutMs = 10U;
inline constexpr std::uint32_t kTimeSyncRetryMs = 5000U;
inline constexpr std::uint32_t kTimeResyncPeriodMs = 60000U;

// The v2 graph inventory is fixed. These lifecycle constants intentionally live
// in the portable core so the production create/finalize scheduler and its
// native tests use the same exact operation counts.
inline constexpr std::size_t kLifecyclePublisherCount = 7U;
inline constexpr std::size_t kLifecycleSubscriptionCount = 7U;
inline constexpr std::size_t kLifecycleServiceCount = 6U;
inline constexpr std::size_t kLifecycleCreateBoundaryCount = 47U;
inline constexpr std::size_t kLifecycleDestroyBoundaryCount = 24U;
inline constexpr std::size_t kLifecycleCreateMaximumDeclaredMilliseconds =
    ((kLifecycleCreateBoundaryCount - 2U) * kCreateCallTimeoutMs) +
    kExecutorWaitMs + kInitialTimeSyncTimeoutMs +
    (kLifecycleCreateBoundaryCount - 1U);
inline constexpr std::size_t kLifecycleDestroyMaximumDeclaredMilliseconds =
    (kLifecycleDestroyBoundaryCount * kDestroyCallTimeoutMs) +
    (kLifecycleDestroyBoundaryCount - 1U);
static_assert(kLifecycleCreateMaximumDeclaredMilliseconds <
              kCreatePhaseDeadlineMs);
static_assert(kLifecycleDestroyMaximumDeclaredMilliseconds <
              kDestroyPhaseDeadlineMs);

enum class EntityCreateStepKind : std::uint8_t {
  kSupportInit = 0,
  kSetContextCreateTimeout,
  kSetContextDestroyTimeout,
  kNodeInit,
  kPublisherInit,
  kSetPublisherTimeout,
  kSubscriptionInit,
  kServiceInit,
  kSetServiceTimeout,
  kExecutorInit,
  kExecutorAddSubscription,
  kExecutorPrime,
  kInitialTimeSync,
  kComplete,
};

enum class EntityDestroyStepKind : std::uint8_t {
  kSetContextDestroyTimeout = 0,
  kExecutorFini,
  kServiceFini,
  kSubscriptionFini,
  kPublisherFini,
  kNodeFini,
  kSupportFini,
  kDisableRemoteWaits,
  kComplete,
};

struct EntityCreateStep {
  EntityCreateStepKind kind{EntityCreateStepKind::kComplete};
  std::size_t index{0U};
  std::uint32_t deadline_ms{0U};
};

struct EntityDestroyStep {
  EntityDestroyStepKind kind{EntityDestroyStepKind::kComplete};
  std::size_t index{0U};
  std::uint32_t deadline_ms{0U};
};

// Returns true only when an operation can consume its complete declared budget
// before the phase deadline. Reaching the deadline is the existing failure
// boundary. Unsigned subtraction deliberately keeps this correct across the
// uint32 millisecond wrap.
constexpr bool OperationFitsInPhase(std::uint32_t phase_start_ms,
                                    std::uint32_t now_ms,
                                    std::uint32_t phase_budget_ms,
                                    std::uint32_t operation_budget_ms) {
  const std::uint32_t elapsed_ms = now_ms - phase_start_ms;
  return elapsed_ms < phase_budget_ms &&
         operation_budget_ms < phase_budget_ms - elapsed_ms;
}

constexpr bool PhaseDeadlineReached(std::uint32_t phase_start_ms,
                                    std::uint32_t now_ms,
                                    std::uint32_t phase_budget_ms) {
  return now_ms - phase_start_ms >= phase_budget_ms;
}

// Enumerates exactly one project-owned middleware boundary at a time. Local
// preparation and the final allocation seal are owned by MicroRosRuntime and do
// not appear as boundary steps.
class EntityCreateCursor {
 public:
  void Reset(std::uint32_t phase_start_ms);
  void Clear();
  EntityCreateStep current() const;
  void Advance();

  bool started() const { return started_; }
  bool complete() const {
    return started_ && kind_ == EntityCreateStepKind::kComplete;
  }
  std::uint32_t phase_start_ms() const { return phase_start_ms_; }
  bool CurrentOperationFits(std::uint32_t now_ms) const;
  bool PhaseDeadlineReached(std::uint32_t now_ms) const;

 private:
  EntityCreateStepKind kind_{EntityCreateStepKind::kComplete};
  std::size_t index_{0U};
  std::uint32_t phase_start_ms_{0U};
  bool started_{false};
};

// Enumerates reverse-order finalization. If the 500 ms remote phase can no
// longer fit another 10 ms call, RequestRemoteWaitAbandon() inserts one
// timeout- zero operation without consuming the pending finalizer. Subsequent
// finalizers are still individually bounded but are local cleanup rather than
// remote wait.
class EntityDestroyCursor {
 public:
  void Reset(std::uint32_t phase_start_ms);
  void Clear();
  EntityDestroyStep current() const;
  void Advance();
  void RequestRemoteWaitAbandon();
  void MarkRemoteWaitsAbandonedWithoutBoundary();

  bool started() const { return started_; }
  bool complete() const {
    return started_ && kind_ == EntityDestroyStepKind::kComplete &&
           !disable_remote_waits_pending_;
  }
  bool remote_waits_abandoned() const { return remote_waits_abandoned_; }
  std::uint32_t phase_start_ms() const { return phase_start_ms_; }
  bool CurrentRemoteOperationFits(std::uint32_t now_ms) const;
  bool RemotePhaseDeadlineReached(std::uint32_t now_ms) const;

 private:
  void SkipInitialTimeoutIfPending();

  EntityDestroyStepKind kind_{EntityDestroyStepKind::kComplete};
  std::size_t index_{0U};
  std::uint32_t phase_start_ms_{0U};
  bool started_{false};
  bool remote_waits_abandoned_{false};
  bool disable_remote_waits_pending_{false};
};

// These values are the stable wire/diagnostic bit layout of Usart1Error. The
// STM32 adapter has static assertions against its platform enum.
inline constexpr std::uint8_t kTransportFramingError = 1U << 0;
inline constexpr std::uint8_t kTransportNoiseError = 1U << 1;
inline constexpr std::uint8_t kTransportOverrunError = 1U << 2;
inline constexpr std::uint8_t kTransportParityError = 1U << 3;
inline constexpr std::uint8_t kTransportTxDmaError = 1U << 4;
inline constexpr std::uint8_t kTransportTxTimeoutError = 1U << 5;
inline constexpr std::uint8_t kTransportRxRingOverrunError = 1U << 6;
inline constexpr std::uint8_t kTransportDmaError = 1U << 7;

TeardownReason ClassifyTransportErrorFlags(std::uint8_t error_flags);

// All lifecycle intervals are much shorter than 2^31 milliseconds. This
// comparison therefore remains correct across the uint32 monotonic wrap.
constexpr bool DeadlineReached(std::uint32_t now_ms,
                               std::uint32_t deadline_ms) {
  return static_cast<std::int32_t>(now_ms - deadline_ms) >= 0;
}

constexpr std::uint32_t NextNonzeroGeneration(std::uint32_t generation) {
  return generation == UINT32_MAX ? 1U : generation + 1U;
}

// Fixed-size round-robin state used by the ACTIVE work-slice schedulers. The
// template keeps the storage to one index and makes an invalid zero-sized
// scheduler a compile-time error.
template <std::size_t kItemCount>
class RoundRobinCursor {
 public:
  static_assert(kItemCount > 0U);

  constexpr std::size_t Peek(std::size_t offset = 0U) const {
    return (next_ + offset) % kItemCount;
  }

  constexpr void AdvancePast(std::size_t item) {
    next_ = (item + 1U) % kItemCount;
  }

  constexpr void Reset() { next_ = 0U; }

 private:
  std::size_t next_{0U};
};

class ActiveWorkScheduler {
 public:
  ActiveWorkClass Next() {
    const std::size_t next = cursor_.Peek();
    cursor_.AdvancePast(next);
    return static_cast<ActiveWorkClass>(next);
  }

  void Reset() { cursor_.Reset(); }

 private:
  RoundRobinCursor<kActiveWorkClassCount> cursor_{};
};

class ActiveSliceBudget {
 public:
  void Reset(ActiveWorkClass allowed_class) {
    allowed_class_ = allowed_class;
    blocking_operation_started_ = false;
  }

  bool TryStartBlockingOperation(ActiveWorkClass requesting_class) {
    if (requesting_class != allowed_class_ || blocking_operation_started_) {
      return false;
    }
    blocking_operation_started_ = true;
    return true;
  }

  bool blocking_operation_started() const {
    return blocking_operation_started_;
  }

 private:
  ActiveWorkClass allowed_class_{ActiveWorkClass::kService};
  bool blocking_operation_started_{false};
};

class SessionLifecycle {
 public:
  void LeaveSafeBoot();
  void OnAgentPing(bool successful, std::uint32_t now_ms);
  void OnEntitiesCreated(bool successful, std::uint32_t now_ms);
  void BeginTeardown(TeardownReason reason);
  void OnTeardownComplete(std::uint32_t now_ms);
  bool AdvanceBackoff(std::uint32_t now_ms);

  SessionState state() const { return state_; }
  TeardownReason teardown_reason() const { return teardown_reason_; }
  std::uint32_t session_generation() const { return session_generation_; }
  std::uint32_t agent_reconnects() const { return agent_reconnects_; }
  std::uint32_t backoff_deadline_ms() const { return backoff_deadline_ms_; }
  std::uint32_t current_backoff_ms() const { return current_backoff_ms_; }

 private:
  void EnterBackoff(std::uint32_t now_ms);

  SessionState state_{SessionState::kSafeBoot};
  TeardownReason teardown_reason_{TeardownReason::kNone};
  std::uint32_t session_generation_{0U};
  std::uint32_t agent_reconnects_{0U};
  std::uint32_t backoff_deadline_ms_{0U};
  std::uint32_t current_backoff_ms_{0U};
  std::uint8_t backoff_index_{0U};
};

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_RUNTIME_CORE_H_
