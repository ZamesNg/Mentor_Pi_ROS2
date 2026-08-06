// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "mentor_pi_mcu/app/microros/runtime_core.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mentor_pi_mcu::app::microros {
namespace {

constexpr std::array<std::uint32_t, 6> kBackoffMilliseconds{100U, 200U,  400U,
                                                            800U, 1600U, 2000U};

constexpr std::uint32_t CreateStepDeadlineMs(EntityCreateStepKind kind) {
  switch (kind) {
    case EntityCreateStepKind::kExecutorPrime:
      return kExecutorWaitMs;
    case EntityCreateStepKind::kInitialTimeSync:
      return kInitialTimeSyncTimeoutMs;
    case EntityCreateStepKind::kComplete:
      return 0U;
    default:
      return kCreateCallTimeoutMs;
  }
}

}  // namespace

void EntityCreateCursor::Reset(std::uint32_t phase_start_ms) {
  kind_ = EntityCreateStepKind::kSupportInit;
  index_ = 0U;
  phase_start_ms_ = phase_start_ms;
  started_ = true;
}

void EntityCreateCursor::Clear() {
  kind_ = EntityCreateStepKind::kComplete;
  index_ = 0U;
  phase_start_ms_ = 0U;
  started_ = false;
}

EntityCreateStep EntityCreateCursor::current() const {
  if (!started_) {
    return {};
  }
  return {kind_, index_, CreateStepDeadlineMs(kind_)};
}

void EntityCreateCursor::Advance() {
  if (!started_) {
    return;
  }
  switch (kind_) {
    case EntityCreateStepKind::kSupportInit:
      kind_ = EntityCreateStepKind::kSetContextCreateTimeout;
      break;
    case EntityCreateStepKind::kSetContextCreateTimeout:
      kind_ = EntityCreateStepKind::kSetContextDestroyTimeout;
      break;
    case EntityCreateStepKind::kSetContextDestroyTimeout:
      kind_ = EntityCreateStepKind::kNodeInit;
      break;
    case EntityCreateStepKind::kNodeInit:
      kind_ = EntityCreateStepKind::kPublisherInit;
      index_ = 0U;
      break;
    case EntityCreateStepKind::kPublisherInit:
      kind_ = EntityCreateStepKind::kSetPublisherTimeout;
      break;
    case EntityCreateStepKind::kSetPublisherTimeout:
      ++index_;
      if (index_ < kLifecyclePublisherCount) {
        kind_ = EntityCreateStepKind::kPublisherInit;
      } else {
        kind_ = EntityCreateStepKind::kSubscriptionInit;
        index_ = 0U;
      }
      break;
    case EntityCreateStepKind::kSubscriptionInit:
      ++index_;
      if (index_ >= kLifecycleSubscriptionCount) {
        kind_ = EntityCreateStepKind::kServiceInit;
        index_ = 0U;
      }
      break;
    case EntityCreateStepKind::kServiceInit:
      kind_ = EntityCreateStepKind::kSetServiceTimeout;
      break;
    case EntityCreateStepKind::kSetServiceTimeout:
      ++index_;
      if (index_ < kLifecycleServiceCount) {
        kind_ = EntityCreateStepKind::kServiceInit;
      } else {
        kind_ = EntityCreateStepKind::kExecutorInit;
        index_ = 0U;
      }
      break;
    case EntityCreateStepKind::kExecutorInit:
      kind_ = EntityCreateStepKind::kExecutorAddSubscription;
      index_ = 0U;
      break;
    case EntityCreateStepKind::kExecutorAddSubscription:
      ++index_;
      if (index_ >= kLifecycleSubscriptionCount) {
        kind_ = EntityCreateStepKind::kExecutorPrime;
        index_ = 0U;
      }
      break;
    case EntityCreateStepKind::kExecutorPrime:
      kind_ = EntityCreateStepKind::kInitialTimeSync;
      break;
    case EntityCreateStepKind::kInitialTimeSync:
      kind_ = EntityCreateStepKind::kComplete;
      break;
    case EntityCreateStepKind::kComplete:
      break;
  }
}

bool EntityCreateCursor::CurrentOperationFits(std::uint32_t now_ms) const {
  const EntityCreateStep step = current();
  return !complete() &&
         OperationFitsInPhase(phase_start_ms_, now_ms, kCreatePhaseDeadlineMs,
                              step.deadline_ms);
}

bool EntityCreateCursor::PhaseDeadlineReached(std::uint32_t now_ms) const {
  return ::mentor_pi_mcu::app::microros::PhaseDeadlineReached(
      phase_start_ms_, now_ms, kCreatePhaseDeadlineMs);
}

void EntityDestroyCursor::Reset(std::uint32_t phase_start_ms) {
  kind_ = EntityDestroyStepKind::kSetContextDestroyTimeout;
  index_ = 0U;
  phase_start_ms_ = phase_start_ms;
  started_ = true;
  remote_waits_abandoned_ = false;
  disable_remote_waits_pending_ = false;
}

void EntityDestroyCursor::Clear() {
  kind_ = EntityDestroyStepKind::kComplete;
  index_ = 0U;
  phase_start_ms_ = 0U;
  started_ = false;
  remote_waits_abandoned_ = false;
  disable_remote_waits_pending_ = false;
}

EntityDestroyStep EntityDestroyCursor::current() const {
  if (!started_) {
    return {};
  }
  if (disable_remote_waits_pending_) {
    // This setter is local context policy, not another remote wait. Giving the
    // fault proxy no withholding budget prevents the abandon operation itself
    // from carrying the remote phase beyond 500 ms.
    return {EntityDestroyStepKind::kDisableRemoteWaits, 0U, 0U};
  }
  const std::uint32_t deadline_ms =
      kind_ == EntityDestroyStepKind::kComplete ? 0U : kDestroyCallTimeoutMs;
  return {kind_, index_, deadline_ms};
}

void EntityDestroyCursor::Advance() {
  if (!started_) {
    return;
  }
  if (disable_remote_waits_pending_) {
    disable_remote_waits_pending_ = false;
    remote_waits_abandoned_ = true;
    return;
  }
  switch (kind_) {
    case EntityDestroyStepKind::kSetContextDestroyTimeout:
      kind_ = EntityDestroyStepKind::kExecutorFini;
      break;
    case EntityDestroyStepKind::kExecutorFini:
      kind_ = EntityDestroyStepKind::kServiceFini;
      index_ = kLifecycleServiceCount - 1U;
      break;
    case EntityDestroyStepKind::kServiceFini:
      if (index_ != 0U) {
        --index_;
      } else {
        kind_ = EntityDestroyStepKind::kSubscriptionFini;
        index_ = kLifecycleSubscriptionCount - 1U;
      }
      break;
    case EntityDestroyStepKind::kSubscriptionFini:
      if (index_ != 0U) {
        --index_;
      } else {
        kind_ = EntityDestroyStepKind::kPublisherFini;
        index_ = kLifecyclePublisherCount - 1U;
      }
      break;
    case EntityDestroyStepKind::kPublisherFini:
      if (index_ != 0U) {
        --index_;
      } else {
        kind_ = EntityDestroyStepKind::kNodeFini;
        index_ = 0U;
      }
      break;
    case EntityDestroyStepKind::kNodeFini:
      kind_ = EntityDestroyStepKind::kSupportFini;
      break;
    case EntityDestroyStepKind::kSupportFini:
      kind_ = EntityDestroyStepKind::kComplete;
      break;
    case EntityDestroyStepKind::kDisableRemoteWaits:
    case EntityDestroyStepKind::kComplete:
      break;
  }
}

void EntityDestroyCursor::RequestRemoteWaitAbandon() {
  if (!started_ || remote_waits_abandoned_ || disable_remote_waits_pending_ ||
      complete()) {
    return;
  }
  SkipInitialTimeoutIfPending();
  disable_remote_waits_pending_ = true;
}

void EntityDestroyCursor::MarkRemoteWaitsAbandonedWithoutBoundary() {
  if (!started_ || remote_waits_abandoned_ || complete()) {
    return;
  }
  SkipInitialTimeoutIfPending();
  disable_remote_waits_pending_ = false;
  remote_waits_abandoned_ = true;
}

bool EntityDestroyCursor::CurrentRemoteOperationFits(
    std::uint32_t now_ms) const {
  const EntityDestroyStep step = current();
  return !complete() && !remote_waits_abandoned_ &&
         !disable_remote_waits_pending_ &&
         OperationFitsInPhase(phase_start_ms_, now_ms, kDestroyPhaseDeadlineMs,
                              step.deadline_ms);
}

bool EntityDestroyCursor::RemotePhaseDeadlineReached(
    std::uint32_t now_ms) const {
  return ::mentor_pi_mcu::app::microros::PhaseDeadlineReached(
      phase_start_ms_, now_ms, kDestroyPhaseDeadlineMs);
}

void EntityDestroyCursor::SkipInitialTimeoutIfPending() {
  if (kind_ == EntityDestroyStepKind::kSetContextDestroyTimeout) {
    kind_ = EntityDestroyStepKind::kExecutorFini;
  }
}

TeardownReason ClassifyTransportErrorFlags(std::uint8_t error_flags) {
  if ((error_flags & kTransportRxRingOverrunError) != 0U) {
    return TeardownReason::kRxOverrun;
  }
  if ((error_flags & (kTransportTxDmaError | kTransportTxTimeoutError)) != 0U) {
    return TeardownReason::kTxTimeout;
  }
  if ((error_flags &
       (kTransportFramingError | kTransportNoiseError | kTransportOverrunError |
        kTransportParityError | kTransportDmaError)) != 0U) {
    return TeardownReason::kUsart1Error;
  }
  return TeardownReason::kNone;
}

void SessionLifecycle::LeaveSafeBoot() {
  if (state_ != SessionState::kSafeBoot) {
    return;
  }
  teardown_reason_ = TeardownReason::kNone;
  state_ = SessionState::kWaitAgent;
}

void SessionLifecycle::OnAgentPing(bool successful, std::uint32_t now_ms) {
  if (state_ != SessionState::kWaitAgent) {
    return;
  }
  if (successful) {
    state_ = SessionState::kCreateEntities;
    return;
  }
  EnterBackoff(now_ms);
}

void SessionLifecycle::OnEntitiesCreated(bool successful,
                                         std::uint32_t now_ms) {
  static_cast<void>(now_ms);
  if (state_ != SessionState::kCreateEntities) {
    return;
  }
  if (!successful) {
    teardown_reason_ = TeardownReason::kEntityError;
    state_ = SessionState::kTeardown;
    return;
  }

  if (session_generation_ == 0U) {
    session_generation_ = 1U;
  } else {
    session_generation_ = NextNonzeroGeneration(session_generation_);
    if (agent_reconnects_ != std::numeric_limits<std::uint32_t>::max()) {
      ++agent_reconnects_;
    }
  }
  backoff_index_ = 0U;
  current_backoff_ms_ = 0U;
  state_ = SessionState::kActive;
}

void SessionLifecycle::BeginTeardown(TeardownReason reason) {
  if (state_ == SessionState::kTeardown) {
    return;
  }
  teardown_reason_ = reason;
  state_ = SessionState::kTeardown;
}

void SessionLifecycle::OnTeardownComplete(std::uint32_t now_ms) {
  if (state_ != SessionState::kTeardown) {
    return;
  }
  EnterBackoff(now_ms);
}

bool SessionLifecycle::AdvanceBackoff(std::uint32_t now_ms) {
  if (state_ != SessionState::kBackoff ||
      !DeadlineReached(now_ms, backoff_deadline_ms_)) {
    return false;
  }
  state_ = SessionState::kWaitAgent;
  return true;
}

void SessionLifecycle::EnterBackoff(std::uint32_t now_ms) {
  const std::size_t index = backoff_index_ < kBackoffMilliseconds.size()
                                ? backoff_index_
                                : kBackoffMilliseconds.size() - 1U;
  current_backoff_ms_ = kBackoffMilliseconds[index];
  backoff_deadline_ms_ = now_ms + current_backoff_ms_;
  if (backoff_index_ + 1U < kBackoffMilliseconds.size()) {
    ++backoff_index_;
  }
  state_ = SessionState::kBackoff;
}

}  // namespace mentor_pi_mcu::app::microros
