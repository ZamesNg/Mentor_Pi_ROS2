// Copyright 2026 Mentor Pi contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef MENTOR_PI_MCU_APP_MICROROS_MIDDLEWARE_FAULT_PROXY_H_
#define MENTOR_PI_MCU_APP_MICROROS_MIDDLEWARE_FAULT_PROXY_H_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mentor_pi_mcu::app::microros {

// Every potentially waiting rcl/rmw boundary owned by MicroRosTask belongs to
// exactly one of these classes. Handle accessors, zero initializers, and epoch
// reads are local data access and intentionally are not middleware calls.
enum class MiddlewareBoundary : std::uint8_t {
  kExecutorSpin = 0U,
  kRequestTake,
  kBestEffortPublish,
  kReliablePublish,
  kSendResponse,
  kAgentPing,
  kTimeSync,
  kEntityCreate,
  kEntityFinalize,
  kCount,
};

inline constexpr std::size_t kMiddlewareBoundaryCount =
    static_cast<std::size_t>(MiddlewareBoundary::kCount);

// This value is deliberately outside the rcl/rmw success and timeout values.
// Fault builds may inject it so call sites follow their existing fatal-error
// path. Normal calls always return the backend result unchanged; an elapsed
// deadline violation is recorded for qualification without changing runtime
// behavior.
inline constexpr std::int32_t kMiddlewareDeadlineExceededResult = -1;

struct MiddlewareFaultPlan {
  MiddlewareBoundary boundary{MiddlewareBoundary::kExecutorSpin};
  std::uint32_t first_occurrence{1U};
  std::uint32_t occurrence_count{1U};
  std::uint32_t withhold_ms{0U};
  std::int32_t injected_result{kMiddlewareDeadlineExceededResult};
};

struct MiddlewareBoundaryStats {
  std::uint32_t calls{0U};
  std::uint32_t injections{0U};
  std::uint32_t deadline_violations{0U};
  std::uint32_t max_elapsed_ms{0U};
};

// A deterministic, allocation-free proxy around project-owned middleware
// calls. In production no plan is configured, so Invoke() calls the supplied
// backend exactly once. Fault builds may skip selected backend calls, wait no
// longer than the call's declared deadline, and return an injected failure.
// Clock and wait operations are passed as templates to avoid std::function and
// all dynamic storage.
class MiddlewareFaultProxy {
 public:
  bool Configure(const MiddlewareFaultPlan& plan) {
    if (!IsValidBoundary(plan.boundary) || plan.first_occurrence == 0U ||
        plan.occurrence_count == 0U || plan.injected_result == 0) {
      return false;
    }
    plan_ = plan;
    matching_occurrences_ = 0U;
    remaining_injections_ = plan.occurrence_count;
    fault_enabled_ = true;
    return true;
  }

  void ClearFault() {
    fault_enabled_ = false;
    matching_occurrences_ = 0U;
    remaining_injections_ = 0U;
  }

  void ResetStats() { stats_ = {}; }

  template <typename Operation, typename Clock, typename Wait>
  std::int32_t Invoke(MiddlewareBoundary boundary, std::uint32_t deadline_ms,
                      Operation operation, Clock clock, Wait wait) {
    const std::size_t index = ToIndex(boundary);
    if (index >= stats_.size()) {
      return kMiddlewareDeadlineExceededResult;
    }

    auto& stats = stats_[index];
    SaturatingIncrement(&stats.calls);
    const std::uint32_t start_ms = clock();
    std::int32_t result = 0;
    if (ShouldInject(boundary)) {
      SaturatingIncrement(&stats.injections);
      const std::uint32_t bounded_wait_ms =
          std::min(plan_.withhold_ms, deadline_ms);
      if (bounded_wait_ms != 0U) {
        wait(bounded_wait_ms);
      }
      result = plan_.injected_result;
    } else {
      result = operation();
    }

    const std::uint32_t elapsed_ms = clock() - start_ms;
    stats.max_elapsed_ms = std::max(stats.max_elapsed_ms, elapsed_ms);
    if (elapsed_ms > deadline_ms) {
      SaturatingIncrement(&stats.deadline_violations);
    }
    return result;
  }

  MiddlewareBoundaryStats stats(MiddlewareBoundary boundary) const {
    const std::size_t index = ToIndex(boundary);
    return index < stats_.size() ? stats_[index] : MiddlewareBoundaryStats{};
  }

  bool fault_enabled() const { return fault_enabled_; }

 private:
  static constexpr std::size_t ToIndex(MiddlewareBoundary boundary) {
    return static_cast<std::size_t>(boundary);
  }

  static constexpr bool IsValidBoundary(MiddlewareBoundary boundary) {
    return ToIndex(boundary) < kMiddlewareBoundaryCount;
  }

  template <typename Integer>
  static void SaturatingIncrement(Integer* value) {
    if (*value != std::numeric_limits<Integer>::max()) {
      ++(*value);
    }
  }

  bool ShouldInject(MiddlewareBoundary boundary) {
    if (!fault_enabled_ || boundary != plan_.boundary) {
      return false;
    }
    SaturatingIncrement(&matching_occurrences_);
    if (matching_occurrences_ < plan_.first_occurrence ||
        remaining_injections_ == 0U) {
      return false;
    }
    --remaining_injections_;
    if (remaining_injections_ == 0U) {
      fault_enabled_ = false;
    }
    return true;
  }

  MiddlewareFaultPlan plan_{};
  std::array<MiddlewareBoundaryStats, kMiddlewareBoundaryCount> stats_{};
  std::uint32_t matching_occurrences_{0U};
  std::uint32_t remaining_injections_{0U};
  bool fault_enabled_{false};
};

}  // namespace mentor_pi_mcu::app::microros

#endif  // MENTOR_PI_MCU_APP_MICROROS_MIDDLEWARE_FAULT_PROXY_H_
