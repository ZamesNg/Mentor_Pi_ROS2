#ifndef MENTOR_PI_MCU_DOMAIN_FIXED_CONTAINERS_H_
#define MENTOR_PI_MCU_DOMAIN_FIXED_CONTAINERS_H_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace mentor_pi::mcu {

template <typename T>
class SaturatingCounter {
  static_assert(std::is_unsigned<T>::value,
                "A saturating counter requires an unsigned type");

 public:
  constexpr SaturatingCounter(T initial = 0) : value_(initial) {}

  void Increment() {
    T current = value_.load(std::memory_order_relaxed);
    while (current != std::numeric_limits<T>::max() &&
           !value_.compare_exchange_weak(current, static_cast<T>(current + 1U),
                                         std::memory_order_relaxed,
                                         std::memory_order_relaxed)) {
    }
  }

  T value() const { return value_.load(std::memory_order_relaxed); }

 private:
  std::atomic<T> value_;
};

// A wait-free SPSC triple buffer. The producer may replace an unread value;
// the consumer always receives the newest fully copied object. Producer and
// consumer methods must each be called by exactly one execution context.
template <typename T>
class LatestMailbox {
  static_assert(std::is_trivially_copyable<T>::value,
                "Mailbox payloads must be trivially copyable");

 public:
  LatestMailbox() = default;
  LatestMailbox(const LatestMailbox&) = delete;
  LatestMailbox& operator=(const LatestMailbox&) = delete;

  // Returns true when this publication replaces an unread publication.
  bool Publish(const T& value) {
    buffers_[producer_index_] = value;
    const std::uint8_t previous_state = middle_state_.exchange(
        static_cast<std::uint8_t>(producer_index_ | kUnreadBit),
        std::memory_order_acq_rel);
    producer_index_ = static_cast<std::uint8_t>(previous_state & kIndexMask);
    return (previous_state & kUnreadBit) != 0U;
  }

  bool ConsumeLatest(T* value) {
    if (value == nullptr || !has_unread()) {
      return false;
    }
    // Index and unread state share one atomic byte. Keeping them together
    // prevents a publication between a separate dirty-flag clear and index
    // swap from being consumed twice or followed by an older buffer.
    const std::uint8_t previous_state =
        middle_state_.exchange(consumer_index_, std::memory_order_acq_rel);
    consumer_index_ = static_cast<std::uint8_t>(previous_state & kIndexMask);
    if ((previous_state & kUnreadBit) == 0U) {
      // A producer-side discard won the race after has_unread(). Ownership
      // still rotated safely, but the discarded buffer is not a value.
      return false;
    }
    *value = buffers_[consumer_index_];
    return true;
  }

  // Producer-side operation. Atomically makes the current publication
  // unavailable without using or modifying the consumer-owned index. Returns
  // true only if this call won over a concurrent ConsumeLatest().
  bool DiscardLatest() {
    const std::uint8_t previous_state =
        middle_state_.fetch_and(kIndexMask, std::memory_order_acq_rel);
    return (previous_state & kUnreadBit) != 0U;
  }

  bool has_unread() const {
    return (middle_state_.load(std::memory_order_acquire) & kUnreadBit) != 0U;
  }

 private:
  static constexpr std::uint8_t kIndexMask = 0x03U;
  static constexpr std::uint8_t kUnreadBit = 0x80U;
  std::array<T, 3> buffers_{};
  std::atomic<std::uint8_t> middle_state_{1};
  std::uint8_t producer_index_{2};
  std::uint8_t consumer_index_{0};
};

// A bounded lock-free queue based on per-cell sequence numbers. PushDropOldest
// allows one producer and one consumer, and safely lets the producer claim a
// dequeue operation when full. Capacity must be a power of two.
template <typename T, std::size_t Capacity>
class DropOldestQueue {
  static_assert(Capacity >= 2U, "Queue capacity must be at least two");
  static_assert((Capacity & (Capacity - 1U)) == 0U,
                "Queue capacity must be a power of two");
  static_assert(std::is_trivially_copyable<T>::value,
                "Queue payloads must be trivially copyable");

 public:
  DropOldestQueue() {
    for (std::size_t index = 0; index < Capacity; ++index) {
      cells_[index].sequence.store(index, std::memory_order_relaxed);
    }
  }
  DropOldestQueue(const DropOldestQueue&) = delete;
  DropOldestQueue& operator=(const DropOldestQueue&) = delete;

  // Returns true if an older entry was removed to retain value.
  bool PushDropOldest(const T& value) {
    bool dropped = false;
    while (!TryPush(value)) {
      T discarded{};
      if (TryPop(&discarded)) {
        dropped = true;
        dropped_count_.Increment();
      }
    }
    return dropped;
  }

  bool TryPop(T* value) {
    if (value == nullptr) {
      return false;
    }
    std::size_t position = dequeue_position_.load(std::memory_order_relaxed);
    for (;;) {
      Cell& cell = cells_[position & kIndexMask];
      const std::size_t sequence =
          cell.sequence.load(std::memory_order_acquire);
      const std::intptr_t difference =
          static_cast<std::intptr_t>(sequence) -
          static_cast<std::intptr_t>(position + 1U);
      if (difference == 0) {
        if (dequeue_position_.compare_exchange_weak(
                position, position + 1U, std::memory_order_relaxed,
                std::memory_order_relaxed)) {
          *value = cell.value;
          cell.sequence.store(position + Capacity, std::memory_order_release);
          return true;
        }
      } else if (difference < 0) {
        return false;
      } else {
        position = dequeue_position_.load(std::memory_order_relaxed);
      }
    }
  }

  std::uint32_t dropped_count() const { return dropped_count_.value(); }

 private:
  struct Cell {
    std::atomic<std::size_t> sequence{0};
    T value{};
  };

  bool TryPush(const T& value) {
    std::size_t position = enqueue_position_.load(std::memory_order_relaxed);
    for (;;) {
      Cell& cell = cells_[position & kIndexMask];
      const std::size_t sequence =
          cell.sequence.load(std::memory_order_acquire);
      const std::intptr_t difference = static_cast<std::intptr_t>(sequence) -
                                       static_cast<std::intptr_t>(position);
      if (difference == 0) {
        if (enqueue_position_.compare_exchange_weak(
                position, position + 1U, std::memory_order_relaxed,
                std::memory_order_relaxed)) {
          cell.value = value;
          cell.sequence.store(position + 1U, std::memory_order_release);
          return true;
        }
      } else if (difference < 0) {
        return false;
      } else {
        position = enqueue_position_.load(std::memory_order_relaxed);
      }
    }
  }

  static constexpr std::size_t kIndexMask = Capacity - 1U;
  std::array<Cell, Capacity> cells_{};
  std::atomic<std::size_t> enqueue_position_{0};
  std::atomic<std::size_t> dequeue_position_{0};
  SaturatingCounter<std::uint32_t> dropped_count_{};
};

}  // namespace mentor_pi::mcu

#endif  // MENTOR_PI_MCU_DOMAIN_FIXED_CONTAINERS_H_
