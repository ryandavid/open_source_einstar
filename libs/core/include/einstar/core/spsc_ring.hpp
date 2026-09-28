#pragma once

#include <atomic>
#include <cstddef>
#include <new>
#include <optional>
#include <vector>

namespace einstar {

// Bounded lock-free single-producer / single-consumer queue.
// try_push fails (rather than overwriting) when full so the producer can count drops.
template <typename T>
class SpscRing {
public:
    explicit SpscRing(std::size_t capacity) : slots_(capacity + 1) {}

    SpscRing(const SpscRing&) = delete;
    SpscRing& operator=(const SpscRing&) = delete;

    [[nodiscard]] bool try_push(T value) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = increment(head);
        if (next == tail_.load(std::memory_order_acquire)) return false;
        slots_[head] = std::move(value);
        head_.store(next, std::memory_order_release);
        return true;
    }

    [[nodiscard]] std::optional<T> try_pop() {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return std::nullopt;
        std::optional<T> out{std::move(slots_[tail])};
        tail_.store(increment(tail), std::memory_order_release);
        return out;
    }

    [[nodiscard]] std::size_t size() const {
        const auto head = head_.load(std::memory_order_acquire);
        const auto tail = tail_.load(std::memory_order_acquire);
        return head >= tail ? head - tail : head + slots_.size() - tail;
    }
    [[nodiscard]] std::size_t capacity() const { return slots_.size() - 1; }
    [[nodiscard]] bool empty() const { return size() == 0; }

private:
    [[nodiscard]] std::size_t increment(std::size_t i) const { return i + 1 == slots_.size() ? 0 : i + 1; }

    std::vector<T> slots_;
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace einstar
