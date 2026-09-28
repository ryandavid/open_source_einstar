#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <vector>

namespace einstar {

class Stopwatch {
public:
    using clock = std::chrono::steady_clock;
    Stopwatch() : start_(clock::now()) {}
    void reset() { start_ = clock::now(); }
    [[nodiscard]] double elapsed_ms() const {
        return std::chrono::duration<double, std::milli>(clock::now() - start_).count();
    }

private:
    clock::time_point start_;
};

// Rolling window of timings with percentile queries (not thread-safe).
class TimingStats {
public:
    explicit TimingStats(std::size_t window = 512) : window_(window) { samples_.reserve(window); }

    void add(double ms) {
        if (samples_.size() < window_) {
            samples_.push_back(ms);
        } else {
            samples_[next_] = ms;
            next_ = (next_ + 1) % window_;
        }
    }

    [[nodiscard]] double percentile(double p) const {
        if (samples_.empty()) return 0.0;
        std::vector<double> sorted = samples_;
        const auto idx = static_cast<std::size_t>(std::clamp(p, 0.0, 1.0) * static_cast<double>(sorted.size() - 1));
        std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(idx), sorted.end());
        return sorted[idx];
    }
    [[nodiscard]] double median() const { return percentile(0.5); }
    [[nodiscard]] std::size_t count() const { return samples_.size(); }

private:
    std::size_t window_;
    std::size_t next_ = 0;
    std::vector<double> samples_;
};

}  // namespace einstar
