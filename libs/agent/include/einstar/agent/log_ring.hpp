#pragma once

// The app's log, kept for the agent: the last records in a ring, read with a cursor (app.logs).

#include <chrono>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>

#include "einstar/agent/protocol.hpp"
#include "einstar/core/log.hpp"

namespace einstar::agent {

class LogRing {
public:
    explicit LogRing(std::size_t capacity = 5000) : capacity_(capacity) {}
    void add(log::Level level, std::string_view message);
    // Records with seq >= since_seq (at least `min_level`, containing `grep`), at most `limit`; plus
    // next_seq (the cursor for the next read) and how many records were evicted before being read.
    [[nodiscard]] json read(std::uint64_t since_seq, std::size_t limit, log::Level min_level, std::string_view grep) const;

private:
    struct Record {
        std::uint64_t seq;
        double time_s;  // since the ring started
        log::Level level;
        std::string message;
    };
    std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<Record> records_;
    std::uint64_t next_seq_ = 0;
    std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
};

}  // namespace einstar::agent
