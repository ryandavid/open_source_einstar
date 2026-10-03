#include "einstar/agent/log_ring.hpp"

#include <algorithm>

namespace einstar::agent {

void LogRing::add(log::Level level, std::string_view message) {
    const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    std::lock_guard lock(mutex_);
    records_.push_back({next_seq_++, t, level, std::string(message)});
    while (records_.size() > capacity_) records_.pop_front();
}

json LogRing::read(std::uint64_t since_seq, std::size_t limit, log::Level min_level, std::string_view grep) const {
    std::lock_guard lock(mutex_);
    const std::uint64_t oldest = records_.empty() ? next_seq_ : records_.front().seq;
    json out = json::array();
    std::uint64_t next = std::max(since_seq, oldest);
    for (const auto& r : records_) {
        if (r.seq < since_seq) continue;
        if (out.size() >= limit) break;
        next = r.seq + 1;  // (the cursor moves past filtered-out records too)
        if (r.level < min_level || (!grep.empty() && r.message.find(grep) == std::string::npos)) continue;
        out.push_back({{"seq", r.seq}, {"t", r.time_s}, {"level", log::level_name(r.level)}, {"message", r.message}});
    }
    if (out.size() < limit) next = std::max(next, next_seq_);
    return {{"records", std::move(out)}, {"next_seq", next}, {"dropped", since_seq < oldest ? oldest - since_seq : 0}};
}

}  // namespace einstar::agent
