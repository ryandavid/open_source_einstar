#include "einstar/core/log.hpp"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <print>

namespace einstar::log {
namespace {

std::atomic<Level> g_level{Level::info};
std::mutex g_mutex;

constexpr std::string_view tag(Level l) {
    switch (l) {
        case Level::trace: return "TRC";
        case Level::debug: return "DBG";
        case Level::info: return "INF";
        case Level::warn: return "WRN";
        case Level::error: return "ERR";
        case Level::off: return "OFF";
    }
    return "???";
}

}  // namespace

void set_level(Level l) { g_level.store(l, std::memory_order_relaxed); }
Level level() { return g_level.load(std::memory_order_relaxed); }

void write(Level l, std::string_view message) {
    using namespace std::chrono;
    const auto now = floor<milliseconds>(system_clock::now());
    std::lock_guard lock(g_mutex);
    std::println(stderr, "{:%H:%M:%S} {} {}", now, tag(l), message);
}

}  // namespace einstar::log
