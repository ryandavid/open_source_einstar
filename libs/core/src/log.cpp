#include "einstar/core/log.hpp"

#include <chrono>
#include <cstdio>
#include <mutex>
#include <print>

namespace einstar::log {
namespace {

std::atomic<Level> g_level{Level::info};
std::mutex g_mutex;
std::function<void(Level, std::string_view)> g_sink;  // guarded by g_mutex

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
    if (g_sink) g_sink(l, message);
}

void set_sink(std::function<void(Level, std::string_view)> sink) {
    std::lock_guard lock(g_mutex);
    g_sink = std::move(sink);
}

std::string_view level_name(Level l) {
    switch (l) {
        case Level::trace: return "trace";
        case Level::debug: return "debug";
        case Level::info: return "info";
        case Level::warn: return "warn";
        case Level::error: return "error";
        case Level::off: return "off";
    }
    return "?";
}

}  // namespace einstar::log
